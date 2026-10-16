"""Small control messages only. This module cannot transport EMG samples."""
import ctypes
import hmac
import json
import os
import socket
import threading
import time
import uuid

PROTOCOL = 'hipexo-emg-control/1'
MAX_LINE = 16384


def wall_ns():
    if os.name == 'nt':
        value = ctypes.c_ulonglong()
        ctypes.windll.kernel32.GetSystemTimePreciseAsFileTime(ctypes.byref(value))
        return (value.value - 116444736000000000) * 100
    return time.time_ns()


def stamp():
    before = time.perf_counter_ns()
    wall = wall_ns()
    after = time.perf_counter_ns()
    return dict(wall_ns=wall, mono_ns=(before + after)//2, read_bracket_ns=after-before)


def encode(message):
    data = json.dumps(message, ensure_ascii=False, allow_nan=False).encode('utf-8') + b'\n'
    if len(data) > MAX_LINE:
        raise ValueError('Control message exceeds 16 KiB')
    return data


def receive(stream):
    data = stream.readline(MAX_LINE + 1)
    if not data:
        raise ConnectionError('Control connection closed; Windows acquisition may still be running')
    if len(data) > MAX_LINE or not data.endswith(b'\n'):
        raise ValueError('Invalid control message length')
    message = json.loads(data)
    if not isinstance(message, dict):
        raise ValueError('Expected control object')
    return message


class ControlServer:
    """One authenticated client. Disconnect NEVER stops local acquisition.

    dispatch must marshal commands to the owning GUI thread; snapshot must only
    read a pre-built, thread-safe status object. Neither accesses SDK samples.
    """
    def __init__(self, token, dispatch, snapshot, host='0.0.0.0', port=8766):
        self.token, self.dispatch, self.snapshot = token, dispatch, snapshot
        self.done = threading.Event()
        self.connected = False
        self.sock = None
        self.listener = socket.socket()
        if os.name == 'nt':
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        self.listener.bind((host, port))
        self.port = self.listener.getsockname()[1]
        self.listener.listen(1)
        self.listener.settimeout(.5)
        self.thread = threading.Thread(target=self._serve, name='EMG-control-only', daemon=True)
        self.thread.start()

    def _serve(self):
        while not self.done.is_set():
            try:
                sock, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.sock = sock
            try:
                sock.settimeout(10)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with sock.makefile('rb') as stream:
                    hello = receive(stream)
                    token = hello.get('token')
                    if hello.get('type') != 'hello' or hello.get('protocol') != PROTOCOL:
                        raise ValueError('Control-only protocol required on port 8766')
                    if not isinstance(token, str) or not hmac.compare_digest(token.encode(), self.token.encode()):
                        raise ValueError('Connection token mismatch')
                    sock.sendall(encode(dict(type='hello_ack', id=hello.get('id'), protocol=PROTOCOL,
                                            data_transport=False, status=self.snapshot())))
                    self.connected = True
                    while not self.done.is_set():
                        message = receive(stream)
                        received = stamp()
                        kind = message.get('type')
                        if kind not in ('ping', 'status', 'start', 'stop', 'mark'):
                            raise ValueError('Only control commands are supported')
                        ident = message.get('id')
                        if not isinstance(ident, str) or not 0 < len(ident) <= 128:
                            raise ValueError('Invalid request ID')
                        try:
                            if kind == 'ping':
                                result = dict(t2_ns=received['wall_ns'], t3_ns=wall_ns())
                            elif kind == 'status':
                                result = dict(status=self.snapshot())
                            else:
                                message['received'] = received
                                result = self.dispatch(message)
                            reply = dict(result, type=kind+'_ack', id=ident)
                        except Exception as exc:
                            reply = dict(type='error', id=ident, message=str(exc))
                        sock.sendall(encode(reply))
            except (OSError, ValueError, ConnectionError):
                pass
            finally:
                self.connected = False
                sock.close()
                self.sock = None

    def close(self):
        self.done.set()
        self.listener.close()
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(1)


class ControlClient:
    """Synchronous client; call from a worker thread, not a Qt UI thread."""
    def __init__(self, host, port, token):
        self.host, self.port, self.token = host, int(port), token
        self.sock = self.stream = None
        self.sync = None
        self.messages_received = 0

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        # Legacy GUI can wait up to 60 seconds for its final CSV writer.
        self.sock.settimeout(90)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.stream = self.sock.makefile('rb')
        reply = self.request('hello', protocol=PROTOCOL, token=self.token)
        if reply.get('protocol') != PROTOCOL or reply.get('data_transport') is not False:
            self.close()
            raise RuntimeError('Not a control-only Windows endpoint')
        return reply['status']

    def request(self, kind, **fields):
        ident = uuid.uuid4().hex
        try:
            self.sock.sendall(encode(dict(fields, type=kind, id=ident)))
            reply = receive(self.stream)
            self.messages_received += 1
            if reply.get('id') != ident:
                raise ValueError('Mismatched reply; reconnect and query status')
            if reply.get('type') == 'error':
                raise RuntimeError(reply.get('message', 'Control error'))
            if reply.get('type') != kind+'_ack':
                raise ValueError('Unexpected reply; EMG data is prohibited')
            return reply
        except (OSError, ValueError, ConnectionError):
            self.close()
            raise

    def synchronize(self, probes=7):
        samples = []
        for _ in range(probes):
            t1 = wall_ns()
            reply = self.request('ping')
            t4 = wall_ns()
            t2, t3 = reply['t2_ns'], reply['t3_ns']
            rtt = (t4-t1)-(t3-t2)
            if rtt >= 0:
                samples.append(dict(offset_ns=((t2-t1)+(t3-t4))//2, rtt_ns=rtt,
                                    t1_ns=t1, t2_ns=t2, t3_ns=t3, t4_ns=t4))
        if not samples:
            raise RuntimeError('Clock changed during synchronization')
        self.sync = dict(min(samples, key=lambda x:x['rtt_ns']),
                         method='minimum_RTT_four_timestamps', offset_direction='Windows_minus_Ubuntu',
                         physical_sample_sync=False)
        return self.sync

    def close(self):
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.stream:
            self.stream.close()
        if self.sock:
            self.sock.close()
        self.sock = self.stream = None

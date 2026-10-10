"""Bounded background CSV recorder, independent of display-ring eviction."""
import csv
import queue
import threading
import time


class FrameRecorder:
    def __init__(self, path, on_error=None, capacity=4096):
        self.path, self.on_error = path, on_error
        self.error = None
        self.stats = {}
        self._queue = queue.Queue(maxsize=capacity)
        self._stop = threading.Event()
        self._fh = open(path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._fh)
        self._writer.writerow(['t_ms', 'stream', 'field', 'sample_id', 't_mono_ns', 'value'])
        self._thread = threading.Thread(target=self._run, name='hipexo-csv', daemon=True)
        self._thread.start()

    def _fail(self, message):
        if self.error is None:
            self.error = message
            if self.on_error:
                self.on_error(message)

    def enqueue(self, rows, summary=None):
        if self.error or self._stop.is_set():
            return False
        try:
            self._queue.put_nowait((rows,summary))
            return True
        except queue.Full:
            self._fail('Recording queue full; recording stopped with missing data. Start a new file after checking disk throughput.')
            self._stop.set()
            return False

    def _run(self):
        last_flush = time.monotonic()
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=.1)
                except queue.Empty:
                    item = None
                if item is not None:
                    rows,summary = item
                    for start in range(0, len(rows), 512):
                        self._writer.writerows(rows[start:start + 512])
                        if len(rows) > 512:
                            time.sleep(0)  # release CPU between large backlog chunks
                    if summary and rows:
                        stream=summary['stream']
                        if stream not in self.stats:
                            self.stats[stream]=dict(summary)
                        else:
                            current=self.stats[stream]
                            for key in ('samples','valid_samples'):
                                current[key]+=summary[key]
                            current['first_t_ms']=min(current['first_t_ms'],summary['first_t_ms'])
                            current['last_t_ms']=max(current['last_t_ms'],summary['last_t_ms'])
                            current['fields']=sorted(set(current['fields'])|set(summary['fields']))
                            current['simulated']=current['simulated'] or summary['simulated']
                if time.monotonic() - last_flush >= 1.0:
                    self._fh.flush()
                    last_flush = time.monotonic()
        except Exception as exc:
            self._fail(f'CSV write failed: {exc}')
        finally:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception as exc:
                self._fail(f'CSV close failed: {exc}')

    def stop(self, timeout=5):
        # Caller serializes this against enqueue using DataManager's lock.
        self._stop.set()
        self._thread.join(timeout)
        if self._thread.is_alive():
            self._fail('CSV is still draining; do not close the application yet')
            return False
        return self.error is None

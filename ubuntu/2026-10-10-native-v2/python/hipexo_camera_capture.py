"""RealSense I/O isolated from acquisition/UI GIL. Launched only in image mode."""
import argparse,json,socket,struct,time,uuid
import numpy as np

def send(sock,meta,raw=b''):
    header=json.dumps(meta,allow_nan=False).encode()
    sock.sendall(struct.pack('!II',len(header),len(raw))+header+raw)

def run(fd,fps):
    import pyrealsense2 as rs
    sock=socket.socket(fileno=fd);sock.settimeout(2.)
    pipeline=None
    try:
        while True:
            if pipeline is None:
                try:
                    if not len(rs.context().query_devices()):raise RuntimeError('RealSense not detected')
                    pipeline=rs.pipeline();cfg=rs.config();cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,fps)
                    profile=pipeline.start(cfg);scale=float(profile.get_device().first_depth_sensor().get_depth_scale());stream=uuid.uuid4().hex
                except RuntimeError as exc:
                    pipeline=None;send(sock,{'error':str(exc)});time.sleep(1.);continue
            try:
                frames=pipeline.wait_for_frames(timeout_ms=1000);wall,mono=time.time_ns(),time.perf_counter_ns();frame=frames.get_depth_frame()
                if not frame:raise RuntimeError('No depth frame')
                depth=np.asanyarray(frame.get_data());intr=frame.profile.as_video_stream_profile().intrinsics
                meta=dict(host_frame_received_wall_ns=wall,host_frame_received_mono_ns=mono,
                    device_frame_number=frame.get_frame_number(),device_timestamp_ms=frame.get_timestamp(),
                    device_timestamp_domain=str(frame.get_frame_timestamp_domain()),camera_stream_id=stream,
                    timestamp_semantics='host_frame_received',depth_scale_m=scale,image_kind='depth_uint16',
                    intrinsics={k:float(getattr(intr,k)) for k in ['fx','fy','ppx','ppy']},shape=list(depth.shape))
                send(sock,meta,depth.tobytes())
            except RuntimeError as exc:
                send(sock,{'error':str(exc)})
                try:pipeline.stop()
                except Exception:pass
                pipeline=None
    except (BrokenPipeError,ConnectionError,OSError):pass
    finally:
        if pipeline:
            try:pipeline.stop()
            except Exception:pass
        sock.close()
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--fd',type=int,required=True);p.add_argument('--fps',type=int,default=15);a=p.parse_args();run(a.fd,a.fps)

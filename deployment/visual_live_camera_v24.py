"""One D435 owner, continuous display and lossless, frame-paired measurements."""
import base64
import hashlib
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from urllib.request import urlopen
from visual_detection_confirm_v24 import ConfirmedDetections

ROOT = Path(__file__).resolve().parents[1]


class LiveCamera:
    def __init__(self, python, device):
        self.python, self.device = python, device
        self.lock = threading.Condition()
        self.closing = threading.Event()
        self.image = self.sample = self.intrinsics = None
        self.frame = self.timestamp_ms = self.captured = 0
        self.camera_error = self.inference_error = None
        self.client = None
        self.display_confirmation = ConfirmedDetections()

    def start(self):
        self.capture_thread = threading.Thread(target=self._capture, daemon=True)
        self.inference_thread = threading.Thread(target=self._infer, daemon=True)
        self.capture_thread.start()
        self.inference_thread.start()

    def _capture(self):
        pipeline = None
        try:
            import numpy as np
            import pyrealsense2 as rs
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device('344322075008')
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            profile = pipeline.start(config)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            with self.lock:
                self.intrinsics = dict(fx=intr.fx, fy=intr.fy, ppx=intr.ppx, ppy=intr.ppy,
                    coeffs=list(intr.coeffs), model=str(intr.model), width=intr.width,
                    height=intr.height, serial='344322075008')
                self.lock.notify_all()
            while not self.closing.is_set():
                color = pipeline.wait_for_frames(5000).get_color_frame()
                if not color:
                    continue
                image = np.asanyarray(color.get_data()).copy()
                with self.lock:
                    self.image, self.frame = image, int(color.get_frame_number())
                    self.timestamp_ms, self.captured = float(color.get_timestamp()), time.monotonic()
                    self.lock.notify_all()
        except Exception as exc:
            with self.lock:
                self.camera_error = repr(exc)
                self.lock.notify_all()
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception as exc:
                    self.camera_error = 'camera cleanup: ' + repr(exc)

    def _infer(self):
        try:
            from battery_grasp.inference_client import InferenceClient
            from cylinder_color_policy_v23 import specification
            thresholds = specification()['class_confidences']
            owner=self
            class TrackedInferenceClient(InferenceClient):
                def _read(self):
                    # Register after Popen exists, before waiting for model
                    # readiness, so closing the service can also cancel startup.
                    owner.client=self
                    if owner.closing.is_set():
                        self.close()
                        self.messages.put(None)
                        return
                    try:
                        super()._read()
                    except (OSError,ValueError):
                        if not owner.closing.is_set():raise
                        self.messages.put(None)
            self.client = TrackedInferenceClient(self.python,
                str(ROOT/'src/battery_grasp/scripts/yolo_worker.py'),
                str(ROOT/'deployment/yolov5-runtime'),
                str(ROOT/'src/battery_grasp/models/battery_v3_camera.torchscript'),
                device=self.device, class_confidences=thresholds)
            previous = -1
            while not self.closing.is_set():
                with self.lock:
                    self.lock.wait_for(lambda: self.closing.is_set() or self.camera_error or
                        (self.image is not None and self.frame > previous), timeout=1)
                    if self.camera_error or self.closing.is_set():
                        return
                    if self.image is None or self.frame <= previous:
                        continue
                    image = self.image.copy()
                    frame, stamp, captured = self.frame, self.timestamp_ms, self.captured
                detections = self.client.detect(image)
                raw = image.tobytes()
                meta = dict(frame_number=frame, timestamp_ms=stamp, captured_monotonic=captured,
                    inferred_monotonic=time.monotonic(), shape=list(image.shape),
                    sha256=hashlib.sha256(raw).hexdigest(), detections=detections,
                    intrinsics=self.intrinsics, image_encoding='raw_bgr8_lossless')
                with self.lock:
                    meta['display_confirmation']=self.display_confirmation.update(detections,frame,captured)
                    self.sample = (meta, raw)
                    previous = frame
                    self.lock.notify_all()
        except Exception as exc:
            with self.lock:
                if not self.closing.is_set():self.inference_error = repr(exc)
                self.lock.notify_all()
        finally:
            if self.client is not None:
                self.client.close()

    def metadata(self):
        with self.lock:
            detection_age=time.monotonic()-self.sample[0]['captured_monotonic'] if self.sample else None
            counts=dict(black=0,red=0,white=0,other=0,total=0)
            fresh=(detection_age is not None and detection_age<=2
                and not self.camera_error and not self.inference_error)
            confirmation=self.sample[0]['display_confirmation'] if self.sample is not None else {}
            if fresh:counts=dict(confirmation['counts'])
            return dict(intrinsics=self.intrinsics, frame_number=self.frame,
                sample_frame=self.sample[0]['frame_number'] if self.sample else -1,
                server_monotonic=time.monotonic(),
                frame_age_seconds=time.monotonic()-self.captured if self.captured else None,
                camera_error=self.camera_error, inference_error=self.inference_error,
                detection_counts=counts,detection_age_seconds=detection_age,
                detection_fresh=fresh,detection_confirmation_seconds=2.0,
                detection_pending_count=confirmation.get('pending_count',0) if fresh else None,
                measurement_ready=self.sample is not None and not self.camera_error and not self.inference_error)

    def measurement(self, after, since, timeout=120):
        with self.lock:
            ready = self.lock.wait_for(lambda: self.closing.is_set() or self.camera_error or
                self.inference_error or (self.sample is not None and
                self.sample[0]['frame_number'] > after and
                self.sample[0]['captured_monotonic'] >= since), timeout=timeout)
            if self.camera_error or self.inference_error:
                raise RuntimeError(self.camera_error or self.inference_error)
            if not ready or self.closing.is_set():
                raise TimeoutError('No fresh, frame-paired measurement')
            meta, raw = self.sample
            return meta, raw

    def render(self, status):
        import cv2
        import numpy as np
        with self.lock:
            image = self.image.copy() if self.image is not None else np.zeros((720,1280,3), np.uint8)
            sample = self.sample
            age = time.monotonic()-self.captured if self.captured else None
            error = self.camera_error or self.inference_error
            frame = self.frame
        # Display boxes may be older than the live image. Measurement consumers
        # receive the corresponding original frame, never this annotated view.
        if sample is not None:
            box_age = time.monotonic()-sample[0]['captured_monotonic']
            if box_age <= 2:
                tracks={row['detection_index']:row for row in sample[0]['display_confirmation']['tracks']}
                for index,d in enumerate(sample[0]['detections']):
                    x,y,w,h = [int(v) for v in d['bbox']]
                    track=tracks.get(index,{})
                    confirmed=track.get('confirmed',False)
                    color=(80,220,100) if confirmed else (0,190,255)
                    label=('CONFIRMED' if confirmed else 'PENDING %.1f/2s'%track.get('visible_seconds',0))
                    cv2.rectangle(image,(x,y),(x+w,y+h),color,2)
                    cv2.putText(image,'%s %.2f %s'%(d['class_name'],d['confidence'],label),
                        (max(0,x),max(18,y-5)),0,.50,color,1)
            cv2.putText(image,'YOLO boxes age %.2fs'%box_age,(15,690),0,.55,(0,230,255),1)
        locked = status.get('locked_target')
        if locked and status.get('busy'):
            box = locked.get('measurement_bbox_px')
            if box:
                x,y,w,h = [int(v) for v in box]
                cv2.rectangle(image,(x,y),(x+w,y+h),(255,180,0),3)
                cv2.putText(image,'LOCKED PICK TARGET (capture position)',(max(0,x),max(40,y-22)),0,.5,(255,180,0),1)
        stale = age is None or age > 1 or bool(self.camera_error)
        cv2.putText(image,('STALE / CAMERA WAIT' if stale else 'LIVE D435 #%d  %.2fs'%(frame,age)),
            (15,30),0,.8,(0,0,255) if stale else (0,255,0),2)
        cv2.putText(image,str(status.get('phase') or status.get('state','IDLE'))[:110],
            (15,60),0,.65,(0,230,255),2)
        if error:
            cv2.putText(image,str(error)[:130],(15,90),0,.5,(0,0,255),1)
        return image

    def close(self):
        self.closing.set()
        with self.lock:
            self.lock.notify_all()
        # This is only the pure inference subprocess, never the robot SDK worker.
        if self.client is not None:
            self.client.close()
        self.capture_thread.join(timeout=6)
        self.inference_thread.join(timeout=6)


class SharedMeasurement:
    """Small rs-pipeline surface plus exact same-frame inference for v23 geometry."""
    def __init__(self, endpoint):
        self.endpoint = endpoint.rstrip('/')
        self.meta = None
        self.last = -1
        self.since = None

    def start(self, _config=None):
        metadata = json.loads(urlopen(self.endpoint+'/api/camera',timeout=3).read())
        if not metadata['intrinsics'] or metadata.get('camera_error'):
            raise RuntimeError('Live camera not ready: '+str(metadata))
        self.last = metadata['sample_frame']
        self.since = metadata['server_monotonic']
        self.intr = SimpleNamespace(**metadata['intrinsics'])
        return self

    def get_stream(self, _stream):
        return self

    def as_video_stream_profile(self):
        return self

    def get_intrinsics(self):
        return self.intr

    def wait_for_frames(self, _timeout):
        import numpy as np
        url = self.endpoint+'/api/sample?after=%d&since=%.9f'%(self.last,self.since)
        # The original inference bridge allows 120s per request. Keep that
        # bound rather than imposing a new machine-speed-dependent cutoff.
        with urlopen(url,timeout=125) as response:
            self.meta=json.loads(base64.b64decode(response.headers['X-Frame-Meta']))
            raw=response.read()
        if hashlib.sha256(raw).hexdigest()!=self.meta['sha256']:
            raise RuntimeError('Lossless frame digest differs')
        if self.meta['frame_number']<=self.last or self.meta['captured_monotonic']<self.since:
            raise RuntimeError('Repeated or pre-session measurement frame')
        self.last=self.meta['frame_number']
        self.image=np.frombuffer(raw,dtype=np.uint8).reshape(self.meta['shape']).copy()
        return self

    def get_color_frame(self):
        return self

    def get_data(self):
        return self.image

    def get_frame_number(self):
        return self.meta['frame_number']

    def get_timestamp(self):
        return self.meta['timestamp_ms']

    def detect(self, image):
        if self.meta is None or hashlib.sha256(image.tobytes()).hexdigest()!=self.meta['sha256']:
            raise RuntimeError('Inference requested for a different camera frame')
        return self.meta['detections']

    def stop(self):
        pass  # Detach this consumer. The display/camera service remains live.

    close = stop

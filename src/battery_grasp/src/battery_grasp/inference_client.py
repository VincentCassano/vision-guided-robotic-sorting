"""Bounded subprocess bridge: ROS and model use separate Python environments."""
import base64
import json
import queue
import subprocess
import threading
from pathlib import Path


class InferenceClient:
    def __init__(self, python, worker, repository, weights, device='cpu', timeout=120, square_confidence=0.25, class_confidences=None):
        if not Path(python).is_absolute() or not Path(python).is_file():
            raise ValueError('inference_python must be an existing absolute executable path')
        if timeout <= 0:
            raise ValueError('Inference timeout must be positive')
        self.timeout = timeout
        self.lock = threading.Lock()
        self.messages = queue.Queue()
        command = [python, worker, '--repository', repository,
                                         '--weights', weights, '--device', device,
                                         '--square-confidence', str(square_confidence)]
        if class_confidences is not None:
            command += ['--class-confidences'] + [str(v) for v in class_confidences]
        self.process = subprocess.Popen(command,
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()
        try:
            if self._receive() != {'ready': True}:
                raise RuntimeError('Unexpected inference startup response')
        except Exception:
            self.close()
            raise

    def _read(self):
        for line in self.process.stdout:
            self.messages.put(line)
        self.messages.put(None)

    def _receive(self):
        try:
            line = self.messages.get(timeout=self.timeout)
        except queue.Empty:
            self.close()
            raise RuntimeError('Inference timed out; worker terminated')
        if line is None:
            raise RuntimeError('Inference worker exited')
        value = json.loads(line)
        if 'error' in value:
            raise RuntimeError(value['error'])
        return value

    def detect(self, bgr):
        import cv2
        import numpy as np
        if bgr is None or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('Expected a uint8 BGR image')
        with self.lock:
            ok, encoded = cv2.imencode('.png', bgr)
            if not ok:
                raise ValueError('Image encoding failed')
            self.process.stdin.write(json.dumps({'image': base64.b64encode(encoded).decode('ascii')}) + '\n')
            self.process.stdin.flush()
            return self._receive()['detections']

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.process.stdin.close()
        self.process.stdout.close()

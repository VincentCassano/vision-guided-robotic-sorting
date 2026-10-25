"""Exact-frame RGB-D projection for simulation diagnostics, with no robot targets."""
import numpy as np
from .perception import project_pixel
from .sorting_core import UnsafePlan, require


class FrameMatcher:
    def __init__(self, capacity=4):
        self.buffers = [{}, {}, {}]
        self.capacity = capacity
        self.last_stamp = -1

    def add(self, kind, stamp, value):
        require(type(stamp) is int and stamp > 0, 'Invalid nanosecond timestamp')
        if stamp <= self.last_stamp:
            return None
        self.buffers[kind][stamp] = value
        for buffer in self.buffers:
            for old in sorted(buffer)[:-self.capacity]:
                del buffer[old]
        if not all(stamp in buffer for buffer in self.buffers):
            return None
        result = tuple(buffer.pop(stamp) for buffer in self.buffers)
        self.last_stamp = stamp
        for buffer in self.buffers:
            for old in list(buffer):
                if old <= stamp:
                    del buffer[old]
        return result


def project_frame(detections, depth, camera):
    require(type(detections.get('schema_version')) is int and detections['schema_version'] == 1, 'Unsupported detection schema')
    require(type(detections['stamp_ns']) is int and detections['stamp_ns'] > 0, 'Invalid detection timestamp')
    require(detections['stamp_ns'] == depth['stamp_ns'] == camera['stamp_ns'], 'Timestamp mismatch')
    require(bool(detections['frame_id']) and detections['frame_id'] == depth['frame_id'] == camera['frame_id'], 'Frame mismatch')
    pixels = depth['image']
    shape = (camera['height'], camera['width'])
    require(pixels.ndim == 2 and pixels.shape == shape, 'Depth/CameraInfo size mismatch')
    require((detections['image_height'], detections['image_width']) == shape, 'RGB/depth size mismatch')
    if depth['encoding'] == '16UC1':
        require(pixels.dtype == np.uint16, '16UC1 requires uint16 millimetres')
        scale = 0.001
    elif depth['encoding'] == '32FC1':
        require(pixels.dtype == np.float32, '32FC1 requires float32 metres')
        scale = 1.0
    else:
        raise UnsafePlan('Unsupported depth encoding')
    k = np.asarray(camera['K'], dtype=float)
    require(k.shape == (9,) and np.isfinite(k).all(), 'Invalid camera matrix')
    require(k[0] > 0 and k[4] > 0 and np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1]), 'Expected rectified pinhole intrinsics')
    distortion = np.asarray(camera.get('D', []), dtype=float)
    require(np.isfinite(distortion).all() and np.allclose(distortion, 0), 'Input must be rectified')
    require(isinstance(detections['detections'], list), 'Invalid detections list')
    objects, rejected = [], []
    for index, item in enumerate(detections['detections']):
        try:
            point = project_pixel(item, pixels, [k[0], k[4], k[2], k[5]], 0.3, 1.5, 2, scale)
            objects.append(dict(item, position_optical_m=point.tolist()))
        except (UnsafePlan, KeyError, ValueError, TypeError) as exc:
            rejected.append({'detection_index': index, 'reason': str(exc)})
    return {'schema_version': 1, 'stamp_ns': detections['stamp_ns'], 'stamp': detections['stamp_ns']/1e9,
            'frame_id': detections['frame_id'], 'simulation_only': True, 'depth_source': 'synthetic_test',
            'execution_ready': False, 'valid': not rejected, 'objects': objects, 'rejected': rejected}

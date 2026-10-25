"""Simulation-only base-frame observations and conservative association."""
import numpy as np
from .sorting_core import require

CATEGORIES = {0: 'NORMAL_BLACK', 1: 'DEFECT_RED', 2: 'DEFECT_WHITE', 3: 'UNSUPPORTED_SQUARE'}


class ObservationTracker:
    def __init__(self, rotation, translation, source_frame='battery_replay_optical_frame',
                 gate=0.02, max_gap=1.0, max_age=10.0):
        self.rotation = np.asarray(rotation, dtype=float)
        self.translation = np.asarray(translation, dtype=float)
        require(self.rotation.shape == (3, 3) and np.isfinite(self.rotation).all(), 'Invalid rotation')
        require(np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-8)
                and np.isclose(np.linalg.det(self.rotation), 1.0), 'Expected proper orthonormal rotation')
        require(self.translation.shape == (3,) and np.isfinite(self.translation).all(), 'Invalid translation')
        require(isinstance(source_frame, str) and bool(source_frame), 'Invalid source frame')
        require(all(np.isfinite(x) and x > 0 for x in (gate, max_gap, max_age)), 'Invalid tracking limits')
        self.source_frame, self.gate = source_frame, gate
        self.max_gap_ns, self.max_age_ns = int(max_gap * 1e9), int(max_age * 1e9)
        self.last_stamp, self.next_id, self.previous = 0, 1, []

    def invalidate(self):
        self.previous = []

    def process(self, frame, now_ns):
        try:
            return self._process(frame, now_ns)
        except (ValueError, KeyError, TypeError):
            self.invalidate()
            raise

    def _process(self, frame, now_ns):
        require(type(frame.get('schema_version')) is int and frame['schema_version'] == 1, 'Unsupported schema')
        stamp = frame['stamp_ns']
        require(type(stamp) is int and stamp > self.last_stamp, 'Duplicate or out-of-order timestamp')
        require(type(now_ns) is int and 0 <= now_ns - stamp <= self.max_age_ns, 'Stale or future observation')
        require(frame.get('simulation_only') is True and frame.get('execution_ready') is False
                and frame.get('depth_source') == 'synthetic_test', 'Expected simulation diagnostics')
        require(frame.get('valid') is True and frame.get('rejected') == [], 'Invalid source observation')
        require(frame.get('frame_id') == self.source_frame, 'Unexpected optical frame')
        require(isinstance(frame['objects'], list) and len(frame['objects']) <= 100, 'Invalid object count')
        objects = []
        for item in frame['objects']:
            require(isinstance(item, dict), 'Invalid object')
            cid = item['class_id']
            require(type(cid) is int and cid in CATEGORIES and item['category'] == CATEGORIES[cid], 'Invalid class/category')
            point = np.asarray(item['position_optical_m'], dtype=float)
            require(point.shape == (3,) and np.isfinite(point).all(), 'Invalid optical point')
            base = self.rotation @ point + self.translation
            require(np.isfinite(base).all(), 'Invalid base point')
            objects.append(dict(item, position_base_m=base.tolist(), grasp_supported=(cid != 3),
                                track_id=None, association='new'))
        previous = self.previous if stamp - self.last_stamp <= self.max_gap_ns else []
        candidates = [[j for j, old in enumerate(previous)
                       if obj['class_id'] == old['class_id'] and
                       np.linalg.norm(np.array(obj['position_base_m']) - old['position_base_m']) <= self.gate]
                      for obj in objects]
        counts = [sum(j in row for row in candidates) for j in range(len(previous))]
        for i, obj in enumerate(objects):
            nearby = any(k != i and other['class_id'] == obj['class_id'] and
                         np.linalg.norm(np.array(obj['position_base_m']) - other['position_base_m']) <= self.gate
                         for k, other in enumerate(objects))
            row = candidates[i]
            if nearby or len(row) > 1 or any(counts[j] > 1 for j in row):
                obj['association'] = 'ambiguous'
            elif len(row) == 1:
                obj.update(track_id=previous[row[0]]['track_id'], association='matched')
            else:
                obj['track_id'] = self.next_id
                self.next_id += 1
        self.previous = [obj for obj in objects if obj['track_id'] is not None]
        self.last_stamp = stamp
        return {'schema_version': 1, 'stamp_ns': stamp, 'stamp': stamp / 1e9,
                'frame_id': 'base_link', 'source_frame_id': self.source_frame,
                'simulation_only': True, 'depth_source': 'synthetic_test',
                'transform_source': 'configured_simulation', 'execution_ready': False,
                'association_scope': 'consecutive_observations_only',
                'valid': True, 'objects': objects, 'rejected': []}

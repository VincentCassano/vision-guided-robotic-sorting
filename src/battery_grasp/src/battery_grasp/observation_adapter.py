"""Validate diagnostic observations for the isolated simulated task controller."""
import math
from .sorting_core import require, validate_config, validate_objects, slot, inside
from .observation_tracking import CATEGORIES


class ObservationAdapter:
    def __init__(self, cfg):
        validate_config(cfg)
        self.cfg = cfg
        self.reset(0)

    def reset(self, stamp_ns):
        require(type(stamp_ns) is int and stamp_ns >= 0, 'Invalid reset timestamp')
        self.after = self.last_stamp = stamp_ns
        self.previous = {}

    def process(self, frame, now_ns):
        stamp = 0
        try:
            require(isinstance(frame, dict), 'Invalid observation')
            require(type(frame.get('schema_version')) is int and frame['schema_version'] == 1, 'Invalid schema')
            stamp = frame['stamp_ns']
            require(type(stamp) is int and stamp > self.last_stamp, 'Old or duplicate observation')
            require(type(now_ns) is int and 0 <= now_ns-stamp <= int(self.cfg['max_age']*1e9), 'Stale/future observation')
            require(frame.get('simulation_only') is True and frame.get('execution_ready') is False
                    and frame.get('transform_source') == 'configured_simulation'
                    and frame.get('depth_source') == 'synthetic_test', 'Expected simulated diagnostic observation')
            require(frame.get('frame_id') == self.cfg['frame_id']
                    and frame.get('source_frame_id') == self.cfg['camera']['frame_id'], 'Unexpected frame')
            require(frame.get('valid') is True and frame.get('rejected') == [], 'Invalid source observation')
            require(isinstance(frame.get('objects'), list) and len(frame['objects']) <= 9, 'Expected at most nine targets')
            objects, current = [], {}
            for item in frame['objects']:
                cid, track = item['class_id'], item['track_id']
                require(type(cid) is int and cid in (0, 1, 2)
                        and item['category'] == CATEGORIES[cid] and item.get('grasp_supported') is True,
                        'Unsupported sorting category')
                require(type(track) is int and track > 0 and track not in current, 'Missing or duplicate track')
                require(item.get('association') in ('new', 'matched'), 'Ambiguous association')
                point = item['position_base_m']
                require(inside(point, self.cfg['workspace']), 'Target outside workspace')
                row, col, identity = slot(point, self.cfg)
                obj = dict(id=identity, row=row, column=col, category=item['category'], position=list(point))
                old = self.previous.get(track)
                stable = (old is not None and item['association'] == 'matched'
                          and stamp-self.last_stamp <= int(self.cfg['max_age']*1e9)
                          and old['id'] == identity and old['category'] == obj['category']
                          and math.dist(old['position'], point) <= self.cfg['max_target_shift'])
                objects.append(obj)
                current[track] = dict(obj, stable=stable)
            validate_objects(objects, self.cfg)
            self.previous, self.last_stamp = current, stamp
            if not all(obj['stable'] for obj in current.values()):
                return self._output(stamp, False, [], ['Waiting for two stable observations'])
            return self._output(stamp, True, objects, [])
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            self.previous = {}
            return self._output(stamp if type(stamp) is int else 0, False, [], [str(exc)])

    def _output(self, stamp, valid, objects, rejected):
        return dict(schema_version=1, stamp_ns=stamp, stamp=stamp/1e9,
                    frame_id=self.cfg['frame_id'], simulation_only=True, execution_ready=False,
                    adapter='stable_observation_grid', simulation_scene_ready=valid,
                    valid=valid, objects=objects, rejected=rejected)

"""Conservative confirmation across consecutive processed camera observations."""
import math


class TemporalConfirmation:
    def __init__(self):
        self.reset()
        self.next_id = 1

    def reset(self):
        self.previous = []
        self.last_time = None

    @staticmethod
    def close(a, b):
        ax, ay, aw, ah = a['bbox']
        bx, by, bw, bh = b['bbox']
        intersection = max(0, min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
        union = aw*ah+bw*bh-intersection
        distance = math.hypot(a['u']-b['u'],a['v']-b['v'])
        return union > 0 and intersection/union >= .5 and distance <= max(4, .15*min(aw,ah,bw,bh))

    def update(self, detections, stamp):
        if not math.isfinite(stamp):
            self.reset()
            raise ValueError('Invalid observation time')
        if self.last_time is not None and (stamp <= self.last_time or stamp-self.last_time > .5):
            self.reset()
        objects = []
        for d in detections:
            if not all(math.isfinite(float(v)) for v in [*d['bbox'],d['u'],d['v']]) or min(d['bbox'][2:]) <= 0:
                self.reset()
                raise ValueError('Invalid detection geometry')
            objects.append(dict(d))
        matches = [[j for j,p in enumerate(self.previous)
                    if d['class_id']==p['class_id'] and self.close(d,p)] for d in objects]
        counts = [sum(j in row for row in matches) for j in range(len(self.previous))]
        tracks = []
        for i,d in enumerate(objects):
            row=matches[i]
            ambiguous = len(row)>1 or any(counts[j]>1 for j in row) or any(
                k!=i and self.close(d,other) for k,other in enumerate(objects))
            old = self.previous[row[0]] if len(row)==1 and not ambiguous else None
            if old is not None and not self.close(d,old['_anchor']): old=None
            if old is None:
                track_id=self.next_id; self.next_id+=1
                since=stamp; anchor=dict(d)
            else:
                track_id=old['track_id']; since=old['_since']; anchor=old['_anchor']
            elapsed=stamp-since
            confirmed=not ambiguous and elapsed>1.0
            d.update(track_id=track_id,stable_seconds=elapsed,visual_confirmed=confirmed,
                     confirmation_state='CONFIRMED' if confirmed else 'PENDING',execution_ready=False)
            if not ambiguous: tracks.append(dict(d,_since=since,_anchor=anchor))
        self.previous=tracks
        self.last_time=stamp
        return objects

    @staticmethod
    def candidates(objects):
        # This is a vision-only candidate list, not authorization for motion.
        return [d for d in objects if d.get('visual_confirmed') is True
                and d.get('depth_valid') is True and d.get('grasp_supported') is True]

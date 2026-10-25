"""Display-only confirmation: a visible target must persist for at least 2s."""
import math


def overlap(a, b):
    ax,ay,aw,ah=a
    bx,by,bw,bh=b
    area=max(0.,min(ax+aw,bx+bw)-max(ax,bx))*max(0.,min(ay+ah,by+bh)-max(ay,by))
    return area/max(aw*ah+bw*bh-area,1e-12)


class ConfirmedDetections:
    """No polling clock, cumulative frame counter or robot-control decisions."""
    def __init__(self):
        self.required_seconds=2.0
        self.maximum_gap_seconds=2.0
        self.minimum_iou=.30
        self.last_frame=-1
        self.last_capture=None
        self.next_id=1
        self.tracks=[]
        self.snapshot=self._snapshot([])

    def _snapshot(self, rows):
        counts=dict(black=0,red=0,white=0,other=0,total=0)
        for row in rows:
            if row['confirmed']:
                name={0:'black',1:'red',2:'white'}.get(row['class_id'],'other')
                counts[name]+=1
                counts['total']+=1
        return dict(required_visible_seconds=self.required_seconds,
            confirmation_time_source='distinct_frame_captured_monotonic',
            maximum_gap_seconds=self.maximum_gap_seconds,
            counts=counts,pending_count=len(rows)-counts['total'],tracks=rows,
            current_visible_objects_only=True,cumulative_detection_count=False)

    def update(self, detections, frame_number, captured):
        # Re-reading the same frame can never make a pending object mature.
        if frame_number<=self.last_frame:
            return self.snapshot
        if (self.last_capture is not None and
                (captured<=self.last_capture or captured-self.last_capture>self.maximum_gap_seconds)):
            self.tracks=[]
        valid=[]
        for index,detection in enumerate(detections):
            box=detection.get('bbox',[])
            category=detection.get('class_id')
            if (type(category) is not int or len(box)!=4
                    or any(type(v) not in (int,float) or not math.isfinite(v) for v in box)
                    or box[2]<=0 or box[3]<=0):
                continue
            valid.append(dict(detection_index=index,class_id=category,bbox=list(box)))
        candidates=[]
        references={}
        for detection in valid:
            options=[i for i,track in enumerate(self.tracks)
                if detection['class_id']==track['class_id']
                and overlap(detection['bbox'],track['bbox'])>=self.minimum_iou]
            candidates.append(options)
            for i in options:references[i]=references.get(i,0)+1
        tracks=[]
        for detection,options in zip(valid,candidates):
            # Ambiguous overlapping objects do not inherit another object's age.
            if len(options)==1 and references[options[0]]==1:
                previous=self.tracks[options[0]]
                identity,first,observations=previous['id'],previous['first_capture'],previous['observations']+1
            else:
                identity,first,observations=self.next_id,captured,1
                self.next_id+=1
            duration=max(0.,captured-first)
            tracks.append(dict(detection,id=identity,first_capture=first,
                observations=observations,visible_seconds=duration,
                confirmed=observations>=2 and duration>=self.required_seconds))
        # Unmatched targets disappear immediately; a later reappearance starts at zero.
        self.tracks=tracks
        self.last_frame,self.last_capture=frame_number,captured
        self.snapshot=self._snapshot(tracks)
        return self.snapshot

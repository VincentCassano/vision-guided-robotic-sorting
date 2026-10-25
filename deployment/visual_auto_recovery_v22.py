"""Bounded visual-only re-observation; no robot, coordinates or quality overrides."""
import copy
import math

from visual_batch_workspace_v20 import same_detection,foot,front_first


class VisualRecovery:
    def __init__(self,max_attempts=3,preview_seconds=15.,budget_seconds=90.):
        self.max_attempts=max_attempts
        self.preview_seconds=preview_seconds
        self.budget_seconds=budget_seconds
        self.tracks=[];self.visible=[];self.current=None;self.selected_since=None
        self.events=[]

    def observe(self,detections):
        used=set();visible=[]
        for detection in sorted(detections,key=front_first):
            box=detection['bbox']
            matches=[t for t in self.tracks if t['id'] not in used
                and same_detection(box,t['bbox'])]
            if matches:track=min(matches,key=lambda t:math.dist(foot(box),foot(t['bbox'])))
            else:
                track=dict(id=len(self.tracks)+1,bbox=box[:],failed_observations=0)
                self.tracks.append(track)
            track['bbox']=box[:];used.add(track['id']);visible.append(track['id'])
        self.visible=visible

    def choose(self,now):
        eligible=[t for t in self.tracks if t['id'] in self.visible
            and t['failed_observations']<self.max_attempts]
        if not eligible:return None
        track=min(eligible,key=lambda t:(t['failed_observations'],front_first({'bbox':t['bbox']})))
        self.current=track['id'];self.selected_since=now
        return track['bbox'][:]

    def preview_expired(self,now):
        return self.current is not None and self.selected_since is not None and now-self.selected_since>=self.preview_seconds

    def failed(self,reason,now,observation=None,image=None):
        track=next((t for t in self.tracks if t['id']==self.current),None)
        if track is not None:
            track['failed_observations']+=1
            event=dict(target_id=track['id'],bbox=track['bbox'][:],
                failed_observations=track['failed_observations'],reason=copy.deepcopy(reason),
                monotonic=now,observation=observation,RGB_file=image,
                no_motion_or_grip_retry=True,not_added_to_completed_ledger=True)
            self.events.append(event)
        self.current=None;self.selected_since=None

    def exhausted(self):
        return bool(self.visible) and all(t['failed_observations']>=self.max_attempts
            for t in self.tracks if t['id'] in self.visible)

    def summary(self):
        return dict(max_observations_per_target=self.max_attempts,
            preview_observation_seconds=self.preview_seconds,
            observation_budget_seconds=self.budget_seconds,
            visible_target_ids=self.visible[:],targets=copy.deepcopy(self.tracks),
            failed_observations=copy.deepcopy(self.events),
            pending_targets=len(self.visible),
            coordinates_from_tracks=False,quality_thresholds_changed=False,
            actuator_retry=False,completed_count_increased_by_deferral=False)

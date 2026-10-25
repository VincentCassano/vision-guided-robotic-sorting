"""One supported four-flex withdrawal; never authorizes a loaded fault resume."""
import copy
import time
from visual_pick_sort_v7_hand import VisualHand,GripTransport
from visual_pick_sort_v5_source import RECIPE,PREPARE


class RecoveryTransport(GripTransport):
    def grant(self,address,values,proof=None):
        authority=self._phase_authority
        if (address!=1135 or values!=[8000]*4 or self.approved is not None
                or self._attempted_writes or self.broken or self.terminal_latched or self._completed
                or not getattr(authority,'supported_open_permission',False)
                or not isinstance(proof,dict)
                or any(abs(a-b)>2 for a,b in zip(proof['targets'],RECIPE))
                or any(not 8000<=p<=cap+150 for p,cap in zip(proof['positions'][:4],RECIPE[:4]))):
            raise RuntimeError('Only one supported v4 four-flex withdrawal permitted')
        self.approved=(address,tuple(values));self.motion_enabled=True


class RecoveryHand(VisualHand):
    def before_open(self):
        if not self._configuration_checked:self.configuration()
        self._critical_configuration();rows=[self.snapshot() for _ in range(3)]
        for row in rows:
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],RECIPE))
                    or any(not 8000<=p<=cap+150 for p,cap in zip(row['positions'][:4],RECIPE[:4]))
                    or abs(row['positions'][4]-65535)>150
                    or not PREPARE[5]-150<=row['positions'][5]<=65535):
                raise RuntimeError('Fresh hand outside supported v4 withdrawal scope')
        if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
            raise RuntimeError('Hand not quiet before supported opening')
        return rows[-1]

    def withdraw(self,anchor):
        before=self.before_open()
        if (any(abs(a-b)>2 for a,b in zip(before['targets'],anchor['targets']))
                or any(abs(a-b)>150 for a,b in zip(before['positions'],anchor['positions']))):
            raise RuntimeError('Hand changed after supported opening permission')
        self.guard();self.io.grant(1135,[8000]*4,proof=before)
        self.report.update(phase='supported_four_flex_opening',supported_open_before=copy.deepcopy(before),
            supported_open_target=[8000]*4,supported_open_opposition_write=False,
            supported_open_is_not_loaded_motion_permission=True)
        self.save();self.io.write_group(1135,[8000]*4)
        minimum=before['positions'][:4];best=abs(before['positions'][5]-before['targets'][5])
        qualified=[];previous=None;began=time.monotonic()
        while time.monotonic()-began<30:
            row=self.snapshot();error=abs(row['positions'][5]-before['targets'][5])
            if (any(abs(a-8000)>2 for a in row['targets'][:4])
                    or row['targets'][4:]!=before['targets'][4:]
                    or any(not 7850<=p<=initial+150 for p,initial in zip(row['positions'][:4],before['positions'][:4]))
                    or any(p>low+150 for p,low in zip(row['positions'][:4],minimum))
                    or abs(row['positions'][4]-before['positions'][4])>150
                    or not max(0,min(before['positions'][5],before['targets'][5])-150)<=row['positions'][5]<=65535
                    or error>best+150):
                raise RuntimeError('Supported opening left four-flex withdrawal/retained-channel sweep')
            minimum=[min(a,b) for a,b in zip(minimum,row['positions'][:4])];best=min(best,error)
            reached=all(abs(p-8000)<=150 for p in row['positions'][:4])
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if reached and quiet:
                qualified.append(copy.deepcopy(row))
                if len(qualified)>=3 and qualified[-1]['finished_monotonic']-qualified[-3]['finished_monotonic']>=1:
                    self._critical_configuration()
                    self.report.update(supported_open_completed=True,supported_open_groups=qualified[-3:],
                        arm_motion_for_recovery=False,normal_PREPARE_qualified=False,
                        retained_target_writes=0,protection_reset_sent=False)
                    self.save();return row
            else:qualified=[]
            previous=row
        raise TimeoutError('Supported four-flex opening not qualified; no retry or normal cycle')

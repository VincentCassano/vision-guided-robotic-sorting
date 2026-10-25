"""One supported ring withdrawal from a freshly qualified HEALTHY completed target.

The old partial/fault position20283 is diagnostic only. The completed command
is20500. Fresh target/actual/quiet gates remain2/150; no status3 exception.
"""
import copy
import time
from visual_pick_sort_v9_hand import VisualHand,GripTransport
from visual_pick_sort_v9_source import RECIPE

OLD=[27500,31500,27000,20500,65535,64556]


class RingRetreatTransport(GripTransport):
    def grant(self,address,values,proof=None):
        if (address!=1138 or values!=[20000] or not isinstance(proof,dict)
                or not getattr(self._phase_authority,'supported_ring_retreat_permission',False)
                or self.approved is not None or self._attempted_writes
                or self.broken or self.terminal_latched or self._completed
                or any(abs(a-b)>2 for a,b in zip(proof['targets'],OLD))
                or min(proof['positions'][3],proof['targets'][3])<=20000
                or any(v in (3,4,5) for i,v in enumerate(proof['status']) if i!=3)
                or proof['status'][3]!=2):
            raise RuntimeError('Only one supported ring20500->20000 withdrawal allowed')
        self.approved=(address,tuple(values));self.motion_enabled=True


class RingRetreatHand(VisualHand):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.retreat_attempted=False

    def check_status(self,row):
        # v8 already observed cleared status3. Every current3/4 now blocks.
        return super().check_status(row)

    def before_retreat(self):
        if not self._configuration_checked:self.configuration()
        self._critical_configuration();rows=[self.snapshot() for unused in range(3)]
        if self.report['observed_healthy_v8_source']['old_ring_status3_cleared'] is not True:
            raise RuntimeError('Reviewed zero-write healthy observation missing')
        for row in rows:
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],OLD))
                    or any(abs(a-b)>150 for a,b in zip(row['positions'],OLD))
                    or min(row['positions'][3],row['targets'][3])<=20000
                    or row['status']!=[2]*6):
                self.report['healthy_start_qualification_failure']=dict(actual=copy.deepcopy(row),
                    expected_target=OLD[:],target_tolerance=2,actual_tolerance=150,
                    actual_target_deltas=[a-b for a,b in zip(row['positions'],OLD)],
                    old_fault_actual_used_as_anchor=False)
                self.save();raise RuntimeError('Current healthy completed hand target not qualified')
        if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
            raise RuntimeError('Supported hand is not quiet')
        self.report['healthy_start_qualification']=dict(groups=copy.deepcopy(rows),expected_target=OLD[:],
            target_tolerance=2,actual_tolerance=150,old_fault_actual_used_as_anchor=False,
            fresh_current_row_bound_after_permission=True)
        self.save()
        return rows[-1]

    def retreat_once(self,anchor):
        before=self.before_retreat()
        if (any(abs(a-b)>2 for a,b in zip(before['targets'],anchor['targets']))
                or any(abs(a-b)>150 for a,b in zip(before['positions'],anchor['positions']))):
            raise RuntimeError('Hand changed after supported ring permission')
        self.guard();self.io.grant(1138,[20000],proof=before);self.retreat_attempted=True
        self.report.update(phase='supported_ring_withdrawal',ring_withdrawal_before=copy.deepcopy(before),
            ring_withdrawal_target=20000,ring_withdrawal_register=1138,
            ring_withdrawal_not_lift_authority=True,uncommanded_channels=[0,1,2,4,5])
        self.save();self.io.write_group(1138,[20000])
        began=time.monotonic();minimum=before['positions'][3];previous=None;since=None;groups=[]
        while time.monotonic()-began<20:
            row=self.snapshot()
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],RECIPE))
                    or any(abs(row['positions'][i]-before['positions'][i])>150 for i in (0,1,2,4,5))
                    or not 19850<=row['positions'][3]<=before['positions'][3]+150
                    or row['positions'][3]>minimum+150):
                raise RuntimeError('Supported ring withdrawal left one-channel sweep')
            minimum=min(minimum,row['positions'][3])
            healthy=(all(s==2 for s in row['status'])
                and all(abs(a-b)<=150 for a,b in zip(row['positions'],RECIPE)))
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if healthy and quiet:
                if since is None:since=time.monotonic()
                groups.append(copy.deepcopy(row))
                if len(groups)>=3 and time.monotonic()-since>=1:
                    self._critical_configuration()
                    self.report.update(supported_ring_withdrawal_completed=True,
                        restored_healthy_grip_groups=groups[-3:],protection_clear_command_sent=False,
                        restored_status_source='new_complete_feedback_not_old_fault_or_reset',
                        actual_holding_force_or_pickup_verified=False)
                    self.save();return row
            else:since=None;groups=[]
            previous=row
        raise TimeoutError('Ring withdrawal did not restore healthy grip; no lift/reset/retry')

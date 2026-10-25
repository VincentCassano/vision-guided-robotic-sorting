"""One explicitly authorized empty-hand opposition retreat; no arm motion."""
import copy
import time
from visual_pick_sort_v7_hand import VisualHand,GripTransport
from visual_pick_sort_v7_source import OLD_PREPARE,PREPARE


class AlignmentTransport(GripTransport):
    def grant(self,address,values,proof=None):
        authority=self._phase_authority
        if (address!=1140 or values!=[PREPARE[5]] or self.approved is not None
                or self._attempted_writes or self.broken or self.terminal_latched or self._completed
                or not getattr(authority,'empty_opposition_permission',False)
                or not isinstance(proof,dict)
                or not (all(abs(a-b)<=2 for a,b in zip(proof['targets'],OLD_PREPARE))
                    or all(abs(a-b)<=2 for a,b in zip(proof['targets'],PREPARE)))
                or values[0]>=min(proof['positions'][5],proof['targets'][5])):
            raise RuntimeError('Only one lower-than-current-and-target empty opposition command permitted')
        self.approved=(address,tuple(values));self.motion_enabled=True


class AlignmentHand(VisualHand):
    def before_alignment(self):
        if not self._configuration_checked:self.configuration()
        self._critical_configuration();rows=[self.snapshot() for _ in range(3)]
        for row in rows:
            if (not (all(abs(a-b)<=2 for a,b in zip(row['targets'],OLD_PREPARE))
                        or all(abs(a-b)<=2 for a,b in zip(row['targets'],PREPARE)))
                    or any(abs(a-b)>150 for a,b in zip(row['positions'][:5],OLD_PREPARE[:5]))
                    or not PREPARE[5]-150<=row['positions'][5]<=65535):
                raise RuntimeError('Current hand not empty PREPARE flexion with reviewed old/new opposition target')
        if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
            raise RuntimeError('Current empty hand not quiet for opposition retreat')
        return rows[-1]

    def align_once(self,anchor):
        before=self.before_alignment()
        if (any(abs(a-b)>2 for a,b in zip(before['targets'],anchor['targets']))
                or any(abs(a-b)>150 for a,b in zip(before['positions'],anchor['positions']))):
            raise RuntimeError('Current hand changed after empty opposition permission')
        target=PREPARE[5]
        if (all(abs(a-b)<=2 for a,b in zip(before['targets'],PREPARE))
                and all(abs(a-b)<=150 for a,b in zip(before['positions'],PREPARE))):
            self.report.update(empty_opposition_withdrawal_completed=True,
                empty_opposition_write_skipped_already_qualified=True,
                empty_opposition_withdrawal_receipt=dict(before=before,target=target,
                    only_register1140_written=False,arm_motion_performed=False,
                    old_fault_state_used_as_new_reference=False))
            self.save();return before
        if target>=min(before['positions'][5],before['targets'][5]):
            raise RuntimeError('Empty opposition command would not be strictly withdrawing')
        self.guard();self.io.grant(1140,[target],proof=before)
        self.report.update(phase='withdrawing_empty_opposition',opposition_before=copy.deepcopy(before),
            opposition_withdrawal_target=target,opposition_only_write_attempted=True,
            opposition_parameter_or_protection_writes=0,automatic_retry=False)
        self.save();self.io.write_group(1140,[target])
        began=time.monotonic();previous=None;quiet_since=None;qualified=[];minimum=before['positions'][5]
        while time.monotonic()-began<30:
            row=self.snapshot()
            if (any(abs(a-b)>2 for a,b in zip(row['targets'][:5],before['targets'][:5]))
                    or abs(row['targets'][5]-target)>2
                    or any(abs(a-b)>150 for a,b in zip(row['positions'][:5],before['positions'][:5]))
                    or not target-150<=row['positions'][5]<=before['positions'][5]+150
                    or row['positions'][5]>minimum+150):
                raise RuntimeError('Empty opposition retreat left one-channel withdrawal sweep')
            minimum=min(minimum,row['positions'][5])
            reached=all(abs(a-b)<=150 for a,b in zip(row['positions'],PREPARE))
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if reached and quiet:
                if quiet_since is None:quiet_since=time.monotonic()
                qualified.append(copy.deepcopy(row))
                if len(qualified)>=3 and time.monotonic()-quiet_since>=1:
                    last=qualified[-3:]
                    if any(max(r['positions'][ch] for r in last)-min(r['positions'][ch] for r in last)>150 for ch in range(6)):
                        raise RuntimeError('New empty opposition pose not quiet inside original150 gate')
                    self._critical_configuration()
                    self.report.update(empty_opposition_withdrawal_completed=True,
                        empty_opposition_withdrawal_receipt=dict(before=before,target=target,groups=last,
                            uncommanded_channels=list(range(5)),arm_motion_performed=False,
                            only_register1140_written=True,old_fault_state_used_as_new_reference=False))
                    self.save();return row
            else:qualified=[];quiet_since=None
            previous=row
        raise TimeoutError('One empty opposition retreat did not reach new PREPARE; no retry or grasp')

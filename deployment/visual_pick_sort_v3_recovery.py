"""Recovery-only retained-channel sweep, never a new normal-grip tolerance."""
import copy
import time
from visual_pick_sort_v2_recovery import RecoveryHand as PreviousRecoveryHand,RecoveryTransport,read_empty_ready,OLD
from visual_pick_sort_v2_source import PREPARE


class RecoveryHand(PreviousRecoveryHand):
    def bind_retained(self,row):
        if self.opening or getattr(self,'retained_anchor',None) is not None:
            raise RuntimeError('Retained-channel observation can be bound only once before opening')
        if (len(row['targets'])!=6 or len(row['positions'])!=6
                or any(abs(a-b)>2 for a,b in zip(row['targets'],OLD))
                or any(type(p) is not int or not 0<=p<=65535 for p in row['positions'])
                or abs(row['positions'][4]-row['targets'][4])>150
                or row['positions'][5]<row['targets'][5]-150):
            raise RuntimeError('Current retained-channel values outside empty-only withdrawal scope')
        self.retained_anchor=copy.deepcopy(row)
        self.retained_best_error=[abs(row['positions'][ch]-row['targets'][ch]) for ch in (4,5)]
        self.report.setdefault('empty_withdrawal_retained_anchors',[]).append(dict(
            source='fresh_this_run_hand_read_not_old_fault_arm_state',snapshot=copy.deepcopy(row),
            channels=[4,5],target_writes_permitted=False,
            normal_prepare_position_tolerance_changed=False))
        self.save()

    def check_status(self,row):
        channels={i for i,v in enumerate(row['status']) if v==3}
        if 4 in row['status'] or not channels<={2,3}:
            self._protection('New/non-middle-ring protection blocks explicit withdrawal')
        if self.allowed_status3 is None:self.allowed_status3=channels
        if not channels<=self.allowed_status3:
            self._protection('New status3 during opening-only scope')
        self.allowed_status3.intersection_update(channels)
        if self.opening:
            if any(abs(a-8000)>2 for a in row['targets'][:4]):
                raise RuntimeError('Opening target changed')
        elif any(abs(a-b)>2 for a,b in zip(row['targets'],OLD)):
            raise RuntimeError('Current target is not the recorded failed v1 grip')
        anchor=getattr(self,'retained_anchor',None)
        if anchor is None:raise RuntimeError('Fresh retained-channel read must be bound before observing withdrawal')
        if row['targets'][4:]!=anchor['targets'][4:]:
            raise RuntimeError('Unwritten little/opposition target changed')
        for i,ch in enumerate((4,5)):
            target=anchor['targets'][ch];initial=anchor['positions'][ch];actual=row['positions'][ch]
            low=max(0,min(initial,target)-150);high=min(65535,max(initial,target)+150)
            error=abs(actual-target)
            if not low<=actual<=high or error>self.retained_best_error[i]+150:
                raise RuntimeError('Unwritten channel moved away from existing target or left current-to-target sweep')
            self.retained_best_error[i]=min(self.retained_best_error[i],error)
        self.report['last_empty_withdrawal_retained_observation']=dict(
            positions=row['positions'][4:],targets=row['targets'][4:],
            best_absolute_error=self.retained_best_error[:],no_retained_target_writes=True,
            normal_prepare_qualified_by_this_observation=False)

    def before_open(self):
        if not self._configuration_checked:self.configuration()
        self._critical_configuration();rows=[];began=time.monotonic()
        while time.monotonic()-began<15:
            row=self.snapshot()
            if any(not 8000<=p<=cap+150 for p,cap in zip(row['positions'][:4],OLD[:4])):
                raise RuntimeError('Withdrawal would add closure or exceeded old cap')
            rows.append(row);rows=rows[-3:]
            if (len(rows)==3 and all(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)<=150
                    for ch in range(6)) and rows[-1]['finished_monotonic']-rows[0]['finished_monotonic']>=1):
                return rows[-1]
        raise TimeoutError('Fresh empty-hand positions not quiet before withdrawal; no write')

    def withdraw(self,anchor):
        before=self.before_open()
        if (any(abs(a-b)>2 for a,b in zip(before['targets'],anchor['targets']))
                or any(abs(a-b)>150 for a,b in zip(before['positions'][:4],anchor['positions'][:4]))):
            raise RuntimeError('Current hand changed after supported-opening confirmation')
        self.guard();self.io.grant(1135,[8000]*4,proof=before)
        self.opening=True;self.anchor=copy.deepcopy(before)
        self.io.write_group(1135,[8000]*4)
        self.report['phase']='explicit_supported_withdrawal';self.save()
        previous=None;minimum=before['positions'][:4];qualified=[];began=time.monotonic()
        while time.monotonic()-began<30:
            row=self.snapshot()
            if (any(not 7850<=p<=initial+150 for p,initial in zip(row['positions'][:4],before['positions'][:4]))
                    or any(p>m+150 for p,m in zip(row['positions'][:4],minimum))):
                raise RuntimeError('Opening left withdrawal bounds')
            minimum=[min(p,m) for p,m in zip(row['positions'][:4],minimum)]
            reached=(all(abs(a-b)<=150 for a,b in zip(row['positions'],PREPARE))
                and not any(v in (3,4) for v in row['status']))
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if reached and quiet:
                qualified.append(copy.deepcopy(row))
                if len(qualified)>=3 and qualified[-1]['finished_monotonic']-qualified[0]['finished_monotonic']>=1:
                    self._critical_configuration()
                    self.report.update(supported_withdrawal_completed=True,
                        supported_withdrawal_groups=qualified[-3:],protection_reset_sent=False,
                        explicit_opening_writes=1,arm_moved_for_recovery=False)
                    self.save();return row
            else:qualified=[]
            previous=row
        raise TimeoutError('Supported withdrawal did not yield healthy PREPARE; no normal cycle')


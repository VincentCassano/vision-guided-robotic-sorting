"""Fresh explicit EMPTY-hand opening at the completed v6 bin endpoint only.

Not a loaded recovery or normal drop. One five-flex withdrawal; no opposition,
reset, init, speed, force, protection or configuration write.
"""
import copy
import time
from visual_pick_sort_v7_hand import VisualHand,GripTransport

OLD=[27500,31500,27000,19000,65535,64556]
DROP=[0,0,0,0,0,64556]


class EmptyOpenTransport(GripTransport):
    def grant(self,address,values,proof=None):
        if (address!=1135 or values!=[0]*5 or self.approved is not None
                or self._attempted_writes or self.broken or self.terminal_latched or self._completed
                or not getattr(self._phase_authority,'empty_five_flex_open_permission',False)
                or not isinstance(proof,dict)
                or any(abs(a-b)>2 for a,b in zip(proof['targets'],OLD))
                or any(abs(a-b)>150 for a,b in zip(proof['positions'],OLD))):
            raise RuntimeError('Only one explicit empty five-flex withdrawal from healthy v6 recipe')
        self.approved=(address,tuple(values));self.motion_enabled=True


class EmptyOpenHand(VisualHand):
    def before_open(self):
        if not self._configuration_checked:self.configuration()
        self._critical_configuration();rows=[self.snapshot() for unused in range(3)]
        selected=None
        for row in rows:
            options=[t for t in (OLD,DROP) if all(abs(a-b)<=2 for a,b in zip(row['targets'],t))
                and all(abs(a-b)<=150 for a,b in zip(row['positions'],t))]
            if len(options)!=1:raise RuntimeError('Empty recovery hand differs from healthy closed-v6/open target')
            if selected is not None and options[0]!=selected:raise RuntimeError('Empty recovery target changed')
            selected=options[0]
        if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
            raise RuntimeError('Empty recovery hand not quiet')
        return rows[-1]

    def open_once(self,anchor):
        before=self.before_open()
        if (any(abs(a-b)>2 for a,b in zip(before['targets'],anchor['targets']))
                or any(abs(a-b)>150 for a,b in zip(before['positions'],anchor['positions']))):
            raise RuntimeError('Hand changed after explicit empty opening permission')
        if all(abs(a-b)<=2 for a,b in zip(before['targets'],DROP)):
            self.report['empty_five_flex_open']=dict(already_open=True,write_skipped=True,feedback=before)
            self.save();return before
        self.guard();self.io.grant(1135,[0]*5,proof=before)
        self.report.update(phase='opening_empty_five_flex_at_v6_bin',
            empty_five_flex_open=dict(before=before,address=1135,count=5,targets=DROP,
                opposition_unwritten=True,loaded_recovery=False))
        self.save();self.io.write_group(1135,[0]*5)
        began=time.monotonic();minimum=before['positions'][:5];previous=None;quiet_since=None;rows=[]
        while time.monotonic()-began<30:
            row=self.snapshot()
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],DROP))
                    or abs(row['positions'][5]-before['positions'][5])>150
                    or any(not 0<=p<=start+150 for p,start in zip(row['positions'][:5],before['positions'][:5]))
                    or any(p>low+150 for p,low in zip(row['positions'][:5],minimum))):
                raise RuntimeError('Empty five-flex withdrawal changed target/opposition/sweep')
            minimum=[min(a,b) for a,b in zip(minimum,row['positions'][:5])]
            reached=all(abs(a-b)<=150 for a,b in zip(row['positions'],DROP))
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if reached and quiet:
                if quiet_since is None:quiet_since=time.monotonic()
                rows.append(copy.deepcopy(row))
                if len(rows)>=3 and time.monotonic()-quiet_since>=1:
                    self._critical_configuration()
                    self.report['empty_five_flex_open'].update(completed=True,groups=rows[-3:])
                    self.save();return row
            else:quiet_since=None;rows=[]
            previous=row
        raise TimeoutError('Empty opening not settled; no retry/return')

"""Explicit empty-hand withdrawal only, separate from the normal cycle.

No arm motion, fault reset or extra closure. Only r after an onsite assertion
that the hand is empty permits four targets to8000.
"""
import copy
import time
from visual_pick_sort_v2_hand import VisualHand,GripTransport
from visual_pick_sort_v2_source import PREPARE
from school_battery_transport import WRITE
OLD=[26500,30500,29500,22000,65535,65056]

class RecoveryTransport(GripTransport):
    def grant(self,address,values,proof=None):
        authority=self._phase_authority
        if (address!=1135 or values!=[8000]*4 or self.approved is not None
                or self._attempted_writes or self.broken or self.terminal_latched or self._completed
                or not getattr(authority,'supported_open_permission',False)
                or not isinstance(proof,dict) or any(p<8000 for p in proof['positions'][:4])
                or any(abs(a-b)>2 for a,b in zip(proof['targets'],OLD))):
            raise RuntimeError('Only one supported four-channel withdrawal is permitted')
        self.approved=(address,tuple(values));self.motion_enabled=True

class RecoveryHand(VisualHand):
    opening=False
    allowed_status3=None
    anchor=None
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
        if (any(abs(a-b)>2 for a,b in zip(row['targets'][4:],PREPARE[4:]))
                or any(abs(a-b)>150 for a,b in zip(row['positions'][4:],PREPARE[4:]))):
            raise RuntimeError('Untouched little/opposition differs')
    def before_open(self):
        if not self._configuration_checked:self.configuration()
        self._critical_configuration();rows=[self.snapshot() for _ in range(3)]
        for row in rows:
            if any(not 8000<=p<=cap+150 for p,cap in zip(row['positions'][:4],OLD[:4])):
                raise RuntimeError('Withdrawal would add closure or exceeded old cap')
        if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
            raise RuntimeError('Hand not quiet for supported withdrawal')
        return rows[-1]
    def withdraw(self,anchor):
        before=self.before_open()
        if (any(abs(a-b)>2 for a,b in zip(before['targets'],anchor['targets']))
                or any(abs(a-b)>150 for a,b in zip(before['positions'],anchor['positions']))):
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


def read_empty_ready(folder,report,save):
    """One read-only group; never qualifies a protected hand for lifting."""
    import json
    import socket
    from school_operational_readiness import OperationalReadTransport
    from supported_contact_transport import PhaseAuthority
    from school_device_identity import identity_fields,same_live_identity
    report['phase']='checking_explicit_withdrawal_readability';save()
    with (folder/'readiness_wire.jsonl').open('w',encoding='utf8') as stream:
        def log(row):stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush()
        io=OperationalReadTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),log,PhaseAuthority())
        try:
            first=io.read(1000,8);identity_fields(first)
            values={name:io.read(address,count) for name,address,count in (
                ('current_limit_ma',1095,6),('speeds',1125,6),('force_targets',1115,5),
                ('targets',1135,6),('positions',1145,6),('current_first_five',1105,5))}
            values['current']=values['current_first_five']+[io.single(1110)]
            values['status']=io.read(1085,6)
            report['explicit_withdrawal_work_reads']=values;save()
            if (values['current_limit_ma']!=[1299]*6 or values['speeds']!=[65535]*6 or any(values['force_targets'])
                    or any(v not in range(6) or v==4 for v in values['status'])
                    or any(v==3 and ch not in (2,3) for ch,v in enumerate(values['status']))
                    or any(c>limit for c,limit in zip(values['current'],values['current_limit_ma']))
                    or any(abs(a-b)>2 for a,b in zip(values['targets'],OLD))):
                raise RuntimeError('Current state outside explicit supported withdrawal scope')
            last=io.read(1000,8)
            if not same_live_identity(last,first):raise RuntimeError('Diagnostic changed during recovery reads')
            report['startup_readiness']=dict(passed=False,withdrawal_reads_only_passed=True,
                final_identity_raw=last,full_values=values,normal_lift_or_grip_authorized=False,
                no_actuator_writes=True,no_retry=True)
            save()
        finally:io.disarm_motion();io.sock.close()

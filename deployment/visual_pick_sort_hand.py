"""One proven recipe close; contact-only per-finger reduction; existing bin monitor."""
import copy
import time
from supported_contact_transport import SupportedContactTransport
from school_battery_transport import WRITE,MULTI
from tuned_grip_bin_hand import TunedHand,NormalDropTransport
from motion_event_hand_transport import MotionEventReadTransport
from school_device_identity import identity_fields,same_live_identity
from visual_pick_sort_source import PREPARE,RECIPE


class IdentityGuard:
    def exchange(self,message):
        value=super().exchange(message)
        if message.get('command')==MULTI and message.get('address')==1000:
            identity_fields(value)
            if not same_live_identity(value,self._phase_authority.accepted_identity_raw):
                self.latch_fault('New diagnostic/identity event');raise RuntimeError('Hand identity or diagnostic changed')
            self._phase_authority.accepted_identity_raw=value[:]
        return value


class MovingTransport(IdentityGuard,MotionEventReadTransport):
    pass


class DropTransport(IdentityGuard,NormalDropTransport):
    pass


class GripTransport(IdentityGuard,SupportedContactTransport):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.approved=None
    def grant(self,address,values,proof=None):
        authority=self._phase_authority;signature=(address,tuple(values))
        if (self.broken or self.terminal_latched or self._completed or self.approved is not None
                or signature in self._attempted_writes or not getattr(authority,'grip_permission',False)):
            raise RuntimeError('No fresh one-attempt grip permission')
        if signature==(1135,tuple(RECIPE)):
            if self._attempted_writes:raise RuntimeError('Recipe close already attempted')
        else:
            ch=address-1135
            if (not 0<=ch<4 or len(values)!=1 or not isinstance(proof,dict)
                    or ch in authority.reduced_channels or (1135,tuple(RECIPE)) not in authority._acknowledged_writes
                    or proof['status'][ch]!=5 or proof['fresh_status'][ch]!=5
                    or any(v not in (0,1,2,5) for v in proof['fresh_status'])
                    or values[0]!=min(proof['positions'][ch],proof['targets'][ch])
                    or not 8000<=values[0]<proof['targets'][ch]):
                raise RuntimeError('Only one fresh status5 withdrawal per flex channel')
            authority.reduced_channels.add(ch)
        self.approved=signature;self.motion_enabled=True
    def _validate(self,message):
        if message.get('command')!=WRITE:return super()._validate(message)
        if (set(message)!={'command','port','device','address','num','data'}
                or any(type(message.get(k)) is not int for k in ('port','device','address','num'))
                or message['port']!=1 or message['device']!=2
                or not isinstance(message['data'],list) or len(message['data'])!=2*message['num']
                or any(type(v) is not int or not 0<=v<=255 for v in message['data'])):
            raise ValueError('Invalid exact grip request')
        signature=(message['address'],tuple(message['data'][i]*256+message['data'][i+1]
            for i in range(0,len(message['data']),2)))
        if (signature!=self.approved or not self.motion_enabled or self.broken
                or self.terminal_latched or self._completed or signature in self._attempted_writes):
            raise RuntimeError('Unapproved or repeated grip write')
        self.approved=None;return signature
    def enable_motion(self):raise RuntimeError('Use exact recipe/contact permission only')
    def set_phase(self,*args,**kwargs):raise RuntimeError('No force/reset/configuration capability')


class VisualHand(TunedHand):
    def preflight_prepare(self):
        if not self._configuration_checked:self.configuration()
        self._critical_configuration();rows=[self.snapshot() for _ in range(3)]
        for row in rows:
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],PREPARE))
                    or any(abs(a-b)>150 for a,b in zip(row['positions'],PREPARE))):
                raise RuntimeError('Current PREPARE changed; no close or automatic preparation')
        if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
            raise RuntimeError('PREPARE not quiet')
        return rows[-1]
    def close_recipe(self):
        before=self.preflight_prepare()
        for ch in range(6):
            if before['status'][ch]==5 and RECIPE[ch]>min(before['targets'][ch],before['positions'][ch]+150):
                raise RuntimeError('No added closure to an already stalled channel')
        active=RECIPE[:]
        self.report.update(phase='grasping',active_grip_targets=active[:],maximum_contact_reductions=4,
            contact_reduction_is_not_holding_force_or_stop_proof=True);self.save()
        self.guard();self.io.grant(1135,RECIPE);self.io.write_group(1135,RECIPE)
        started=time.monotonic();previous=None;stable=0;quiet_since=None
        while time.monotonic()-started<30:
            row=self.snapshot()
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],active))
                    or any(p>bound+150 or p<min(start,8000)-150 for p,bound,start in
                        zip(row['positions'],RECIPE,before['positions']))
                    or any(abs(a-b)>150 for a,b in zip(row['positions'][4:],before['positions'][4:]))):
                raise RuntimeError('Grasp target/sweep/orientation changed')
            reduced=False
            for ch in range(4):
                if row['status'][ch]==5 and ch not in self.sequence.authority.reduced_channels and row['positions'][ch]<active[ch]-2:
                    fresh=self.read('status_before_contact_reduction',1085,6)
                    if any(v not in (0,1,2,5) for v in fresh):raise RuntimeError('Protection before contact reduction')
                    proof=dict(copy.deepcopy(row),fresh_status=fresh[:]);value=min(active[ch],row['positions'][ch])
                    self.guard();self.io.grant(1135+ch,[value],proof=proof);self.io.write_group(1135+ch,[value])
                    active[ch]=value;self.report.setdefault('contact_reductions',[]).append(
                        dict(channel=ch,value=value,proof=proof,withdrawal_only=True))
                    self.report['active_grip_targets']=active[:];self.save();reduced=True;break
            if reduced:previous=None;stable=0;quiet_since=None;continue
            reached=all(abs(a-b)<=150 for a,b in zip(row['positions'],active))
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if reached and quiet:
                if quiet_since is None:quiet_since=time.monotonic()
                stable+=1
                if stable>=3 and time.monotonic()-quiet_since>=1:
                    self._critical_configuration()
                    self.report['grasp_feedback']=copy.deepcopy(row);self.save();return active,row
            else:stable=0;quiet_since=None
            previous=row
        raise TimeoutError('Grip feedback did not settle; no retry/lift or automatic opening')

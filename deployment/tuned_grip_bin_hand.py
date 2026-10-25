"""Read-only tuned-grip monitor and one explicitly authorized normal bin drop."""
import copy
import math
from battery_grasp.real_grasp import orientation_angle_rad
from school_operational_pose_hand import CycleHand
from school_operational_hand import TrialHand
from school_device_identity import identity_fields, same_live_identity
from supported_contact_transport import SupportedContactTransport, WritePhase
from school_battery_transport import WRITE

RECIPE=[25500,30000,28000,21250,65535,65222]
DROP=[0,0,0,0,0,65222]


def normal_drop_values(authority):
    values=getattr(authority,'normal_drop_targets',DROP)
    held=getattr(authority,'held_opposition_target',65222)
    if (not isinstance(values,list) or len(values)!=6 or values[:5]!=[0]*5
            or any(type(v) is not int or not 0<=v<=65535 for v in values)
            or type(held) is not int or values[5]!=held):
        raise RuntimeError('Normal opening must preserve the freshly qualified opposition target')
    return values[:]


class HeldReadTransport(SupportedContactTransport):
    def _validate(self,message):
        if not isinstance(message,dict) or message.get('command')==WRITE:
            raise RuntimeError('Held/empty arm-motion monitor permits no hand write')
        return super()._validate(message)

    def set_phase(self,*args,**kwargs):raise RuntimeError('Read-only held monitor has no write phase')
    def enable_motion(self):raise RuntimeError('Read-only held monitor has no write enable')


class NormalDropTransport(HeldReadTransport):
    def set_phase(self,phase,authority):
        permit=getattr(authority,'bin_drop_permission',None)
        if (phase!='normal_tuned_bin_drop' or authority is not self._phase_authority
                or self.motion_enabled or self.broken or self.terminal_latched or self._completed
                or authority._attempted_writes or self._phase is not WritePhase.READONLY
                or not getattr(authority,'tuned_grip_freshly_qualified',False)
                or not getattr(authority,'selected_bin_endpoint_qualified',False)
                or not isinstance(permit,dict) or permit.get('index')!=11
                or permit.get('selected_bin_normal_drop_authorized') is not True
                or permit.get('shell_actually_lifted_and_retained') is not True):
            self.latch_fault('Normal tuned drop lacks fresh grip, observed lift or bin endpoint')
            raise RuntimeError('Normal drop permission missing')
        self._phase=WritePhase.RELEASE;self._phase_index=0
        self.log(dict(event='normal_tuned_drop_phase',permission=permit,
            source_success_sha256=authority.tuned_source_sha256,
            historical_close_ACK_fabricated=False,read_pacing_qualification_fabricated=False))

    def enable_motion(self):
        if self._phase is not WritePhase.RELEASE or self.broken or self.terminal_latched or self._completed:
            raise RuntimeError('No healthy normal-drop phase')
        self.motion_enabled=True

    def _validate(self,message):
        if message.get('command')!=WRITE:return super()._validate(message)
        values=normal_drop_values(self._phase_authority)
        expected=dict(command=WRITE,port=1,device=2,address=1135,num=6,
            data=[b for v in values for b in (v>>8,v&255)])
        signature=(1135,tuple(values))
        if (message!=expected or any(type(message.get(k)) is not int for k in ('port','device','address','num'))
                or any(type(v) is not int for v in message.get('data',[]))
                or not self.motion_enabled or self.broken or self.terminal_latched or self._completed
                or self._phase is not WritePhase.RELEASE or self._phase_index!=0
                or signature in self._attempted_writes):
            raise RuntimeError('Only one exact normal selected-bin opening is allowed')
        return signature


class TunedHand(CycleHand):
    def read_feedback_group(self):return TrialHand.read_feedback_group(self)

    def _identity_checked(self,raw):
        identity_fields(raw);authority=self.sequence.authority
        if not same_live_identity(raw,authority.accepted_identity_raw):
            raise RuntimeError('Hand identity/new diagnostic changed')
        authority.accepted_identity_raw=raw[:]

    def guard(self):
        try:
            q,p=self.sequence.arm_state()
            if self.sequence.bin_motion is None and (
                    any(abs(a-b)>.08 for a,b in zip(q,self.sequence.guard_target))
                    or math.dist(p[:3],self.sequence.guard_pose[:3])>.0005
                    or orientation_angle_rad(p[3:],self.sequence.guard_pose[3:])>.005):
                raise RuntimeError('Stationary arm changed during tuned hand operation')
        except BaseException:self._disable();raise

    def checked_row(self,row,targets,anchor):
        recipe=getattr(self.sequence,'tuned_recipe',RECIPE)
        drop=getattr(self.sequence,'tuned_drop_targets',DROP)
        if list(targets) not in (recipe,drop):raise RuntimeError('Unreviewed tuned/drop recipe')
        if (any(abs(a-b)>2 for a,b in zip(row['targets'],targets))
                or any(abs(a-b)>150 for a,b in zip(row['positions'],targets))
                or any(abs(a-b)>2 for a,b in zip(anchor['targets'],targets))
                or any(abs(a-b)>150 for a,b in zip(row['positions'],anchor['positions']))):
            raise RuntimeError('Tuned hand target/actual/anchor changed')
        return row

    def preflight_loaded(self,targets,anchor=None):
        if self.io.motion_enabled:raise RuntimeError('Loaded observer cannot enable hand writes')
        if not self._configuration_checked:self.configuration()
        critical=self._critical_configuration();row=self.snapshot()
        self.checked_row(row,targets,anchor)
        self.report['last_loaded_preflight']=dict(copy.deepcopy(row),**critical);self.save()
        return row

    def monitor_loaded(self,targets,anchor):
        if self.io.motion_enabled:raise RuntimeError('Arm-motion hand monitor is read only')
        row=self.checked_row(self.snapshot(),targets,anchor)
        name='last_empty_bin_return_monitor' if targets==getattr(self.sequence,'tuned_drop_targets',DROP) else 'last_loaded_monitor'
        self.report[name]=copy.deepcopy(row);self.save();return row

    def preflight_empty(self,targets):
        if targets!=getattr(self.sequence,'tuned_drop_targets',DROP):raise RuntimeError('Empty return requires normal-drop target')
        return self.preflight_loaded(targets,self.sequence.load_anchor)

    def open_at_bin(self,targets,anchor):
        drop=normal_drop_values(self.sequence.authority)
        self._critical_configuration();before=self.checked_row(self.snapshot(),targets,anchor)
        self.guard();self.io.set_phase('normal_tuned_bin_drop',self.sequence.authority)
        self.io.enable_motion();self.sequence.hand_motion_enabled=True
        self.report.update(bin_opening_before=copy.deepcopy(before),bin_opening_targets=drop[:]);self.save()
        self.io.write_group(1135,drop)
        began=self.clock();previous=None;quiet_since=None;groups=0;minima=before['positions'][:5]
        while self.clock()-began<30:
            row=self.snapshot()
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],drop))
                    or abs(row['positions'][5]-before['positions'][5])>150
                    or any(not 0<=p<=initial+150 for p,initial in zip(row['positions'][:5],before['positions'][:5]))
                    or any(p>low+150 for p,low in zip(row['positions'][:5],minima))):
                raise RuntimeError('Normal opening target/withdrawal sweep changed')
            minima=[min(a,b) for a,b in zip(minima,row['positions'][:5])]
            reached=all(abs(a-b)<=150 for a,b in zip(row['positions'],drop))
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if reached and quiet:
                if quiet_since is None:quiet_since=self.clock()
                groups+=1
                if groups>=3 and self.clock()-quiet_since>=1:
                    self._critical_configuration()
                    self.report.update(bin_opening_feedback=copy.deepcopy(row),bin_opening_feedback_completed=True)
                    self.save();return row
            else:quiet_since=None;groups=0
            previous=row
        self._disable();raise TimeoutError('Normal bin opening not confirmed; no retry/return')

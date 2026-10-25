"""Close only four flex channels; little/opposition remain read-only during grip."""
import copy
import time
from supported_contact_transport import SupportedContactTransport,WritePhase
from school_battery_transport import WRITE,MULTI,SINGLE
from tuned_grip_bin_hand import TunedHand,NormalDropTransport,normal_drop_values
from motion_event_hand_transport import MotionEventReadTransport
from school_device_identity import identity_fields,same_live_identity
from visual_pick_sort_v8_source import PREPARE,RECIPE


class IdentityGuard:
    def exchange(self,message):
        value=super().exchange(message)
        if message.get('command')==MULTI and message.get('address')==1000:
            identity_fields(value)
            if not same_live_identity(value,self._phase_authority.accepted_identity_raw):
                self.latch_fault('New diagnostic/identity event');raise RuntimeError('Hand identity or diagnostic changed')
            self._phase_authority.accepted_identity_raw=value[:]
        return value


class ReadPacing:
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self._pacing_probe=False;self._socket_identity_seen=False
    def pre_send_quiet_seconds(self,message):
        if (message.get('command') in (MULTI,SINGLE) and self.previous is not None
                and self.previous[0] in (MULTI,SINGLE) and self._socket_identity_seen
                and not self.broken and not self.terminal_latched
                and (self._pacing_probe or getattr(self._phase_authority,'read_pacing_qualified',False))):
            return .100
        return .300
    def exchange(self,message):
        value=super().exchange(message)
        if message.get('command')==MULTI and message.get('address')==1000:self._socket_identity_seen=True
        return value


class MovingTransport(ReadPacing,IdentityGuard,MotionEventReadTransport):
    def arm_event(self,row):
        if not isinstance(row,dict) or row.get('state')!='current_trajectory_state':return False
        context=self.event_context()
        if not isinstance(context,dict) or context.get('kind')!='connected_school_transfer':
            return super().arm_event(row)
        from school_battery_transport import TransportError
        index=len(context['events'])
        if (set(row)!={'state','trajectory_state','device','trajectory_connect'}
                or type(row.get('device')) is not int or row['device']!=0
                or type(row.get('trajectory_state')) is not bool or row['trajectory_state'] is not True
                or type(row.get('trajectory_connect')) is not int
                or context.get('acknowledged') is not True or index>=3
                or row['trajectory_connect']!=[1,1,0][index]):
            raise TransportError('Connected route notification failure/order/shape/duplicate; no continuation')
        context['events'].append(copy.deepcopy(row))
        context['event_seen']=len(context['events'])==3
        self.log(dict(event='connected_arm_motion_notification',sequence=self.sequence,
            monotonic=time.monotonic(),notification=row,waypoint_index=index,
            used_as_hand_response=False,intermediate_endpoint_qualified=False))
        return True


class DropTransport(ReadPacing,IdentityGuard,NormalDropTransport):
    def set_phase(self,phase,authority):
        permit=getattr(authority,'continuous_cycle_permission',None)
        if (phase!='normal_tuned_bin_drop' or authority is not self._phase_authority
                or self.motion_enabled or self.broken or self.terminal_latched or self._completed
                or authority._attempted_writes or self._phase is not WritePhase.READONLY
                or not getattr(authority,'tuned_grip_freshly_qualified',False)
                or not getattr(authority,'selected_bin_endpoint_qualified',False)
                or not getattr(authority,'continuous_loaded_route_completed',False)
                or not isinstance(permit,dict) or permit.get('index') not in (5,14)
                or (permit.get('index')==14 and permit.get('supported_ring_retreat_and_continuous_cycle_authorized') is not True)
                or permit.get('continuous_lift_transfer_normal_drop_authorized') is not True
                or permit.get('planned_loaded_route_min20mm_clear_confirmed') is not True):
            self.latch_fault('Continuous normal drop lacks upfront route permission or healthy endpoint')
            raise RuntimeError('Continuous normal drop permission missing')
        self._phase=WritePhase.RELEASE;self._phase_index=0
        self.log(dict(event='continuous_normal_drop_phase',permission=permit,
            source_success_sha256=authority.tuned_source_sha256,
            actual_retention_sensor_verified=False,future_operator_observation_fabricated=False))

    def _validate(self,message):
        if message.get('command')!=WRITE:return super()._validate(message)
        from tuned_grip_bin_hand import normal_drop_values
        values=normal_drop_values(self._phase_authority)
        expected=dict(command=WRITE,port=1,device=2,address=1135,num=5,
            data=[b for v in values[:5] for b in (v>>8,v&255)])
        signature=(1135,tuple(values[:5]))
        if (message!=expected or any(type(message.get(k)) is not int for k in ('port','device','address','num'))
                or any(type(v) is not int for v in message.get('data',[]))
                or not self.motion_enabled or self.broken or self.terminal_latched or self._completed
                or self._phase is not WritePhase.RELEASE or self._phase_index!=0
                or signature in self._attempted_writes):
            raise RuntimeError('Only one exact five-flex normal drop; opposition write forbidden')
        return signature


class GripTransport(ReadPacing,IdentityGuard,SupportedContactTransport):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.approved=None
    def grant(self,address,values,proof=None):
        authority=self._phase_authority;signature=(address,tuple(values))
        if (self.broken or self.terminal_latched or self._completed or self.approved is not None
                or signature in self._attempted_writes or not getattr(authority,'grip_permission',False)):
            raise RuntimeError('No fresh one-attempt grip permission')
        if signature==(1135,tuple(RECIPE[:4])):
            if self._attempted_writes:raise RuntimeError('Recipe close already attempted')
        else:
            ch=address-1135
            if (not 0<=ch<4 or len(values)!=1 or not isinstance(proof,dict)
                    or ch in authority.reduced_channels or (1135,tuple(RECIPE[:4])) not in authority._acknowledged_writes
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
    def qualify_read_pacing(self):
        authority=self.sequence.authority
        if getattr(authority,'read_pacing_qualified',False):return
        if self.io.motion_enabled or self.io._attempted_writes or self.io.terminal_latched:
            raise RuntimeError('Readonly pacing probe must precede all normal-cycle hand writes')
        before=self.preflight_prepare();self.io._pacing_probe=True;rows=[]
        try:
            for unused in range(3):
                row=self.snapshot()
                if (any(abs(a-b)>2 for a,b in zip(row['targets'],PREPARE))
                        or any(abs(a-b)>150 for a,b in zip(row['positions'],PREPARE))):
                    raise RuntimeError('Readonly pacing probe changed PREPARE')
                rows.append(copy.deepcopy(row))
        finally:self.io._pacing_probe=False
        after=self.snapshot();evidence=[before]+rows+[after]
        if (any(any(abs(a-b)>2 for a,b in zip(r['targets'],PREPARE))
                or any(abs(a-b)>150 for a,b in zip(r['positions'],PREPARE)) for r in evidence)
                or any(max(r['positions'][ch] for r in evidence)-min(r['positions'][ch] for r in evidence)>150 for ch in range(6))):
            raise RuntimeError('Readonly pacing qualification failed; no slower retry or close')
        self.guard();authority.read_pacing_qualified=True
        self.report['known_read_pacing_qualification']=dict(passed=True,before=before,groups=rows,after=after,
            read_gap_seconds=.100,write_and_first_postwrite_gap_seconds=.300,
            manufacturer_minimum_claimed=False,no_actuator_writes_in_probe=True)
        self.save()

    def close_recipe(self):
        before=self.preflight_prepare()
        for ch in range(6):
            if before['status'][ch]==5 and RECIPE[ch]>min(before['targets'][ch],before['positions'][ch]+150):
                raise RuntimeError('No added closure to an already stalled channel')
        active=RECIPE[:]
        self.report.update(phase='grasping',active_grip_targets=active[:],maximum_contact_reductions=4,
            contact_reduction_is_not_holding_force_or_stop_proof=True);self.save()
        self.report['grip_command_scope']=dict(address=1135,count=4,values=RECIPE[:4],
            uncommanded_channels=[4,5],retained_targets=before['targets'][4:],retained_actual=before['positions'][4:])
        self.save();self.guard();self.io.grant(1135,RECIPE[:4]);self.io.write_group(1135,RECIPE[:4])
        started=time.monotonic();previous=None;stable=0;quiet_since=None;qualified=[]
        while time.monotonic()-started<30:
            row=self.snapshot()
            if (any(abs(a-b)>2 for a,b in zip(row['targets'],active))
                    or any(p>bound+150 or p<min(start,8000)-150 for p,bound,start in
                        zip(row['positions'],RECIPE,before['positions']))
                    or any(abs(a-b)>150 for a,b in zip(row['positions'][4:],before['positions'][4:]))):
                self.report['grasp_guard_failure']=dict(before=copy.deepcopy(before),actual=copy.deepcopy(row),
                    target_errors=[a-b for a,b in zip(row['targets'],active)],
                    retained_actual_deltas=[a-b for a,b in zip(row['positions'][4:],before['positions'][4:])],
                    target_tolerance=2,position_tolerance=150,uncommanded_channels=[4,5])
                self.save();raise RuntimeError('Grasp target/sweep/orientation changed')
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
            if reduced:previous=None;stable=0;quiet_since=None;qualified=[];continue
            reached=all(abs(a-b)<=150 for a,b in zip(row['positions'],active))
            quiet=previous is not None and all(abs(a-b)<=150 for a,b in zip(row['positions'],previous['positions']))
            if reached and quiet:
                if quiet_since is None:quiet_since=time.monotonic()
                stable+=1;qualified.append(copy.deepcopy(row))
                if stable>=3 and time.monotonic()-quiet_since>=1:
                    self._critical_configuration()
                    self.report['grasp_feedback']=copy.deepcopy(row)
                    self.report['grasp_qualified_groups']=qualified[-3:];self.save();return active,row
            else:stable=0;quiet_since=None;qualified=[]
            previous=row
        raise TimeoutError('Grip feedback did not settle; no retry/lift or automatic opening')

    def open_at_bin(self,targets,anchor):
        drop=normal_drop_values(self.sequence.authority)
        self._critical_configuration();before=self.checked_row(self.snapshot(),targets,anchor)
        self.guard();self.io.set_phase('normal_tuned_bin_drop',self.sequence.authority)
        self.io.enable_motion();self.sequence.hand_motion_enabled=True
        self.report.update(bin_opening_before=copy.deepcopy(before),bin_opening_targets=drop[:]);self.save()
        self.report['normal_drop_command_scope']=dict(address=1135,count=5,opposition_unwritten=True)
        self.save();self.io.write_group(1135,drop[:5])
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

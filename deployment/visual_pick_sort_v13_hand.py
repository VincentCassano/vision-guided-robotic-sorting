"""Exact new-grasp hand phases; target feedback, not actual arrival, gates flow.

No SDK import, arm command, reset or configuration write exists here. Ordinary
status5 and stopped-short actual positions remain observable without changing
the requested grip. Genuine protection, current, identity or protocol failures
are returned to the arm worker through an exception and retained raw evidence.
"""
import copy
import math
import time

from manual_hand_bus_v2 import _ManualTransport, CONFIG_BLOCKS
from school_battery_transport import MULTI, WRITE, TransportError
from school_device_identity import identity_fields, same_live_identity, configuration_matches
from battery_grasp.real_grasp import orientation_angle_rad


PREPARE=[35000,7500,9480,30000,65535,65535]
GRIP=[40000,12500,14480,30000,65535,65535]
DROP=[0,0,0,0,0,65535]
PHASE_STEPS={
    'prepare':[(1140,(65535,)),(1139,(65535,)),(1138,(30000,)),
               (1135,(35000,)),(1136,(7500,)),(1137,(9480,))],
    'hold':[(1139,(65535,)),(1138,(30000,))],
    'grasp':[(1135,(40000,)),(1136,(12500,)),(1137,(14480,))],
    'drop':[(1135,(0,0,0,0,0))],
}
PHASE_ORDER=['prepare','hold','grasp','drop']


class HandFault(RuntimeError):
    pass


def _u16(value,n,label):
    if (not isinstance(value,(list,tuple)) or len(value)!=n
            or any(type(v) is not int or not 0<=v<=65535 for v in value)):
        raise HandFault(label+': complete unsigned register vector required')
    return list(value)


class CycleTransport(_ManualTransport):
    """Strict CRLF hand transport plus one run-scoped ordered write capability."""
    def __init__(self,sock,log,authority,motion_context_callback):
        super().__init__(sock,log)
        self._phase_authority=authority
        self._attempted_writes=authority._attempted_writes
        self.identity_raw=copy.deepcopy(getattr(authority,'accepted_identity_raw',None))
        self.event_context=motion_context_callback
        if not hasattr(authority,'v13_hand_progress'):
            authority.v13_hand_progress=dict(completed=[],active=None,index=0,steps=[])

    def _arm_telemetry(self,row):
        if isinstance(row,dict) and row.get('command') in ('start_drag_teach','stop_drag_teach'):
            return super()._arm_telemetry(row)
        if not isinstance(row,dict) or row.get('state')!='current_trajectory_state':return False
        context=self.event_context()
        if (set(row)!={'state','trajectory_state','device','trajectory_connect'}
                or type(row.get('device')) is not int or row['device']!=0
                or type(row.get('trajectory_state')) is not bool
                or row['trajectory_state'] is not True
                or type(row.get('trajectory_connect')) is not int
                or not isinstance(context,dict) or context.get('acknowledged') is not True):
            raise TransportError('Invalid or unscoped current arm trajectory notification')
        if context.get('kind')=='connected_school_transfer':
            events=context.get('events')
            if (not isinstance(events,list) or len(events)>=3
                    or row['trajectory_connect']!=[1,1,0][len(events)]):
                raise TransportError('Connected route notification order differs from1/1/0')
            index=len(events);events.append(copy.deepcopy(row))
            context['event_seen']=len(events)==3
        else:
            if (context.get('method') not in ('Movel_Cmd','Movej_P_Cmd')
                    or row['trajectory_connect']!=0 or context.get('event_seen') is not False):
                raise TransportError('Unexpected or duplicate single-segment arm notification')
            index=0;context['event_seen']=True
        self.log(dict(event='v13_arm_motion_notification',sequence=self.sequence,
            monotonic=time.monotonic(),notification=copy.deepcopy(row),waypoint_index=index,
            connected=context.get('kind')=='connected_school_transfer',used_as_hand_response=False,
            endpoint_or_object_retention_qualified=False))
        return True

    def begin_phase(self,name):
        state=self._phase_authority.v13_hand_progress
        if (self.broken or self.terminal_latched or self._completed
                or state['active'] is not None or len(state['completed'])>=len(PHASE_ORDER)
                or name!=PHASE_ORDER[len(state['completed'])]):
            raise HandFault('Hand phase must follow prepare/hold/grasp/drop once, without retry')
        state.update(active=name,index=0,steps=[])
        self.log(dict(event='v13_hand_phase_started',name=name,
            exact_order=copy.deepcopy(PHASE_STEPS[name]),actual_arrival_required=False))

    def permit_step(self,address,values,already_matching=False):
        state=self._phase_authority.v13_hand_progress
        name=state['active'];signature=(address,tuple(values))
        if (name not in PHASE_STEPS or state['index']>=len(PHASE_STEPS[name])
                or signature!=PHASE_STEPS[name][state['index']]
                or self.broken or self.terminal_latched or self._completed or self.permit is not None):
            raise HandFault('Target differs from the next exact new-grasp phase step')
        if already_matching:
            state['steps'].append(dict(address=address,values=list(values),sent=False,
                qualification='current_target_already_matches_within2'))
            state['index']+=1
            return False
        if signature in self._attempted_writes:
            raise HandFault('This exact target was already attempted; no automatic retry')
        self.permit=signature
        self.permit_serial=name+':'+str(state['index'])
        self.last_write_tx_started=False;self.last_write_acknowledged=False
        self.motion_enabled=True
        return True

    def _validate(self,message):
        if not isinstance(message,dict) or message.get('command')!=WRITE:
            return super()._validate(message)
        if self.permit is None:raise HandFault('No exact new-grasp target permit')
        address,values=self.permit
        expected=dict(command=WRITE,port=1,device=2,address=address,num=len(values),
            data=[byte for value in values for byte in (value>>8,value&255)])
        if (message!=expected or any(type(message.get(k)) is not int for k in ('port','device','address','num'))
                or any(type(v) is not int for v in message.get('data',[]))
                or not self.motion_enabled or self.broken or self.terminal_latched or self._completed):
            raise HandFault('Only the exact single-position or five-flex release step is writable')
        self.permit=None
        return address,values

    def exchange(self,message):
        value=super().exchange(message)
        if message.get('command')==MULTI and message.get('address')==1000:
            self._phase_authority.accepted_identity_raw=value[:]
        if message.get('command')==WRITE:
            state=self._phase_authority.v13_hand_progress
            state['steps'].append(dict(address=message['address'],
                values=[message['data'][i]*256+message['data'][i+1]
                    for i in range(0,len(message['data']),2)],sent=True,acknowledged=True))
            state['index']+=1
        return value

    def finish_phase(self):
        state=self._phase_authority.v13_hand_progress;name=state['active']
        if (name not in PHASE_STEPS or state['index']!=len(PHASE_STEPS[name])
                or self.broken or self.terminal_latched or self.motion_enabled or self.permit is not None):
            raise HandFault('Incomplete hand phase cannot advance')
        self.log(dict(event='v13_hand_phase_completed',name=name,steps=copy.deepcopy(state['steps']),
            actual_arrival_required=False,physical_holding_verified=False))
        state['completed'].append(name);state.update(active=None,index=0,steps=[])


class CycleHand:
    def __init__(self,seq,io,report,save,samples):
        self.sequence,self.io,self.report,self.save,self.samples=seq,io,report,save,samples
        self.clock=getattr(seq,'clock',time.monotonic)
        self._configuration_checked=bool(getattr(seq.authority,'v13_hand_configuration',None))
        if self._configuration_checked:
            self.report['configuration']=copy.deepcopy(seq.authority.v13_hand_configuration)
        self.report.setdefault('feedback_policy',{}).update(
            target_register_tolerance=2,actual_finger_arrival_required=False,
            actual_finger_settling_required=False,status5_blocks_planned_closure=False,
            automatic_target_reduction=False,normal_stall_is_not_grasp_failure=True,
            raw_feedback_retained=True,positions_are_not_force_measurements=True)

    def event(self,event_type,**fields):
        row=dict(event=event_type,monotonic=self.clock(),phase=self.report.get('phase'),**fields)
        if self.samples is not None:
            import json
            self.samples.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');self.samples.flush()
        return row

    def _fault(self,message,row=None):
        self.report.setdefault('hand_faults',[]).append(dict(reason=message,
            snapshot=copy.deepcopy(row),phase=self.report.get('phase'),monotonic=self.clock()))
        self.save();self.io.latch_fault(message)
        raise HandFault(message)

    def guard(self):
        q,p=self.sequence.arm_state()
        if self.sequence.bin_motion is None and (
                any(abs(a-b)>.08 for a,b in zip(q,self.sequence.guard_target))
                or math.dist(p[:3],self.sequence.guard_pose[:3])>.0005
                or orientation_angle_rad(p[3:],self.sequence.guard_pose[3:])>.005):
            self._fault('Stationary arm changed during explicit hand phase')

    def read(self,name,address,count):
        began=self.clock();values=self.io.read(address,count)
        self.event('v13_hand_read',name=name,address=address,values=values,started_monotonic=began)
        return values

    def configuration(self):
        cached=getattr(self.sequence.authority,'v13_hand_configuration',None)
        if cached is None:
            actual={name:self.read(name,address,count) for name,address,count in CONFIG_BLOCKS}
            self.report['configuration']=copy.deepcopy(actual);self.save()
            if not configuration_matches(actual,self.sequence.profile['observed_configuration']):
                self._fault('Current complete calibration/PID/protection profile differs')
            self.sequence.authority.v13_hand_configuration=copy.deepcopy(actual)
        else:self.report['configuration']=copy.deepcopy(cached)
        self._configuration_checked=True
        self._critical_configuration()
        self.save();return copy.deepcopy(self.report['configuration'])

    def _critical_configuration(self):
        if not self._configuration_checked:self.configuration();return self.report['last_hand_critical']
        identity=self.read('critical_identity',1000,8);identity_fields(identity)
        limits=_u16(self.read('current_limit_ma',1095,6),6,'current limits')
        speeds=_u16(self.read('speeds',1125,6),6,'speed configuration')
        force=_u16(self.read('force_targets',1115,5),5,'force targets')
        row=dict(identity=identity,current_limit_ma=limits,speeds=speeds,force_targets=force)
        self.report['last_hand_critical']=copy.deepcopy(row);self.save()
        if (limits!=self.sequence.profile['current_limit_ma'] or speeds!=[65535]*6 or any(force)):
            self._fault('Current hand limits/speeds/force configuration changed',row)
        return row

    def snapshot(self):
        if not self._configuration_checked:self.configuration()
        began=self.clock();identity=self.read('identity_feedback',1000,8)
        targets=_u16(self.read('targets',1135,6),6,'targets')
        currents=_u16(self.read('current_ma_0_to_4',1105,5),5,'current')
        positions=_u16(self.read('positions',1145,6),6,'actual positions')
        rotation=self.io.single(1110)
        _u16([rotation],1,'opposition current')
        self.event('v13_hand_read',name='opposition_current_ma',address=1110,values=[rotation])
        limits=_u16(self.read('live_current_limit_ma',1095,6),6,'live current limits')
        status=_u16(self.read('status',1085,6),6,'status')
        row=dict(identity=identity,targets=targets,positions=positions,current=currents+[rotation],
            current_limit_ma=limits,status=status,reads_are_sequential=True,
            started_monotonic=began,finished_monotonic=self.clock(),actual_position_arrival_required=False,
            physical_stop_verified=False,holding_verified=False)
        self.report['last_snapshot']=copy.deepcopy(row)
        self.event('v13_hand_snapshot',snapshot=row);self.save()
        if any(v not in range(6) for v in status):self._fault('Invalid actual hand status',row)
        if any(v in (3,4) for v in status):self._fault('Real hand protection3/4 reported',row)
        if limits!=self.sequence.profile['current_limit_ma']:
            self._fault('Current hardware current-limit configuration changed',row)
        if any(limit<=0 or current>limit for current,limit in zip(row['current'],limits)):
            self._fault('Measured current exceeds current hardware limit',row)
        if 5 in status:
            self.report['normal_status5_observations']=self.report.get('normal_status5_observations',0)+1
            self.event('normal_status5_observed_no_target_change',status=status,
                no_failure_or_arrival_wait=True)
            self.save()
        self.guard()
        return row

    def monitor_loaded(self,targets,anchor=None):
        expected=_u16(targets,6,'planned targets')
        row=self.snapshot()
        if any(abs(a-b)>2 for a,b in zip(row['targets'],expected)):
            self._fault('Hand target readback differs from this explicit phase',row)
        self.report['last_loaded_monitor']=copy.deepcopy(row);self.save()
        return row

    def preflight_loaded(self,targets,anchor=None):
        self._critical_configuration()
        return self.monitor_loaded(targets,anchor)

    def preflight_empty(self,targets):return self.preflight_loaded(targets)

    def _phase(self,name,start_targets,end_targets):
        self._critical_configuration()
        before=self.snapshot()
        if start_targets is not None and any(abs(a-b)>2 for a,b in zip(before['targets'],start_targets)):
            self._fault('Hand start targets differ before '+name,before)
        if self.sequence.bin_motion is not None:self._fault('No hand phase writes while arm motion is active',before)
        self.guard();self.io.begin_phase(name)
        expected=before['targets'][:]
        for address,values in PHASE_STEPS[name]:
            self.guard()
            offset=address-1135
            matching=all(abs(expected[offset+i]-v)<=2 for i,v in enumerate(values))
            if self.io.permit_step(address,values,already_matching=matching):
                self.io.write_group(address,list(values))
            expected[offset:offset+len(values)]=list(values)
        # One fresh post-sequence group, never a target-arrival/settling loop.
        result=self.monitor_loaded(end_targets)
        self.io.finish_phase()
        self.report.setdefault('new_grasp_hand_phases',[]).append(dict(name=name,
            before=copy.deepcopy(before),after=copy.deepcopy(result),
            actual_arrival_waited=False,actual_settling_waited=False,automatic_retry=False))
        self.save();return result

    def prepare_at_high(self):return self._phase('prepare',None,PREPARE)
    def hold_pair(self):return self._phase('hold',PREPARE,PREPARE)
    def grasp_ordered(self):return self._phase('grasp',PREPARE,GRIP)

    def drop(self):
        if (getattr(self.sequence.authority,'selected_bin_endpoint_qualified',False) is not True
                or getattr(self.sequence.authority,'continuous_loaded_route_completed',False) is not True):
            self._fault('Normal opening requires this actual selected-bin endpoint')
        return self._phase('drop',GRIP,DROP)

    def open_at_bin(self,targets,anchor=None):
        if list(targets)!=GRIP:self._fault('Normal opening must follow the exact new grip target')
        return self.drop()

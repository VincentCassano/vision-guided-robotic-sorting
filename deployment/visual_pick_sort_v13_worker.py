"""One fresh visual target, new wrist/ordered grip, lift and continuous bin route.

SDK only in this actual Linux/aarch64 child. No keyboard waits, old wrist source,
payload/frame/protection setters, retries, or automatic fault opening/return.
"""
import argparse
import copy
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import socket
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
MANIFEST=ROOT/'deployment/visual-pick-sort-v13-files.sha256'
ANCHOR=ROOT/'deployment/new_grasp_reference_200928/new_task_anchor.json'
ANCHOR_SHA='b6405647fe47db40885818163115e9b54f1acb8030f2faac01bcb004eacae8a5'
PRE=[35000,7500,9480,30000,65535,65535]
GRIP=[40000,12500,14480,30000,65535,65535]
DROP=[0,0,0,0,0,65535]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def delivery(expected):
    if sha(MANIFEST)!=expected:raise ValueError('Current delivery manifest changed')
    checked={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT not in path.parents or name in checked or sha(path)!=digest:
            raise ValueError('Current delivery differs: '+name)
        checked[name]=digest
    if not checked:raise ValueError('Empty delivery manifest')
    return checked


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,required=True)
    parser.add_argument('--target-record',type=Path,required=True)
    parser.add_argument('--sdk-dir',type=Path,required=True)
    parser.add_argument('--manifest-sha256',required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    args=parser.parse_args()
    folder=args.folder.resolve();folder.mkdir(parents=True,exist_ok=True)
    report=dict(scope='new_wrist_single_black_shell_visual_pick_sort_v13',revision='fix2',phase='preflight',
        outcome='starting',exceptions=[],cleanup_errors=[],execution_ready=False,
        arm_actuator_writes=0,arm_stop_writes=0,hand_register_writes=0,
        controller_configuration_writes=0,configuration_writes=0,selected_color='black',
        actual_hand_arrival_required=False,status5_or_unreached_is_failure=False,
        old_wrist_or_task_offsets_used=False,old_inflight_ring_middle_tightening_used=False,
        automatic_fault_open=False,automatic_retry=False,automatic_return=False,
        offline_tests_executed=False,object_lift=False,holding_sensor_verified=False,
        shell_in_selected_bin_observed_by_operator=False,physical_stop_verified=False,
        SDK_cleanup_finished=False,started_timestamp_utc=datetime.now(timezone.utc).isoformat())
    robot=seq=None
    def save():
        from known_cylinder_records import dump
        from capture_installed_hand_geometry import _json_safe
        dump(folder/'worker_result.json',_json_safe(report))
    def append(stream,row):
        from capture_installed_hand_geometry import _json_safe
        stream.write(json.dumps(_json_safe(row),ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
    save()
    try:
        if platform.system()!='Linux' or platform.machine()!='aarch64':
            raise RuntimeError('Actual reviewed Linux aarch64 robot required; no SDK loaded')
        report['delivery_source_sha256']=delivery(args.manifest_sha256)
        source=json.loads(args.target_record.read_bytes());report['target_record_sha256']=sha(args.target_record)
        if sha(ANCHOR)!=ANCHOR_SHA or source.get('source_anchor_sha256')!=ANCHOR_SHA:
            raise ValueError('New healthy seated wrist reference bytes differ')
        anchor=json.loads(ANCHOR.read_bytes())
        if source.get('anchor')!=anchor:raise ValueError('Target source is not the accepted new wrist anchor')
        from visual_pick_sort_v13_source import accepted_target
        anchor,candidate=accepted_target(source)
        if source.get('candidate')!=candidate:raise ValueError('Fresh visual target/camera/plane derivation differs')
        permit=source.get('scene_permission',{})
        if any(permit.get(key) is not True for key in ('hand_empty','inert_shell','fixed_head_board_mount',
                'exclusive','onsite_stop','whole_route_min20mm_clear','continuous_cycle_authorized')):
            raise ValueError('Current empty-hand/full-route/continuous-cycle authorization missing')
        report.update(accepted_visual_source=source,source_anchor_sha256=ANCHOR_SHA,
            grasp_sequence=['thumb40000','index12500','middle14480','lift140mm','queued_black_bin','open_five'],
            planned_route_clearance_source='onsite_operator_upfront_not_sensor_or_old_ruler_value',
            prepare_targets=PRE[:],grasp_targets=GRIP[:],normal_drop_targets=DROP[:])
        save()
        from known_cylinder_records import dump
        from run_arm_hand_sequence import Sequence
        from school_bin_cycle_v4_sequence import SchoolBinCycle
        from high_descent_prepare_worker import same_pose
        from battery_grasp.real_grasp import orientation_angle_rad
        from supported_contact_transport import PhaseAuthority
        from visual_pick_sort_v13_hand import CycleTransport,CycleHand
        from visual_pick_sort_v7_connected import move_connected_transfer
        from visual_run_configuration_v3 import observe_configuration,bind_or_require_frames,require_current_configuration
        from pregrasp_measurement_worker import source_audit,NATIVE_SHA
        from inspect_left_hand import connect
        from sdk_environment import collect_sdk_environment
        from taught_model_readonly import collect_model_parameters
        from battery_grasp.taught_dh_model import EXPECTED_DH
        from arm_protection_transport import ProtectionClient
        from high_trial_tip_readonly import collect
        from school_battery_transport import WRITE
        import fcntl
        report['sdk_source_audit']=source_audit(args.sdk_dir);save()
        profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
        authority=PhaseAuthority()
        authority.selected_bin_endpoint_qualified=False
        authority.continuous_loaded_route_completed=False
        authority.continuous_cycle_permission=copy.deepcopy(permit)
        authority.held_opposition_target=65535
        authority.normal_drop_targets=DROP[:]
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as twire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            model=collect_model_parameters(folder/'fresh_model');report['fresh_model']=model;save()
            if (not model['identity_valid'] or not model['dh_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Fresh controller identity/firmware/DH differs')
            with (folder/'current_configuration_wire.jsonl').open('w',encoding='utf8') as cwire:
                run=observe_configuration(cwire)
            report['observed_run_configuration']=run;save()

            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                    report['hand_register_writes']+=1;save()

            class Route(Sequence):
                def __init__(self,*a,**kw):
                    super().__init__(*a,**kw)
                    self.folder=folder;self.authority=authority;self.drive_limits=None
                    self.bin_motion=None;self.notification=None;self.bin_route=None;self.plan=None
                    self.route_allowed=True;self.protection_prepared=True;self.active_motion=False
                    self.load_targets=None;self.load_anchor=None;self.hand_motion_enabled=False
                    self.tuned_recipe=GRIP[:];self.tuned_drop_targets=DROP[:]

                def full_arm(self):
                    arm=super().full_arm();bind_or_require_frames(run,arm['frame_signatures'])
                    if arm['frame_names']!={'work':'World','tool':'Arm_Tip'}:
                        raise RuntimeError('Fresh World/Arm_Tip geometry differs')
                    if self.reference is not None and arm['limits']!=self.reference['limits']:
                        raise RuntimeError('Current joint limits/configuration changed in this run')
                    return arm

                def arm_state(self):
                    if (folder/'abort.json').exists():raise KeyboardInterrupt('Parent requested stop')
                    q,p=super().arm_state()
                    if self.drive_limits is not None and any(min(v-a,b-v)<3 for v,a,b in zip(q,
                            self.drive_limits['minimum'],self.drive_limits['maximum'])):
                        raise RuntimeError('Actual drive joint reserve below3degrees')
                    motion=self.bin_motion
                    if motion is not None and motion['method']=='Movel_Cmd':
                        start,target=motion['start']['pose'],motion['target']
                        angle=orientation_angle_rad(p[3:],start[3:])
                        failed=[]
                        if angle>.005:failed.append('rotation_over0.005rad')
                        if motion.get('corridor_kind')=='line_to_live_hover':
                            delta=[b-a for a,b in zip(start[:3],target[:3])]
                            squared=sum(v*v for v in delta)
                            if squared<=0:raise RuntimeError('Nonzero current alignment segment required')
                            fraction=sum((p[i]-start[i])*delta[i] for i in range(3))/squared
                            bounded=max(0.,min(1.,fraction))
                            closest=[start[i]+bounded*delta[i] for i in range(3)]
                            error=math.dist(p[:3],closest)
                            if error>.001:failed.append('off_commanded_line_segment_over1mm')
                            report['last_current_alignment_corridor']=dict(start=start,target=target,
                                actual_pose=p,progress_fraction=fraction,segment_error_mm=error*1000,
                                rotation_rad=angle,failed_checks=failed)
                        else:
                            xy=math.dist(p[:2],start[:2])
                            if xy>.001:failed.append('xy_over1mm')
                            if not min(start[2],target[2])-.001<=p[2]<=max(start[2],target[2])+.001:
                                failed.append('z_outside_this_command')
                            report['last_current_vertical_corridor']=dict(start=start,target=target,
                                actual_pose=p,xy_mm=xy*1000,rotation_rad=angle,failed_checks=failed)
                        save()
                        if failed:raise RuntimeError('Current Cartesian command corridor changed')
                    return q,p

                def call(self,name,*values,**options):
                    if not name.startswith('Get_'):
                        if name=='Move_Stop_Cmd':
                            if not self.arm_stop_attempted:raise RuntimeError('Stop is fault-only')
                        elif name not in ('Movel_Cmd','Movej_P_Cmd') or not self.route_allowed:
                            raise RuntimeError('No arm/configuration write outside the exact route')
                        elif isinstance(self.notification,dict) and self.notification.get('kind')=='connected_school_transfer' and self.bin_motion is not None and self.bin_motion.get('connected'):
                            index=self.notification['next_submit']
                            if index>=3:raise RuntimeError('Connected queue already submitted')
                            planned=self.notification['commands'][index]
                            if (name!=planned['method'] or len(values)!=2 or values[0]!=planned['target']
                                    or values[1]!=planned['speed'] or options!={'trajectory_connect':planned['trajectory_connect'],'r':0,'block':False}):
                                raise RuntimeError('Connected exact ordered command differs')
                            self.notification['next_submit']+=1
                        elif (self.bin_motion is None or name!=self.bin_motion['method']
                                or len(values)!=2 or values[0]!=self.bin_motion['target']
                                or options!={'trajectory_connect':0,'r':0,'block':False}):
                            raise RuntimeError('No exact current segment capability')
                    if name in ('Get_Collision_Stage','Get_Self_Collision_Enable') and self.io is not None:
                        raise RuntimeError('Close hand channel before raw protection query')
                    value=super().call(name,*values,**options)
                    if name=='Get_Current_Arm_State':
                        # Preserve this exact frame before inherited checks can reject it.
                        # A previous healthy last_arm_state is not this failed sample.
                        report['last_arm_raw_query']=copy.deepcopy(value)
                        report.pop('joint_reserve_diagnostic',None)
                        if (isinstance(value,list) and len(value)==5
                                and isinstance(value[1],list) and len(value[1])==6
                                and all(type(q) in (int,float) and math.isfinite(q) for q in value[1])
                                and self.reference is not None):
                            rows=[]
                            for index,(q,low,high) in enumerate(zip(value[1],
                                    self.reference['limits']['minimum'],self.reference['limits']['maximum'])):
                                rows.append(dict(joint=index+1,angle_degrees=q,minimum_degrees=low,
                                    maximum_degrees=high,lower_margin_degrees=q-low,
                                    upper_margin_degrees=high-q,reserve_degrees=min(q-low,high-q)))
                            report['joint_reserve_diagnostic']=dict(phase=report.get('phase'),
                                current_motion=copy.deepcopy(self.bin_motion),joints=rows,
                                query_status=value[0],arm_error=value[3],system_error=value[4],
                                failed_joints=[r['joint'] for r in rows if r['reserve_degrees']<3],
                                required_reserve_degrees=3,source='this_exact_raw_query',
                                qualified_startup_reference=False)
                        save()
                    if name in ('Movel_Cmd','Movej_P_Cmd'):
                        if not (isinstance(self.notification,dict) and self.notification.get('kind')=='connected_school_transfer'
                                and self.bin_motion is not None and self.bin_motion.get('connected')):
                            self.notification=dict(acknowledged=type(value) is int and value==0,
                                event_seen=False,method=name,phase=report['phase'])
                    return value

                def close_hand(self):
                    if self.io is not None:
                        io,self.io=self.io,None;self.hand_motion_enabled=False
                        io.disarm_motion()
                        try:io.sock.close()
                        except BaseException as exc:
                            report['cleanup_errors'].append(dict(method='hand.close',error=repr(exc)));save();raise

                def protections(self):
                    if self.io is not None:raise RuntimeError('Protection requires the hand lane closed')
                    if self.call('Get_Collision_Stage')!=[0,8] or self.call('Get_Self_Collision_Enable')!=[0,True]:
                        raise RuntimeError('Fresh actual collision protections not8/True')

                def current_configuration(self):
                    self.close_hand()
                    with (folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as cwire:
                        require_current_configuration(run,cwire)

                def stationary(self,expected):
                    arm=self.full_arm()
                    if not same_pose(arm,expected) or arm['limits']!=expected['limits']:
                        raise RuntimeError('Arm changed after the fresh current start')
                    self.guard_target,self.guard_pose=arm['joints'][:],arm['pose'][:]
                    return arm

                def move_segment(self,arm,target,method,phase,hand,speed=2,immediate_after_grip=False):
                    fresh=self.stationary(arm)
                    if (math.dist(fresh['pose'][:3],target[:3])<=.0005
                            and orientation_angle_rad(fresh['pose'][3:],target[3:])<=.005):
                        report.setdefault('arm_segments',[]).append(dict(phase=phase,method=method,
                            target_sdk_pose_m_rad=target[:],endpoint=copy.deepcopy(fresh),
                            skipped_already_at_target=True,arm_command_sent=False,
                            completion_notification_required=False))
                        save();return fresh
                    if not immediate_after_grip:
                        self.current_configuration();self.protections()
                        replacement=self.hand_factory(self,True);hand.io=replacement.io
                        hand._critical_configuration();hand.monitor_loaded(self.load_targets,self.load_anchor)
                        self.stationary(fresh)
                    elif (self.io is None or not report.get('ordered_grip_ack_and_readback_pass')
                            or self.load_targets!=GRIP or not report.get('precontact_protections_checked')):
                        raise RuntimeError('Immediate lift requires this current exact grip and precontact checks')
                    self.bin_motion=dict(method=method,target=target[:],start=copy.deepcopy(fresh),
                        corridor_kind='line_to_live_hover' if phase=='aligning_new_visual_target_at_high'
                            and method=='Movel_Cmd' else 'vertical_or_joint')
                    report.update(phase=phase,execution_ready=True,current_school_motion=copy.deepcopy(self.bin_motion))
                    self.active_motion=True;self.arm_command_attempted=True
                    report['arm_command_attempted']=True;report['arm_actuator_writes']+=1;save()
                    began=self.clock();rc=self.call(method,target,speed,trajectory_connect=0,r=0,block=False)
                    if type(rc) is not int or rc!=0:raise RuntimeError('Current arm command rejected/uncertain; no retry')
                    previous=None;since=None;groups=0;last_hand=began;endpoint=None
                    while self.clock()-began<100:
                        q,p=self.arm_state()
                        if self.clock()-last_hand>=.5:
                            hand.monitor_loaded(self.load_targets,self.load_anchor);last_hand=self.clock()
                        reached=math.dist(p[:3],target[:3])<=.0005 and orientation_angle_rad(p[3:],target[3:])<=.005
                        quiet=(previous is not None and max(abs(a-b) for a,b in zip(q,previous[0]))<=.02
                            and math.dist(p[:3],previous[1][:3])<=.0002
                            and orientation_angle_rad(p[3:],previous[1][3:])<=.002)
                        if reached and quiet:
                            if since is None:since=self.clock()
                            groups+=1
                            if groups>=3 and self.clock()-since>=1:
                                endpoint=self.full_arm()
                                if (endpoint['frame_signatures']!=fresh['frame_signatures'] or endpoint['limits']!=fresh['limits']
                                        or math.dist(endpoint['pose'][:3],target[:3])>.0005
                                        or orientation_angle_rad(endpoint['pose'][3:],target[3:])>.005):
                                    raise RuntimeError('Current segment endpoint changed')
                                hand.monitor_loaded(self.load_targets,self.load_anchor)
                                if self.notification.get('event_seen') is not True:
                                    raise RuntimeError('Current segment completion notification missing')
                                break
                        else:since=None;groups=0
                        previous=q,p;self.pause(.1)
                    if endpoint is None:raise TimeoutError('Current segment feedback deadline exceeded')
                    self.active_motion=False
                    report.setdefault('arm_segments',[]).append(dict(phase=phase,method=method,
                        target_sdk_pose_m_rad=target[:],speed_percent=speed,endpoint=copy.deepcopy(endpoint),
                        elapsed_seconds=self.clock()-began,notification=copy.deepcopy(self.notification),
                        physical_object_retention_sensor_verified=False))
                    self.guard_target,self.guard_pose=endpoint['joints'][:],endpoint['pose'][:]
                    self.bin_motion=None;report['execution_ready']=False;save();return endpoint

                def fault(self):
                    self.authority._terminal_latched=True
                    if self.io is not None:self.io.latch_fault('Current cycle fault; no subsequent hand writes')
                    if self.active_motion and not self.arm_stop_attempted:
                        self.arm_stop_attempted=True;report['arm_stop_writes']+=1;save()
                        try:report['arm_stop_attempt']=dict(result=self.call('Move_Stop_Cmd',block=False),physical_stop_verified=False)
                        except BaseException as exc:report['arm_stop_attempt']=dict(error=repr(exc),physical_stop_verified=False)
                    save()

            def factory(active,guarded):
                if not guarded or active.io is not None:raise RuntimeError('One scoped hand socket required')
                sock=socket.create_connection(('169.254.128.18',8080),timeout=3)
                active.io=CycleTransport(sock,hand_log,authority,lambda:active.notification if active.bin_motion is not None else None)
                return CycleHand(active,active.io,report,save,samples)

            robot=connect(args.sdk_dir)
            seq=Route(robot,None,profile,report,save,samples,factory,
                protection_client=ProtectionClient(lambda row:append(pwire,row)))
            try:
                metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata;save()
                if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                        or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                        or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):
                    raise RuntimeError('Actual SDK/native/real mode differs')
                initial=seq.full_arm();seq.reference=copy.deepcopy(initial)
                if initial['limits']!=anchor['reference_arm']['limits']:
                    raise RuntimeError('Current joint configuration differs from new taught reference')
                seq.guard_target,seq.guard_pose=initial['joints'][:],initial['pose'][:]
                report.update(startup_reference=copy.deepcopy(initial),reference_source='this_run_fresh_current_arm_not_assumed_at_anchor',
                    old_fault_last_arm_state_used=False);save()
                low,high=seq.safety_reads();seq.drive_limits=dict(minimum=low,maximum=high)
                install=report['arm_safety_reads']['Get_Install_Pose']
                if (not isinstance(install,list) or len(install)!=4 or install[0]!=0
                        or any(abs(a-b)>.001 for a,b in zip(install[1:],[0,45,0]))):
                    raise RuntimeError('Actual installation differs')
                tip=collect(lambda row:append(twire,row));report['live_tip_parameters']=tip;save()
                if tip['line_speed_m_s']>.25 or tip['line_acc_m_s2']>1.6 or tip['angular_speed_rad_s']>.6 or tip['angular_acc_rad_s2']>4:
                    raise RuntimeError('Actual tip parameters exceed recorded bounds')
                seq.protections()
                hand=factory(seq,True);hand.configuration();startup=hand.snapshot()
                seq.load_targets=startup['targets'][:];seq.load_anchor=copy.deepcopy(startup)
                hand.monitor_loaded(seq.load_targets,seq.load_anchor)
                seq.stationary(initial);report['startup_hand']=copy.deepcopy(startup);save()
                target=candidate['pregrasp_tcp_candidate_m_rad'][:]
                # Avoid the old current-XY/+140mm point that exhausted J4 reserve.
                # Clear the immediate area first, then align toward the live target
                # while rising to the unchanged final hover; never lower automatically.
                entry_z=max(initial['pose'][2],target[2]+.100)
                high_z=max(initial['pose'][2],target[2]+.140)
                report['empty_approach_waypoints']=dict(
                    entry_height_above_target_mm=100,final_hover_height_above_target_mm=140,
                    current_xy_full_140mm_raise_used=False,
                    entry_pose=initial['pose'][:2]+[entry_z]+initial['pose'][3:],
                    final_hover_pose=target[:2]+[high_z]+target[3:],
                    path_scope='onsite_upfront_whole_sweep_clearance; new connection not preaccepted',
                    automatic_fault_recovery=False)
                save()
                raised=initial
                if entry_z-initial['pose'][2]>.0005:
                    pose=initial['pose'][:];pose[2]=entry_z
                    raised=seq.move_segment(initial,pose,'Movel_Cmd','raising_empty_to_entry_100mm',hand,speed=2)
                else:
                    report['empty_raise_skipped_already_at_or_above_entry']=True;save()
                seq.current_configuration();seq.protections()
                if seq.io is None:hand=factory(seq,True)
                hand._critical_configuration();seq.stationary(raised)
                prepared=hand.prepare_at_high();seq.load_targets=PRE[:];seq.load_anchor=copy.deepcopy(prepared)
                seq.close_hand()
                hand=factory(seq,True);hand._critical_configuration()
                rotated_pose=raised['pose'][:3]+target[3:]
                rotated=seq.move_segment(raised,rotated_pose,'Movej_P_Cmd','orienting_new_wrist_at_high',hand,speed=2)
                hover=target[:];hover[2]=high_z
                aligned=seq.move_segment(rotated,hover,'Movel_Cmd','aligning_new_visual_target_at_high',hand,speed=2)
                end=seq.move_segment(aligned,target,'Movel_Cmd','descending_new_wrist_to_visual_target',hand,speed=2)
                # All slow static/route/protection queries precede intended contact.
                seq.current_configuration();seq.protections();seq.stationary(end)
                SchoolBinCycle.prepare_bin_route(seq,end)
                from visual_pick_sort_v13_source import back40_offset
                delta=back40_offset();drop=seq.bin_route['bins']['black'];original_drop=drop['pose'][:]
                drop['pose']=[v+delta[i] if i<3 else v for i,v in enumerate(original_drop)]
                report.update(school_bin_route=copy.deepcopy(seq.bin_route),
                    bin_task_offset=dict(distance_mm=40,direction='toward_robot_fixed_camera_horizontal',
                        delta_world_mm=[v*1000 for v in delta],original=original_drop,target=drop['pose'][:]),
                    precontact_protections_checked=True,loaded_keyboard_waits=0);save()
                hand=factory(seq,True);hand._critical_configuration();hand.preflight_loaded(PRE,seq.load_anchor)
                report['phase']='setting_little_then_ring_hold';save();hand.hold_pair()
                report['phase']='ordered_thumb_index_middle_grasp';save()
                grip=hand.grasp_ordered();seq.load_targets=GRIP[:];seq.load_anchor=copy.deepcopy(grip)
                report.update(ordered_grip_ack_and_readback_pass=True,contact_actual_arrival_wait=False,
                    commanded_grip_targets=GRIP[:],grip_feedback=copy.deepcopy(grip));save()
                upper_pose=end['pose'][:];upper_pose[2]+=.140
                upper=seq.move_segment(end,upper_pose,'Movel_Cmd','lifting_new_ordered_grip_140mm',hand,
                    speed=5,immediate_after_grip=True)
                report.update(loaded_lift_performed=True,loaded_arm_up_feedback_observed=True,
                    actual_object_lift_sensor_verified=False);save()
                rows=seq.bin_route['transit']+[seq.bin_route['bins']['black']]
                seq.active_motion=True
                final=move_connected_transfer(seq,upper,rows,hand)
                seq.active_motion=False
                authority.continuous_loaded_route_completed=True
                authority.selected_bin_endpoint_qualified=True
                seq.close_hand();seq.stationary(final);seq.current_configuration();seq.protections()
                hand=factory(seq,True);hand._critical_configuration();hand.preflight_loaded(GRIP,seq.load_anchor)
                report['phase']='opening_five_flex_at_black_bin';save()
                opened=hand.open_at_bin(GRIP,seq.load_anchor)
                seq.load_targets=DROP[:];seq.load_anchor=copy.deepcopy(opened)
                seq.close_hand()
                report.update(phase='completed',outcome='motion_cycle_completed_waiting_operator_result',
                    final_arm=copy.deepcopy(final),final_hand=copy.deepcopy(opened),
                    loaded_transfer_performed=True,bin_opening_targets_written=True,
                    actual_object_lift_not_inferred_from_arm_encoders=True,
                    shell_in_selected_bin_observed_by_operator=False,automatic_return=False)
                save()
            except BaseException:
                seq.fault();raise
            finally:seq.close_hand()
    except BaseException as exc:
        report.update(phase='fault',outcome='worker_aborted_no_automatic_retry',execution_ready=False)
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()));save()
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:
                    value=getattr(robot,name)()
                    report.setdefault('sdk_cleanup_results',[]).append(dict(method=name,raw_repr=repr(value)))
                    if name=='RM_API_UnInit' and (type(value) is not int or value!=0):
                        report['cleanup_errors'].append(dict(method=name,error='Nonzero or unknown SDK cleanup',raw_repr=repr(value)))
                except BaseException as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
            report['SDK_cleanup_finished']=not report['cleanup_errors']
        if report['cleanup_errors']:report['outcome']='failed_cleanup'
        report.update(execution_ready=False,closed_timestamp_utc=datetime.now(timezone.utc).isoformat());save()
    return 0 if report.get('outcome')=='motion_cycle_completed_waiting_operator_result' and not report['exceptions'] and not report['cleanup_errors'] else 1


if __name__=='__main__':raise SystemExit(main())

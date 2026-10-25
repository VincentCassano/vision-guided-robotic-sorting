#!/usr/bin/env python3
"""Supervised empty-hand J6 +1 degree, settled arm, verified hand pose, open.

This is a bounded commissioning test, not an automatic pick/transfer executor.
No IK, Cartesian trajectory, payload, speed-register or frame writes.
Optional explicit setup permits only collision stage8 and self-collision True.
"""
import argparse
import copy
import ctypes
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import time
import traceback
import zipfile

from arm_protection_transport import ProtectionClient, METHODS as PROTECTION_METHODS
from capture_installed_hand_geometry import _arm, _json_safe, _vector
from inspect_left_arm import encode, snapshot
from inspect_left_hand import connect
from run_school_battery_hand import Hand, ADAPTED_PLAN
from hand_feedback_policy import FeedbackHand
from school_battery_transport import Transport
from sdk_environment import collect_sdk_environment

ROOT = Path(__file__).resolve().parents[1]
NATIVE_SHA = '052f94a92d5c7320f227ea811916375fa0d352250329570bd50e0c7abe603ad5'
REFERENCE_SHA = 'd4ac7cb27717548ae096922afb367779f3194efe5bb96da1adbf22b0d5f92603'
RESUME_SHA = '6f4f6964e2f1ac3e8b5fa28cbce0954ed5a75f4e145527f61c451bb7f299d087'
DELTA_DEG = 1.0
SPEED_PERCENT = 5
SOURCE_NAMES = ('run_arm_hand_sequence.py', 'capture_installed_hand_geometry.py',
    'inspect_left_arm.py', 'inspect_left_hand.py', 'arm_sdk_reviewed_hashes.json',
    'run_school_battery_hand.py', 'school_battery_transport.py', 'run_school_battery.py',
    'school_battery_pose.json', 'sdk_environment.py', 'arm_protection_transport.py',
    'hand_feedback_policy.py')
POSE_WRITES = {(s['address'],tuple(s['values'])) for s in ADAPTED_PLAN} | {
    (1135,(0,)), (1136,(0,0,0,0)), (1140,(728,))}


class SequenceTransport(Transport):
    def write_group(self, address, values):
        if (address,tuple(values)) not in POSE_WRITES:
            raise ValueError('Sequence permits verified pose targets only; no speed/45000 writes')
        guard = getattr(self, 'before_write', None)
        if guard is not None: guard()
        return super().write_group(address,values)


def accepted_reference(document, profile):
    if document.get('schema_version') != 1 or document.get('outcome') != 'references_recorded':
        raise ValueError('Complete accepted reference capture required')
    if document.get('read_only') is not True or document.get('hardware_writes') != 0:
        raise ValueError('Reference must be read-only')
    if document.get('exceptions') or document.get('cleanup_errors'):
        raise ValueError('Reference contains exceptions or cleanup errors')
    roles = {r.get('role') for r in document.get('points', []) if r.get('accepted') is True}
    if not {'safe_reference', 'pick_hover'} <= roles:
        raise ValueError('Two accepted references required')
    rows = [r for r in document['points'] if r.get('role') == 'safe_reference']
    if len(rows) != 1 or rows[0].get('accepted') is not True:
        raise ValueError('Exactly one accepted safe reference required')
    row = rows[0]
    clearance = row.get('clearance_mm')
    if type(clearance) not in (int, float) or not math.isfinite(clearance) or clearance < 100:
        raise ValueError('Reference clearance must be at least100 mm')
    analysis = row.get('analysis', {})
    if analysis.get('diagnostic_pass') is not True or analysis.get('hand_pose_classification') != 'open':
        raise ValueError('Safe reference did not pass open-hand bracket')
    arm = _arm(row['arm_query_brackets']['after']['queries'])
    if row['hand_preflight_after']['targets'] != profile['poses']['open']:
        raise ValueError('Reference hand does not match frozen open pose')
    return arm


def check_reference(live, reference):
    if live['frame_signatures'] != reference['frame_signatures'] or live['limits'] != reference['limits']:
        raise RuntimeError('Frames or joint configuration changed; no motion')
    if any(v < 3 for v in live['reserve']):
        raise RuntimeError('Less than3 degrees of joint reserve')
    if any(abs(a-b) > .5 for a,b in zip(live['joints'], reference['joints'])):
        raise RuntimeError('Not at recorded observation reference; use teaching controls, never automatic return')
    if math.dist(live['pose'][:3], reference['pose'][:3]) > .003:
        raise RuntimeError('TCP differs from observation reference by more than3 mm')
    if any(abs(math.atan2(math.sin(a-b), math.cos(a-b))) > .01
           for a,b in zip(live['pose'][3:], reference['pose'][3:])):
        raise RuntimeError('Wrist orientation changed from observation reference')


def accepted_resume(document, reference, profile):
    """Derive only the audited post-wrist reference, never a movement target.

    CLI also pins the exact original result bytes. Old feedback is evidence for
    resuming scope, not permission to omit fresh device/position checks.
    """
    if (document.get('schema_version') != 1
            or document.get('outcome') != 'aborted_no_further_motion'
            or document.get('preflight_pass') is not True
            or document.get('arm_move_feedback_pass') is not True
            or document.get('cleanup_errors')
            or document.get('completed_adapted_stages') != 0):
        raise ValueError('Require audited200240 stopped sequence and completed wrist feedback')
    prepared = ADAPTED_PLAN[0]['values']
    moves = document.get('moves', [])
    if (len(moves) != 1 or moves[0].get('step') != ADAPTED_PLAN[0]
            or moves[0].get('expected') != prepared):
        raise ValueError('Resume requires exactly the previously issued prepare target')
    if document.get('configuration') != profile['observed_configuration']:
        raise ValueError('Prior hand configuration does not match the frozen profile')
    if document.get('fresh_protection_values',{}).get('Get_Collision_Stage') != [0,8] or (
            document.get('fresh_protection_values',{}).get('Get_Self_Collision_Enable') != [0,True]):
        raise ValueError('Prior raw protection replies must confirm8/true')
    candidate = document.get('candidate', {})
    start = _vector(candidate.get('start_joints'),6,'prior start joints')
    target = _vector(candidate.get('target_joints'),6,'prior target joints')
    if (any(abs(a-b)>1e-6 for a,b in zip(start[:5],target[:5]))
            or abs(target[5]-start[5]-1)>1e-6
            or candidate.get('j6_delta_degrees') != 1):
        raise ValueError('Prior arm move was not the accepted J6 +1 degree scope')
    post_fault = document.get('post_fault', [])
    if len(post_fault) != 3:
        raise ValueError('Complete prior post-fault read-only evidence required')
    for row in post_fault:
        actual = _vector(row.get('positions'),6,'prior hand positions')
        targets = _vector(row.get('targets'),6,'prior hand targets')
        if (row.get('status') != [2]*6
                or any(abs(a-b)>150 for a,b in zip(actual,prepared))
                or any(abs(a-b)>2 for a,b in zip(targets,prepared))):
            raise ValueError('Prior feedback did not reach the prepare target')
    derived = copy.deepcopy(reference)
    derived['joints'] = target
    derived['pose'] = _vector(document.get('arm_hand_phase_pose'),6,'prior hand-phase pose')
    check_reference(dict(reference, joints=start),reference)
    if any(min(q-a,b-q)<3 for q,a,b in zip(target,
            reference['limits']['minimum'],reference['limits']['maximum'])):
        raise ValueError('Prior resume reference lacks3 degrees reserve')
    return derived


def wrist_target(live):
    # Copy LIVE joints: using the older reference would also move J1--5.
    target = live['joints'][:]
    target[5] += DELTA_DEG
    if any(min(q-low, high-q) < 3 for q,low,high in
           zip(target, live['limits']['minimum'], live['limits']['maximum'])):
        raise RuntimeError('Wrist target lacks3 degrees of reserve')
    return target


def check_resumed_reference(live, reference):
    check_reference(live, reference)
    if (any(abs(a-b)>.08 for a,b in zip(live['joints'],reference['joints']))
            or math.dist(live['pose'][:3],reference['pose'][:3])>.0005
            or any(abs(math.atan2(math.sin(a-b),math.cos(a-b)))>.005
                   for a,b in zip(live['pose'][3:],reference['pose'][3:]))):
        raise RuntimeError('Arm changed from audited post-wrist position; no automatic reposition')


class Sequence:
    def __init__(self, robot, reference, profile, report, save, samples,
                 hand_factory, confirm=input, clock=time.monotonic, pause=time.sleep,
                 enable_arm_protection=False, protection_client=None, resume_document=None):
        self.robot, self.reference, self.profile = robot, reference, profile
        self.report, self.save, self.samples = report, save, samples
        self.hand_factory, self.confirm, self.clock, self.pause = hand_factory, confirm, clock, pause
        self.arm_command_attempted = False
        self.arm_stop_attempted = False
        self.io = None
        self.guard_target = None
        self.guard_pose = None
        self.enable_arm_protection = enable_arm_protection
        self.protection_prepared = False
        self.protection_client = protection_client
        self.resume_document = resume_document
        self.start_hand_target = (ADAPTED_PLAN[0]['values'][:] if resume_document is not None
                                  else profile['poses']['open'][:])

    def event(self, name, **fields):
        self.samples.write(json.dumps(_json_safe(dict(event=name,monotonic=self.clock(),**fields)))+'\n')
        self.samples.flush()

    def call(self, name, *args, **kwargs):
        direct = name in PROTECTION_METHODS
        prefix = 'arm_protection_call' if direct else 'sdk_call'
        self.event(prefix+'_before', method=name, args=args, kwargs=kwargs)
        try:
            if direct and self.protection_client is None:
                raise RuntimeError('Fresh controller protection channel required; cached SDK fallback forbidden')
            receiver = self.protection_client if direct else self.robot
            raw = getattr(receiver, name)(*args, **kwargs)
            if name == 'Get_Install_Pose' and isinstance(raw,(list,tuple)):
                raw = [v.value if isinstance(v,(ctypes.c_float,ctypes.c_double)) else v for v in raw]
            answer = encode(raw)
        except (Exception, KeyboardInterrupt) as exc:
            self.event(prefix+'_exception', method=name, error=repr(exc))
            raise
        self.event(prefix+'_result', method=name, result=answer)
        if direct:
            self.report.setdefault('fresh_protection_values', {})[name] = answer
            self.save()
        return answer

    def full_arm(self):
        queries = snapshot(self.robot)
        self.event('sdk_full_snapshot', queries=queries)
        return _arm(queries)

    def arm_state(self):
        raw = self.call('Get_Current_Arm_State')
        if not isinstance(raw, list) or len(raw) != 5 or type(raw[0]) is not int or raw[0] != 0:
            raise RuntimeError('Arm state GET failed or malformed')
        joints, pose = _vector(raw[1],6,'arm joints'), _vector(raw[2],6,'arm pose')
        if any(type(e) is not int or e != 0 for e in raw[3:]):
            raise RuntimeError('Arm/system error: '+str(raw[3:]))
        if any(min(q-low, high-q) < 3 for q,low,high in zip(joints,
                self.reference['limits']['minimum'],self.reference['limits']['maximum'])):
            raise RuntimeError('Arm joint reserve dropped below3 degrees')
        self.report['last_arm_state'] = dict(joints=joints,pose=pose,raw=raw)
        self.save()
        return joints, pose

    def settled(self, target, travel_start=None, timeout=15):
        begin = self.clock(); stable_since = None; previous = None; count = 0
        while self.clock()-begin < timeout:
            joints, pose = self.arm_state()
            if travel_start is not None:
                if any(abs(joints[i]-travel_start[i]) > .2 for i in range(5)):
                    raise RuntimeError('Unexpected J1--5 movement')
                if not travel_start[5]-.2 <= joints[5] <= target[5]+.2:
                    raise RuntimeError('J6 outside bounded +1 degree travel')
            reached = all(abs(q-t) <= .08 for q,t in zip(joints,target))
            stationary = previous is not None and all(abs(q-p) <= .02 for q,p in zip(joints,previous[0]))
            stationary = stationary and math.dist(pose[:3],previous[1][:3]) <= .0002
            stationary = stationary and all(abs(math.atan2(math.sin(a-b),math.cos(a-b))) <= .002
                                           for a,b in zip(pose[3:],previous[1][3:]))
            if reached and stationary:
                if stable_since is None: stable_since = self.clock()
                count += 1
                if self.clock()-stable_since >= 2 and count >= 5:
                    self.event('arm_target_observed_stable',target=target,joints=joints,pose=pose,
                               observation_seconds=self.clock()-stable_since,
                               physical_stop_verified=False)
                    return joints,pose
            else:
                stable_since = None; count = 0
            previous = (joints,pose)
            self.pause(.1)
        raise RuntimeError('Arm feedback failed to settle at target; hand stays disabled')

    def safety_reads(self, allow_disabled=False):
        rows = {}
        for name in ('Get_Joint_EN_State','Get_Joint_Err_Flag','Get_Collision_Stage',
                     'Get_Joint_Drive_Min_Pos','Get_Joint_Drive_Max_Pos',
                     'Get_Joint_Current','Get_Joint_Temperature','Get_Joint_Voltage','Get_Self_Collision_Enable',
                     'Get_Arm_Software_Version','Get_Install_Pose'):
            try: rows[name] = self.call(name)
            except Exception as exc: rows[name] = dict(error=repr(exc))
        self.report['unknowns'] = ['Actual payload/center-of-mass not certified',
            'Full scene geometry and pick trajectory not certified',
            'Automatic contact stop/release not verified',
            'Software_Info ctrl/plan fields not reliable semantic version decoding',
            'Optional safety GET failures are recorded and never replaced by guesses']
        self.report['arm_safety_reads'] = rows; self.save()
        self.report['arm_protection_read_source'] = 'fresh_socket_raw_controller_JSON_not_SDK_cache'
        def good(name,length):
            value = rows[name]
            if not isinstance(value,list) or len(value)!=length or type(value[0]) is not int or value[0]!=0:
                raise RuntimeError('Required safety GET unsuccessful: '+name)
            return value[1:]
        enabled = good('Get_Joint_EN_State',2)[0]
        errors,brakes = good('Get_Joint_Err_Flag',3)
        if enabled != [1]*6 or errors != [0]*6 or brakes != [0]*6:
            raise RuntimeError('Joint enables/errors/brakes not ready')
        collision = good('Get_Collision_Stage',2)[0]
        if type(collision) is not int or not 0 <= collision <= 8:
            raise RuntimeError('Unknown/malformed collision level; no configuration or motion')
        if collision==0 and not allow_disabled:
            raise RuntimeError('Collision protection disabled/unknown; no parameter change or motion')
        self_collision = good('Get_Self_Collision_Enable',2)[0]
        if type(self_collision) is not bool:
            raise RuntimeError('Unknown/malformed self-collision state; no configuration or motion')
        if not self_collision and not allow_disabled:
            raise RuntimeError('Self-collision detection disabled; no motion')
        if self.protection_prepared and (collision!=8 or self_collision is not True):
            raise RuntimeError('Prepared protections changed outside this script; no further writes')
        low = _vector(good('Get_Joint_Drive_Min_Pos',2)[0],6,'drive minimum')
        high = _vector(good('Get_Joint_Drive_Max_Pos',2)[0],6,'drive maximum')
        if any(a>=b for a,b in zip(low,high)):
            raise RuntimeError('Invalid drive limit ranges')
        return low,high

    def prepare_arm_protection(self):
        """One-way opt-in protection setup; never retries or restores level0.

        Readback verifies configuration only, not collision stopping performance.
        It does not change the hand or certify its geometry/payload.
        """
        original=self.report['arm_safety_reads']
        expected=(original['Get_Collision_Stage'][1],original['Get_Self_Collision_Enable'][1])
        setup=dict(before=dict(collision_stage=expected[0],self_collision_enabled=expected[1]),
            target=dict(collision_stage=8,self_collision_enabled=True),writes=[],
            outcome='starting',collision_stop_verified=False,automatic_restore=False)
        self.report['arm_protection_setup']=setup;self.save()
        if expected==(8,True):
            setup['outcome']='already_enabled_readback_only'
            self.protection_prepared=True;self.save();return
        print('本机碰撞等级{}，自碰撞{}。将开启等级8（最灵敏）和自碰撞检测。'.format(*expected))
        print('保护配置会保留，不自动恢复关闭；这不证明能检测手接触或及时停止。')
        if self.confirm('确认空手、机械臂已实际停稳、退出示教且现场守急停；e=开启这两项保护，其他输入取消：').strip().lower()!='e':
            setup['outcome']='operator_cancelled_no_writes';self.save()
            raise RuntimeError('Operator cancelled protection setup; no movement')
        # The hand TCP was closed before input. Revalidate every safety property
        # after the human wait, and refuse any external configuration change.
        fresh=self.full_arm();check_reference(fresh,self.reference)
        low,high=self.safety_reads(allow_disabled=True)
        rows=self.report['arm_safety_reads']
        if (rows['Get_Collision_Stage'][1],rows['Get_Self_Collision_Enable'][1])!=expected:
            raise RuntimeError('Protection changed during operator confirmation; no setup writes')
        if any(rows[name]!=original[name] for name in ('Get_Joint_Drive_Min_Pos','Get_Joint_Drive_Max_Pos')):
            raise RuntimeError('Drive limits changed during confirmation; no setup writes')
        candidate=wrist_target(fresh)
        if any(min(q-a,b-q)<3 for q,a,b in zip(fresh['joints'],low,high)) or any(
                min(q-a,b-q)<3 for q,a,b in zip(candidate,low,high)):
            raise RuntimeError('Fresh drive reserve failed; no setup writes')
        self.settled(fresh['joints'])
        check_reference(self.full_arm(),self.reference)
        commands=[]
        if expected[0]!=8:commands.append(('Set_Collision_Stage',8,dict(block=True)))
        if expected[1] is not True:commands.append(('Set_Self_Collision_Enable',True,{}))
        self.protection_client.authorize_settings()
        for name,value,kwargs in commands:
            attempt=dict(method=name,value=value,outcome='attempted_result_unknown')
            setup['writes'].append(attempt);self.save()
            try: status=self.call(name,value,**kwargs)
            except (Exception,KeyboardInterrupt):
                setup['outcome']='setup_uncertain_no_retry';self.save();raise
            attempt['status']=status
            if type(status) is not int or status!=0:
                setup['outcome']='setup_rejected_no_retry';self.save()
                raise RuntimeError('Protection setup rejected/uncertain: '+name)
            attempt['outcome']='raw_controller_acknowledged_not_performance_verified';self.save()
        for i in range(2):
            collision=self.call('Get_Collision_Stage')
            self_collision=self.call('Get_Self_Collision_Enable')
            setup.setdefault('readbacks',[]).append(dict(collision=collision,self_collision=self_collision))
            self.save()
            if (not isinstance(collision,list) or len(collision)!=2
                    or type(collision[0]) is not int or type(collision[1]) is not int or collision!=[0,8]
                    or not isinstance(self_collision,list) or len(self_collision)!=2
                    or type(self_collision[0]) is not int or self_collision!=[0,True]
                    or type(self_collision[1]) is not bool):
                setup['outcome']='readback_mismatch_no_motion';self.save()
                self.report['next_action'] = ('Controller accepted settings but fresh replies do not confirm them. '
                    'Check controller protection configuration/compatibility onsite; do not repeat settings or bypass the gate.')
                raise RuntimeError('Fresh controller protection mismatch: collision={}, self_collision={}; '
                                   'no retry or motion. Check controller configuration, do not repeat this run.'.format(
                                       collision,self_collision))
            self.settled(fresh['joints'])
        check_reference(self.full_arm(),self.reference)
        self.protection_prepared=True
        setup['outcome']='configuration_readbacks_passed'
        self.save()

    def hand_open_check(self):
        # This communication pattern passed on192825. Read the current state
        # needed for motion, without repeating the timed diagnostic experiment.
        hand = self.hand_factory(self, True)
        try:
            if not self.report.get('hand_configuration_checked_this_run'):
                hand.configuration()
                if self.report['configuration'] != self.profile['observed_configuration']:
                    raise RuntimeError('Hand configuration differs from frozen on-device profile')
                self.report['hand_configuration_checked_this_run']=True
            state=hand.preflight(expected=self.start_hand_target)
            self.report['guarded_readonly_transport_preflight_pass']=True;self.save()
            return state
        finally: self.close_hand()

    def close_hand(self):
        if self.io is not None:
            current = self.io; self.io = None
            current.disable_motion()
            try: current.sock.close()
            except Exception as exc:
                self.report.setdefault('cleanup_errors',[]).append(dict(method='hand.close',error=repr(exc)))
                self.save()
                raise

    def preflight(self):
        metadata = collect_sdk_environment(self.robot)
        self.report['sdk_environment'] = metadata; self.save()
        if (metadata.get('metadata_complete') is not True
                or metadata.get('loaded_native',{}).get('sha256') != NATIVE_SHA
                or metadata.get('api_version') != '4.3.1'
                or metadata.get('host_machine') != 'aarch64'
                or metadata.get('controller_run_mode_raw',{}).get('data') != [1]
                or metadata.get('controller_software_info',{}).get('data',[{}])[0].get('product_version') != 'RM65-BI'):
            raise RuntimeError('Live SDK/controller identity or real mode differs from accepted device')
        arm = self.full_arm(); check_reference(arm,self.reference)
        if self.resume_document is not None: check_resumed_reference(arm,self.reference)
        allow_disabled=self.enable_arm_protection and not self.protection_prepared
        low,high = self.safety_reads(allow_disabled=allow_disabled)
        candidate=arm['joints'][:] if self.resume_document is not None else wrist_target(arm)
        if any(min(q-a,b-q)<3 for q,a,b in zip(arm['joints'],low,high)) or any(
                min(q-a,b-q)<3 for q,a,b in zip(candidate,low,high)):
            raise RuntimeError('Current/target violates drive reserve before any setup write')
        self.guard_target=arm['joints'][:];self.guard_pose=arm['pose'][:]
        self.hand_open_check()
        self.settled(arm['joints'])
        if self.enable_arm_protection and not self.protection_prepared:
            self.prepare_arm_protection()
            # No redundant hand experiment here. The mandatory fresh preflight
            # following m opens a new hand socket before the actual Movej.
        fresh = self.full_arm(); check_reference(fresh,self.reference)
        if self.resume_document is not None: check_resumed_reference(fresh,self.reference)
        target = self.reference['joints'][:] if self.resume_document is not None else wrist_target(fresh)
        if any(min(q-a,b-q)<3 for q,a,b in zip(fresh['joints'],low,high)) or any(
                min(q-a,b-q)<3 for q,a,b in zip(target,low,high)):
            raise RuntimeError('Current/target violates drive3-degree reserve')
        self.report['candidate'] = dict(start_joints=fresh['joints'],target_joints=target,
            j6_delta_degrees=0 if self.resume_document is not None else 1,
            speed_percent=0 if self.resume_document is not None else 5,trajectory_connect=0,radius=0,
            full_pick_path_ready=False,automatic_contact_release_verified=False)
        self.report['preflight_pass'] = True; self.save()
        return fresh,target

    def run(self, execute):
        if self.resume_document is not None:
            return self.run_resumed(execute)
        if self.enable_arm_protection and not execute:
            raise RuntimeError('Protection setup is forbidden in read-only mode')
        arm,target = self.preflight()
        if not execute:
            self.report['outcome']='read_only_preflight_complete'; return
        print('本次只转J6 +1°（速度5%），不是观察位到电池位移动。无自动回臂。')
        print('现场退出拖动/示教；确认机械臂活动部件、线缆和手在微转及全部手型扫掠范围无接触，最小净空>=100mm。')
        print('硬物接触立即硬件急停；Ctrl+C尝试一次软件臂停止。程序退出/回读不证明停止或断扭矩释放。')
        print('人工输入时不连续监控；现场人员必须一直观察。急停后不得以断扭矩假设自动张开。')
        answer = self.confirm('现场急停人员就位并重新量净空；输入净空毫米，q退出：').strip().lower()
        if answer == 'q':
            self.report['outcome']='operator_exit_before_motion' if self.report.get('arm_protection_setup',{}).get('writes') else 'operator_exit_no_writes'
            return
        try: clearance = float(answer)
        except ValueError: raise RuntimeError('Need measured whole swept clearance in mm')
        if not math.isfinite(clearance) or clearance < 100:
            raise RuntimeError('Fresh whole-arm/hand swept clearance must be>=100 mm')
        self.report['fresh_whole_sweep_clearance_mm']=clearance; self.save()
        if self.confirm('确认上述完整动作范围、无人同时操控；输入m执行一次腕微转，其他输入退出：').strip().lower()!='m':
            self.report['outcome']='operator_exit_before_motion' if self.report.get('arm_protection_setup',{}).get('writes') else 'operator_exit_no_writes'
            return
        # Human input has no hand TCP open. All checks are fresh afterwards.
        fresh,target = self.preflight()
        self.arm_command_attempted = True
        self.report['arm_command_attempted'] = True; self.save()
        status = self.call('Movej_Cmd',target,SPEED_PERCENT,trajectory_connect=0,r=0,block=False)
        if type(status) is not int or status != 0:
            raise RuntimeError('Movej rejected/uncertain; no retry')
        self.settled(target,travel_start=fresh['joints'])
        self.report['arm_move_feedback_pass']=True; self.save()
        if self.confirm('臂反馈已稳定。现场确认实际停止且手扫掠空；输入h执行已验证手型，其他输入结束：').strip().lower()!='h':
            self.report['outcome']='operator_exit_after_arm_move'; return
        self.settled(target)
        low,high=self.safety_reads()
        if any(min(q-a,b-q)<3 for q,a,b in zip(target,low,high)):
            raise RuntimeError('Drive reserve changed before hand phase')
        final_arm = self.full_arm()
        if final_arm['frame_signatures']!=self.reference['frame_signatures'] or final_arm['limits']!=self.reference['limits']:
            raise RuntimeError('Arm configuration changed after motion')
        self.report['arm_hand_phase_pose']=final_arm['pose'][:];self.save()
        self.guard_target=target[:];self.guard_pose=final_arm['pose'][:]
        self.hand_phase(target)

    def run_resumed(self, execute):
        if not execute or self.enable_arm_protection:
            raise RuntimeError('Resume is hand continuation only; no protection setters or arm motion')
        self.protection_prepared=True  # Fresh raw8/true is required; no setter path.
        arm,target=self.preflight()
        # Check the pinned post-wrist point rather than accepting any nearby
        # reference position, and never silently align to it.
        if any(abs(q-t)>.08 for q,t in zip(arm['joints'],target)):
            raise RuntimeError('Arm moved since audited capture; no automatic reposition')
        print('续接200240：机械臂不移动，不重做预抓；继续手型第2/3段，然后正常张开。')
        print('空手、现场守急停；接触硬物立即硬件停止，再现场安全释放。输入期间不连续监控。')
        answer=self.confirm('现场确认完整手型扫掠无接触；输入当前最小净空毫米，q退出：').strip().lower()
        if answer=='q':
            self.report['outcome']='operator_exit_no_writes';return
        try: clearance=float(answer)
        except ValueError: raise RuntimeError('Need measured hand swept clearance in mm')
        if not math.isfinite(clearance) or clearance<100:
            raise RuntimeError('Hand swept clearance must be>=100 mm')
        self.report['fresh_whole_sweep_clearance_mm']=clearance;self.save()
        if self.confirm('现场确认臂实际停稳、空手无遮挡且无人同时控制；h继续手型，其他输入退出：').strip().lower()!='h':
            self.report['outcome']='operator_exit_no_writes';return
        fresh,target=self.preflight()
        if any(abs(q-t)>.08 for q,t in zip(fresh['joints'],target)):
            raise RuntimeError('Arm changed during confirmation; no hand writes')
        self.settled(target)
        self.report.update(arm_move_reused_from_capture='arm_hand_sequence_20261009_200240_418539',
                           prior_arm_move_feedback_pass=True,arm_command_attempted=False,
                           arm_hand_phase_pose=fresh['pose'][:])
        self.guard_target=target[:];self.guard_pose=fresh['pose'][:]
        self.save()
        self.hand_phase(target,start_index=1)

    def hand_phase(self, target, start_index=0):
        hand = self.hand_factory(self, True)
        try:
            hand.configuration()
            if self.report['configuration'] != self.profile['observed_configuration']:
                raise RuntimeError('Hand configuration changed after wrist motion')
            hand.preflight(expected=self.start_hand_target)
            previous = self.start_hand_target[:]
            self.report['required_speeds'] = [65535]*6
            if start_index:
                hand.wait(previous,(),previous)
                hand.preflight(expected=previous)
                self.report['stage1_source']='prior_prepare_target_reconfirmed_without_write'
            self.io.enable_motion()
            self.report['completed_adapted_stages']=start_index;self.save()
            for i in range(start_index,len(ADAPTED_PLAN)):
                step=ADAPTED_PLAN[i]
                while True:
                    command = self.confirm('n=手型第{}/3段 {}；r=分阶段张开结束；q=保持退出：'.format(i+1,step['phase'])).strip().lower()
                    if command in ('n','r','q'): break
                    print('请输入n/r/q。接触立即硬件急停，勿等待输入。')
                if command == 'q':
                    self.report['outcome']='operator_exit_hand_pose_held'; return
                if command == 'r': break
                previous = hand.write_pose(step,previous)
                hand.observe_pose(step['phase'],previous)
                self.report['completed_adapted_stages']=i+1;self.save()
                print('REACHED',step['phase'],previous,flush=True)
            else:
                if self.confirm('手型到位。r=分阶段张开结束；其他输入保持退出：').strip().lower()!='r':
                    self.report['outcome']='operator_exit_hand_pose_held'; return
            previous = hand.release_stages(previous)
            hand.observe_pose('final_open',previous)
            self.report['final_hand'] = hand.preflight(expected=self.profile['poses']['open'])
        except (Exception,KeyboardInterrupt):
            self.io.disable_motion()
            self.fault()
            hand.fault_details()
            raise
        finally: self.close_hand()
        self.settled(target)
        complete=self.report.get('completed_adapted_stages')==3
        completed_outcome='completed_resumed_hand_open' if start_index else 'completed_wrist_hand_open'
        self.report.update(outcome=completed_outcome if complete else 'completed_early_open',
                           verified_hand_sequence_completed=complete,hand_open_feedback_pass=True,
                           full_pick_path_ready=False,physical_stop_verified=False,torque_release_verified=False)

    def fault(self):
        if self.io is not None: self.io.disable_motion()
        if self.arm_command_attempted and not self.arm_stop_attempted:
            self.arm_stop_attempted = True
            try:
                self.report['arm_stop_attempt'] = dict(result=self.call('Move_Stop_Cmd',block=False),
                    physical_stop_verified=False,note='Arm only; does not stop or release the hand')
            except (Exception,KeyboardInterrupt) as exc:
                self.report['arm_stop_attempt'] = dict(error=repr(exc),physical_stop_verified=False)
        try: self.arm_state()
        except (Exception,KeyboardInterrupt) as exc: self.report['post_fault_arm_read_error']=repr(exc)
        self.save()


class ArmGuardHand(FeedbackHand):
    def __init__(self, sequence, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sequence = sequence
        self.expected_arm = sequence.guard_target[:]
        self.expected_pose = sequence.guard_pose[:]

    def guard(self):
        joints,pose = self.sequence.arm_state()
        if (any(abs(a-b)>.08 for a,b in zip(joints,self.expected_arm))
                or math.dist(pose[:3],self.expected_pose[:3])>.0005
                or any(abs(math.atan2(math.sin(a-b),math.cos(a-b)))>.005
                       for a,b in zip(pose[3:],self.expected_pose[3:]))):
            self.io.disable_motion()
            raise RuntimeError('Arm drift during hand phase; no further hand writes')

    def snapshot(self):
        self.guard()
        state = super().snapshot()
        self.guard()
        return state

    def preflight(self, *args, **kwargs):
        state = super().preflight(*args, **kwargs)
        self.guard()
        return state

    def write_pose(self, step, previous):
        # Recheck measured configuration after each human wait, at the command
        # boundary. This is not continuous configuration monitoring.
        self.configuration()
        if self.report['configuration'] != self.sequence.profile['observed_configuration']:
            self.io.disable_motion()
            raise RuntimeError('Hand configuration changed before pose write')
        self.guard()
        return super().write_pose(step, previous)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--resume-report',type=Path,
                        help='Audited200240 result.json only: continue hand without moving wrist again')
    parser.add_argument('--enable-arm-protection',action='store_true',
                        help='Opt-in stage8 and self-collisionTrue only; onsite e confirmation required')
    parser.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--references',type=Path,default=Path(__file__).with_name('grasp_references_accepted_20261009.json'))
    parser.add_argument('--profile',type=Path,default=Path(__file__).with_name('battery_hand_verified_profile.json'))
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args=parser.parse_args(argv)
    if not args.confirm_empty_clear_exclusive or not args.confirm_onsite_stop:
        parser.error('Requires empty hand, exclusive teaching/motion control and onsite hardware stop; no connection opened')
    if args.enable_arm_protection and not args.execute:
        parser.error('Protection setup requires --execute; read-only mode never changes parameters')
    if args.resume_report is not None and (not args.execute or args.enable_arm_protection):
        parser.error('Resume requires --execute and forbids protection setting; no connection opened')
    profile=json.loads(args.profile.read_text(encoding='utf-8'))
    if hashlib.sha256(args.references.read_bytes()).hexdigest()!=REFERENCE_SHA:
        parser.error('Reference file differs from audited180315 capture; no connection opened')
    references=json.loads(args.references.read_text(encoding='utf-8'))
    if hashlib.sha256(args.profile.read_bytes()).hexdigest()!=references.get('profile_sha256'):
        parser.error('Hand profile differs from audited capture; no connection opened')
    reference=accepted_reference(references,profile)
    resume_document=None
    if args.resume_report is not None:
        if not args.resume_report.is_file() or hashlib.sha256(args.resume_report.read_bytes()).hexdigest()!=RESUME_SHA:
            parser.error('Resume report differs from audited200240; no connection opened')
        resume_document=json.loads(args.resume_report.read_text(encoding='utf-8'))
        if (resume_document.get('reference_sha256')!=REFERENCE_SHA
                or resume_document.get('profile_sha256')!=references.get('profile_sha256')):
            parser.error('Resume profile/reference differs from audited baseline; no connection opened')
        reference=accepted_resume(resume_document,reference,profile)
    folder=ROOT/'captures'/('arm_hand_sequence_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    report=dict(schema_version=1,outcome='starting',preflight_pass=False,moves=[],
        execution_scope=('empty_hand_resume_no_arm_motion' if resume_document is not None
                         else 'empty_hand_J6_plus1_degree_only' if args.execute else 'read_only'),
        resume_report_sha256=RESUME_SHA if resume_document is not None else None,
        full_pick_path_ready=False,physical_stop_verified=False,torque_release_verified=False,
        automatic_contact_release_verified=False,exceptions=[],cleanup_errors=[],
        verified_hand_sequence_completed=False,
        arm_protection_setup_requested=args.enable_arm_protection,
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        sdk_wire_capture_available=False,arm_log_scope='motion/state: decoded SDK; protections: separate raw TCP log',
        arm_protection_wire_capture_available=True,arm_protection_read_source='fresh_socket_raw_controller_JSON_not_SDK_cache',
        reference_sha256=hashlib.sha256(args.references.read_bytes()).hexdigest(),
        profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
        source_sha256={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in SOURCE_NAMES})
    def save():
        temporary=folder/'result.tmp'
        temporary.write_text(json.dumps(_json_safe(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        temporary.replace(folder/'result.json')
    save();robot=None;sequence=None
    try:
        import fcntl
        with (folder/'wire.jsonl').open('w',encoding='utf-8') as wire, \
             (folder/'arm_protection_wire.jsonl').open('w',encoding='utf-8') as protection_wire, \
             (folder/'samples.jsonl').open('w',encoding='utf-8') as samples, \
             (ROOT/'captures/hand_stage.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            robot=connect(args.sdk_dir)
            def protection_log(row):
                protection_wire.write(json.dumps(row)+'\n');protection_wire.flush()
            protection=ProtectionClient(protection_log)
            def factory(seq,guarded):
                if seq.io is not None: raise RuntimeError('Hand connection already open')
                def log(row):wire.write(json.dumps(row)+'\n');wire.flush()
                seq.io=SequenceTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),log)
                cls=ArmGuardHand if guarded else FeedbackHand
                positional=(seq,) if guarded else ()
                hand=cls(*positional,seq.io,report,save,samples)
                if guarded: seq.io.before_write=hand.guard
                return hand
            def confirm(text):
                answer=input(text)
                sequence.event('operator_input',prompt=text,value=answer)
                return answer
            sequence=Sequence(robot,reference,profile,report,save,samples,factory,confirm=confirm,
                              enable_arm_protection=args.enable_arm_protection,protection_client=protection,
                              resume_document=resume_document)
            try: sequence.run(args.execute)
            except (Exception,KeyboardInterrupt):
                sequence.fault()
                raise
            finally: sequence.close_hand()
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report['outcome']='aborted_no_further_motion'
        if report.get('fresh_protection_values'):
            print('控制器原始保护回复：',report['fresh_protection_values'],flush=True)
        if 'next_action' not in report:
            report['next_action']='Use the saved complete error and original replies; do not retry settings or motion blindly.'
        print('ABORT:',exc,'接触或继续运动立即现场硬件急停；无自动张开/回臂。',flush=True)
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try: getattr(robot,name)()
                except Exception as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
        if report['cleanup_errors']: report['outcome']='failed_cleanup'
        save();print('SAVED:',folder,'OUTCOME:',report['outcome'],flush=True)
        try:
            bundle=folder.with_suffix('.zip')
            with zipfile.ZipFile(bundle,'x',zipfile.ZIP_DEFLATED) as archive:
                for item in sorted(folder.iterdir()):
                    if item.is_file(): archive.write(item,folder.name+'/'+item.name)
            print('完整结果已自动打包：',bundle,flush=True)
        except Exception as exc:
            print('结果目录已保存，自动打包失败：',repr(exc),flush=True)
    return 0 if report['outcome'] in ('read_only_preflight_complete','completed_wrist_hand_open',
                                    'completed_resumed_hand_open','completed_early_open') else 1


if __name__=='__main__':raise SystemExit(main())

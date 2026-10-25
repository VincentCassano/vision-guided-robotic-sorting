"""Actual controller queue: three same-type MoveJ-P commands, connect1/1/0, r0.

SDK4.3.1 is pinned. Nonzero blend and mixed MoveL/MoveJ-P are not enabled.
No software endpoint pause between queued points; r0 is not a nonzero-speed
corner guarantee. Intermediate notifications are not stationary measurements.
"""
import copy
import math
from high_descent_prepare_worker import same_pose
from battery_grasp.real_grasp import orientation_angle_rad
from visual_run_configuration_v3 import require_current_configuration


def move_connected_transfer(seq,arm,rows,hand):
    if (len(rows)!=3 or [r['phase'] for r in rows]!=[
            'school_high_transit','school_restart_transit','school_black_drop']
            or not seq.route_allowed or seq.authority._terminal_latched):
        raise RuntimeError('Exact healthy three-point black transfer scope required')
    seq.plan=None;seq.close_hand()
    with (seq.folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as cwire:
        require_current_configuration(seq.report['observed_run_configuration'],cwire)
    fresh=seq.full_arm()
    if not same_pose(fresh,arm) or fresh['limits']!=arm['limits']:
        raise RuntimeError('Connected route start changed')
    if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:
        raise RuntimeError('Connected route protections changed')
    replacement=seq.hand_factory(seq,True);hand.io=replacement.io
    hand._critical_configuration();hand.monitor_loaded(seq.load_targets,seq.load_anchor)
    after=seq.full_arm()
    if not same_pose(after,fresh) or after['limits']!=fresh['limits']:
        raise RuntimeError('Arm moved during connected route preflight')
    commands=[dict(method='Movej_P_Cmd',target=r['pose'][:],speed=10,
        trajectory_connect=flag,r=0,phase=r['phase']) for r,flag in zip(rows,[1,1,0])]
    target=commands[-1]['target']
    context=dict(kind='connected_school_transfer',method='Movej_P_Cmd',commands=commands,
        next_submit=0,acknowledgements=[],events=[],acknowledged=False,event_seen=False)
    seq.notification=context
    seq.bin_motion=dict(method='Movej_P_Cmd',target=target[:],start=copy.deepcopy(fresh),connected=True)
    seq.report.update(phase='connected_school_transfer',execution_ready=True,
        current_school_motion=copy.deepcopy(seq.bin_motion),connected_transfer=context,
        intermediate_endpoint_settling_required=False,software_wait_between_transfer_commands=False,
        blend_radius=0,nonzero_corner_velocity_guaranteed=False,
        controller_planned_joint_path=True,straight_cartesian_path_claimed=False)
    began=seq.clock();seq.save()
    for command in commands:
        # No hand/configuration query, sleep or per-waypoint quiet gate here.
        # The first two commands are queued; the last connect0 starts the group.
        if (seq.folder/'abort.json').exists():raise KeyboardInterrupt('Parent abort before queue submission')
        seq.arm_command_attempted=True;seq.report['arm_command_attempted']=True
        seq.report['arm_actuator_writes']+=1
        seq.save()
        rc=seq.call('Movej_P_Cmd',command['target'],command['speed'],
            trajectory_connect=command['trajectory_connect'],r=0,block=False)
        context['acknowledgements'].append(dict(index=len(context['acknowledgements']),
            rc=rc,elapsed_seconds=seq.clock()-began))
        seq.save()
        if type(rc) is not int or rc!=0:
            raise RuntimeError('Connected queue refused/uncertain; stop, no retry/flush command')
    context['acknowledged']=True;seq.save()
    previous=None;quiet_since=None;groups=0;last_hand=began;endpoint=None
    while seq.clock()-began<100:
        q,p=seq.arm_state()
        if seq.clock()-last_hand>=.5:
            hand.monitor_loaded(seq.load_targets,seq.load_anchor);last_hand=seq.clock()
        reached=(math.dist(p[:3],target[:3])<=.0005
            and orientation_angle_rad(p[3:],target[3:])<=.005)
        quiet=(previous is not None and max(abs(a-b) for a,b in zip(q,previous[0]))<=.02
            and math.dist(p[:3],previous[1][:3])<=.0002
            and orientation_angle_rad(p[3:],previous[1][3:])<=.002)
        if reached and quiet:
            if quiet_since is None:quiet_since=seq.clock()
            groups+=1
            if groups>=3 and seq.clock()-quiet_since>=1:
                endpoint=seq.full_arm()
                if (endpoint['frame_signatures']!=fresh['frame_signatures']
                        or endpoint['limits']!=fresh['limits']
                        or math.dist(endpoint['pose'][:3],target[:3])>.0005
                        or orientation_angle_rad(endpoint['pose'][3:],target[3:])>.005
                        or any(min(v-a,b-v)<3 for v,a,b in zip(endpoint['joints'],
                            seq.drive_limits['minimum'],seq.drive_limits['maximum']))):
                    raise RuntimeError('Connected final endpoint changed')
                hand.monitor_loaded(seq.load_targets,seq.load_anchor)
                if context['event_seen'] is not True:
                    raise RuntimeError('Connected route missing ordered1/1/0 completion events')
                break
        else:quiet_since=None;groups=0
        previous=q,p;seq.pause(.1)
    if endpoint is None:raise TimeoutError('Connected transfer final endpoint timeout')
    seq.report.setdefault('arm_segments',[]).append(dict(phase='connected_school_transfer',
        method='Movej_P_Cmd',queued_commands=copy.deepcopy(commands),speed_percent=10,
        target_sdk_pose_m_rad=target[:],endpoint=copy.deepcopy(endpoint),
        elapsed_seconds=seq.clock()-began,notification=copy.deepcopy(context),
        intermediate_stationary_endpoints_claimed=False,shell_retention_sensor_verified=False,
        continuous_swept_geometry_verified=False))
    seq.guard_target,seq.guard_pose=endpoint['joints'][:],endpoint['pose'][:]
    seq.bin_motion=None;seq.report['execution_ready']=False;seq.save()
    return endpoint

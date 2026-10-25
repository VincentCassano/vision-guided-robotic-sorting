"""Pure numerical model for ONE supervised high-clearance World-Z trial.

Not an old-path executor or a battery approach planner. No SDK/no network.
The only distance is50mm; current XY/orientation remain fixed. Sampled IK is
feasibility evidence, not a continuous controller or physical collision proof.
"""
import math
from .real_grasp import number, rotation
from .pregrasp_sweep import frames

DISTANCE_MM = 50.0
PREPARE = [8000,8000,8000,8000,65535,65535]
OPEN = [0,0,0,0,0,728]


def matrix_angle(a,b):
    cosine=(sum(a[i][j]*b[i][j] for i in range(3) for j in range(3))-1)/2
    return math.degrees(math.acos(max(-1,min(1,cosine))))


def solve(a,b):
    """Pivoted six-dimensional linear solve; refuse a singular Jacobian."""
    m=[list(row)+[v] for row,v in zip(a,b)]
    for k in range(6):
        p=max(range(k,6),key=lambda i:abs(m[i][k]))
        if abs(m[p][k])<1e-8:
            raise ValueError('Local IK singular/ill-conditioned; no motion')
        m[k],m[p]=m[p],m[k]
        divisor=m[k][k]
        m[k]=[v/divisor for v in m[k]]
        for i in range(6):
            if i!=k:
                factor=m[i][k]
                m[i]=[v-factor*w for v,w in zip(m[i],m[k])]
    return [row[-1] for row in m]


def pose_error(q,target,orientation):
    t=frames(q)[-1]
    r=[row[:3] for row in t[:3]]
    e=[[sum(orientation[i][k]*r[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    return [t[i][3]-target[i] for i in range(3)]+[
        50*(e[2][1]-e[1][2]),50*(e[0][2]-e[2][0]),50*(e[1][0]-e[0][1])]


def inverse_near(seed,target,orientation):
    q=list(seed)
    for _ in range(60):
        residual=pose_error(q,target,orientation)
        t=frames(q)[-1]
        position_error=math.sqrt(sum((t[i][3]-target[i])**2 for i in range(3)))
        angle=matrix_angle([r[:3] for r in t[:3]],orientation)
        if position_error<.01 and angle<.01:
            return q,position_error,angle
        jacobian=[[0.]*6 for _ in range(6)]
        for j in range(6):
            trial=q[:];trial[j]+=.001
            value=pose_error(trial,target,orientation)
            for i in range(6):jacobian[i][j]=(value[i]-residual[i])/.001
        delta=solve(jacobian,[-v for v in residual])
        scale=min(1.,2/max(max(abs(v) for v in delta),1e-12))
        q=[a+scale*b for a,b in zip(q,delta)]
    raise ValueError('Local50mm IK did not converge; no motion')


def reserve(q,limits):
    return min(min(v-a,b-v) for v,a,b in zip(q,limits['minimum'],limits['maximum']))


def build_trial(arm,drive_limits,line_speed_m_s,line_acc_m_s2):
    speed=number(line_speed_m_s,'live maximum line speed')
    acceleration=number(line_acc_m_s2,'live maximum line acceleration')
    if speed<=0 or acceleration<=0 or acceleration/speed<3:
        raise ValueError('Invalid live line parameters/acceleration ratio; no configuration change')
    # v is a command-specific percentage, not a persistent speed setter.
    percent=min(5,int(math.floor(10/(speed*1000)*100+1e-8)))
    if percent<1:
        raise ValueError('Even1percent exceeds10mm/s; no speed configuration write')
    nominal_speed=speed*1000*percent/100
    if DISTANCE_MM/nominal_speed>30:
        raise ValueError('Live speed would make50mm trial exceed30s; no slow test or speed setter')
    if (arm['frame_names']!={'work':'World','tool':'Arm_Tip'}
            or any(abs(v)>1e-9 for row in arm['frame_signatures'].values()
                   for group in ('position','euler') for v in row[group].values())):
        raise ValueError('Zero World/Arm_Tip required')
    if any(v<=0 for v in arm['limits']['speed']+arm['limits']['acceleration']):
        raise ValueError('Invalid ordinary joint dynamics')
    start=[v*1000 for v in arm['pose'][:3]]
    orientation=rotation(arm['pose'][3:])
    t=frames(arm['joints'])[-1]
    witness_position=math.dist([t[i][3] for i in range(3)],start)
    witness_rotation=matrix_angle([r[:3] for r in t[:3]],orientation)
    if witness_position>1 or witness_rotation>.5:
        raise ValueError('Actual-DH FK does not match fresh controller start; no fitted correction')
    q=arm['joints'][:]
    rows=[]
    for i in range(11):
        target=[start[0],start[1],start[2]-5*i]
        q,err,angle=inverse_near(q,target,orientation)
        if reserve(q,arm['limits'])<4 or reserve(q,drive_limits)<4:
            raise ValueError('Local IK has less than4degree sampled margin (3degree live minimum plus1degree tracking box)')
        if max(abs(a-b) for a,b in zip(q,arm['joints']))>20:
            raise ValueError('Local IK requires a large joint change; no trial')
        rows.append(dict(index=i,joint_degrees=q[:],target_world_mm=target,
            position_residual_mm=err,rotation_residual_degrees=angle,
            ordinary_reserve_degrees=reserve(q,arm['limits']),drive_reserve_degrees=reserve(q,drive_limits)))
    limits=dict(minimum=[min([arm['joints'][i]]+[r['joint_degrees'][i] for r in rows])-1 for i in range(6)],
                maximum=[max([arm['joints'][i]]+[r['joint_degrees'][i] for r in rows])+1 for i in range(6)])
    actual_acceleration=acceleration*1000*percent/100
    timeout=min(45.,DISTANCE_MM/nominal_speed+2*nominal_speed/actual_acceleration+10)
    return dict(schema_version=1,scope='qualified_onsite_empty_hand_high50mm_trial',
        distance_mm=50.,start=arm,drive_limits=drive_limits,
        target_sdk_pose_m_rad=arm['pose'][:2]+[arm['pose'][2]-.05]+arm['pose'][3:],
        target_world_mm=rows[-1]['target_world_mm'],fixed_xy=True,fixed_wrist=True,
        arm_command_count=1,hand_target_after_settled=PREPARE[:],
        speed_percent=percent,nominal_speed_mm_s=nominal_speed,
        live_maximum_line_speed_m_s=speed,live_maximum_line_acceleration_m_s2=acceleration,
        expected_translation_seconds=DISTANCE_MM/nominal_speed,arm_timeout_seconds=timeout,
        trajectory_connect=0,radius=0,block=False,
        local_ik_rows=rows,live_joint_guard_limits=limits,
        start_FK_error_mm=witness_position,start_FK_error_degrees=witness_rotation,
        numerical_feasibility_pass=True,continuous_controller_reserve_verified=False,
        full_scene_geometry_certified=False,controller_motion_live_verified=False,
        full_pregrasp_execution_ready=False,automatically_reposition_to_old_start=False,
        automatic_grip=False,automatic_lift=False,automatic_open=False,automatic_return=False)


def check_live(arm,plan):
    if arm['frame_signatures']!=plan['start']['frame_signatures'] or arm['limits']!=plan['start']['limits']:
        raise ValueError('Frame or limits changed during trial')
    if reserve(arm['joints'],arm['limits'])<3 or reserve(arm['joints'],plan['drive_limits'])<3:
        raise ValueError('Actual joint reserve below3degrees')
    if any(not a<=q<=b for q,a,b in zip(arm['joints'],
            plan['live_joint_guard_limits']['minimum'],plan['live_joint_guard_limits']['maximum'])):
        raise ValueError('Actual IK branch/tracking left bounded local candidate')
    p=arm['pose']
    start=plan['start']['pose']
    if math.dist(p[:2],start[:2])*1000>1:
        raise ValueError('Actual XY deviated by more than1mm')
    if any(abs(math.atan2(math.sin(a-b),math.cos(a-b)))>.005 for a,b in zip(p[3:],start[3:])):
        raise ValueError('Actual wrist changed beyond0.005rad')
    if not start[2]-.051<=p[2]<=start[2]+.001:
        raise ValueError('Actual Z left bounded50mm descent')
    return True

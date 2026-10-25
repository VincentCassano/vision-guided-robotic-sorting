"""Signed100mm Cartesian rehearsal from THIS SESSION's taught PREPARE pose."""
import math
from .real_grasp import number,rotation,orientation_angle_rad
from .pregrasp_sweep import frames
from .high_descent_trial import inverse_near,matrix_angle,reserve


def segment(arm,drive,delta_mm,speed,acceleration):
    if abs(delta_mm)>101 or abs(delta_mm)<98:
        raise ValueError('Only the100mm lift or return segment is permitted')
    speed=number(speed,'live speed');acceleration=number(acceleration,'live acceleration')
    if speed<=0 or acceleration<=0:raise ValueError('Invalid live tip parameters')
    if arm['frame_names']!={'work':'World','tool':'Arm_Tip'} or any(
            abs(v)>1e-9 for row in arm['frame_signatures'].values()
            for field in ('position','euler') for v in row[field].values()):
        raise ValueError('Zero World/Arm_Tip required')
    if any(v<=0 for v in arm['limits']['speed']+arm['limits']['acceleration']):
        raise ValueError('Invalid ordinary joint dynamics')
    pct=min(5,int(math.floor(1000/(speed*1000)+1e-8)))
    nominal=speed*1000*pct/100
    if pct<1 or nominal<=0 or abs(delta_mm)/nominal>40:
        raise ValueError('Live speed does not support bounded100mm rehearsal')
    r=rotation(arm['pose'][3:]);t=frames(arm['joints'])[-1]
    xyz=[v*1000 for v in arm['pose'][:3]]
    witness=math.dist([t[i][3] for i in range(3)],xyz)
    angle=matrix_angle([row[:3] for row in t[:3]],r)
    if witness>1 or angle>.5:raise ValueError('Actual-DH start FK mismatch')
    q=arm['joints'][:];rows=[]
    count=int(math.ceil(abs(delta_mm)/5))
    previous=q[:]
    for i in range(count+1):
        target=[xyz[0],xyz[1],xyz[2]+delta_mm*i/count]
        q,error,orientation_error=inverse_near(q,target,r)
        if min(reserve(q,arm['limits']),reserve(q,drive))<4:
            raise ValueError('Sampled joint reserve below4degrees')
        if max(abs(a-b) for a,b in zip(q,previous))>3:
            raise ValueError('Local IK branch jump exceeds3degrees per5mm')
        if max(abs(a-b) for a,b in zip(q,arm['joints']))>60:
            raise ValueError('100mm candidate exceeds bounded60degree joint excursion')
        rows.append(dict(joint_degrees=q[:],target_world_mm=target,
            position_residual_mm=error,rotation_residual_degrees=orientation_error,
            ordinary_reserve_degrees=reserve(q,arm['limits']),drive_reserve_degrees=reserve(q,drive)))
        previous=q[:]
    tracking=dict(minimum=[min([arm['joints'][j]]+[row['joint_degrees'][j] for row in rows])-1 for j in range(6)],
        maximum=[max([arm['joints'][j]]+[row['joint_degrees'][j] for row in rows])+1 for j in range(6)])
    timeout=abs(delta_mm)/nominal+2*nominal/(acceleration*1000*pct/100)+10
    if timeout>60:raise ValueError('Live acceleration/speed exceeds bounded rehearsal deadline')
    target=arm['pose'][:];target[2]+=delta_mm/1000
    return dict(start=arm,drive_limits=drive,target_sdk_pose_m_rad=target,
        target_world_mm=[v*1000 for v in target[:3]],distance_mm=delta_mm,
        speed_percent=pct,nominal_speed_mm_s=nominal,arm_timeout_seconds=timeout,
        local_ik_rows=rows,live_joint_guard_limits=tracking,start_FK_error_mm=witness,
        start_FK_error_degrees=angle,full_scene_geometry_certified=False,
        full_pregrasp_execution_ready=False,vision_target_binding_verified=False)


def check_segment(arm,plan):
    if arm['frame_signatures']!=plan['start']['frame_signatures'] or arm['limits']!=plan['start']['limits']:
        raise ValueError('Frames or limits changed')
    if min(reserve(arm['joints'],arm['limits']),reserve(arm['joints'],plan['drive_limits']))<3:
        raise ValueError('Live joint reserve below3degrees')
    if any(not lo<=q<=hi for q,lo,hi in zip(arm['joints'],
            plan['live_joint_guard_limits']['minimum'],plan['live_joint_guard_limits']['maximum'])):
        raise ValueError('Actual joint branch left computed tracking box')
    start=plan['start']['pose'];p=arm['pose'];end=plan['target_sdk_pose_m_rad']
    if math.dist(p[:2],start[:2])*1000>1:raise ValueError('XY drift exceeds1mm')
    if orientation_angle_rad(p[3:],start[3:])>.005:
        raise ValueError('Physical wrist rotation exceeded0.005rad')
    if not min(start[2],end[2])-.001<=p[2]<=max(start[2],end[2])+.001:
        raise ValueError('Z left the signed100mm segment')


def teach_permission(text):
    if text.strip().lower()=='q':return dict(value='q')
    gap=number(float(text),'minimum current hand clearance')
    if gap<14:raise ValueError('Minimum hand clearance must be at least14mm, including battery and board')
    return dict(value='t',hand_minimum_clearance_mm=gap,
        battery_in_grasp_space_no_contact=True,teaching_exited_and_hands_removed=True)

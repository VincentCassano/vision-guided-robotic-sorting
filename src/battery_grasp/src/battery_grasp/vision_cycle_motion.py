"""Live-input motion calculation; called onsite, no independent test entry."""
import math
import time
from .high_descent_trial import inverse_near,matrix_angle,reserve
from .pregrasp_sweep import frames
from .real_grasp import rotation


def build_segment(arm,drive,target_z_m,speed,acceleration,record=None):
    distance=(target_z_m-arm['pose'][2])*1000
    if not 30<=abs(distance)<=120:raise ValueError('This pilot requires a30..120mm taught vertical corridor')
    if not all(math.isfinite(v) and v>0 for v in (speed,acceleration)):
        raise ValueError('Invalid live motion parameters')
    if arm['frame_names']!={'work':'World','tool':'Arm_Tip'}:raise ValueError('World/Arm_Tip required')
    if any(abs(v)>1e-9 for row in arm['frame_signatures'].values()
            for field in ('position','euler') for v in row[field].values()):raise ValueError('Zero frames required')
    if any(v<=0 for v in arm['limits']['speed']+arm['limits']['acceleration']):raise ValueError('Invalid joint dynamics')
    # Near-object command <=5mm/s; no persistent speed/protection reduction.
    percent=min(5,int(500/(speed*1000)))
    nominal=speed*1000*percent/100
    if percent<1 or nominal<=0 or abs(distance)/nominal>60:raise ValueError('Live parameters cannot support bounded near-object speed')
    xyz=[v*1000 for v in arm['pose'][:3]];r=rotation(arm['pose'][3:]);t=frames(arm['joints'])[-1]
    if math.dist([t[i][3] for i in range(3)],xyz)>1 or matrix_angle([v[:3] for v in t[:3]],r)>.5:
        raise ValueError('Actual-DH FK does not agree with fresh controller pose')
    n=int(math.ceil(abs(distance)/5));rows=[];attempts=0;subdivisions=0;began=time.monotonic()
    def emit(row):
        if record is not None:record(row)
    def candidate(q0,f0,f1,depth):
        nonlocal attempts,subdivisions
        if attempts>=4096 or time.monotonic()-began>45:
            raise ValueError('Live IK calculation budget exceeded; no actuator command')
        attempts+=1;point=xyz[:2]+[xyz[2]+distance*f1]
        entry=dict(event='runtime_ik_candidate',attempt=attempts,from_fraction=f0,to_fraction=f1,
            interval_mm=abs(distance*(f1-f0)),depth=depth,seed_joint_degrees=q0[:],target_world_mm=point)
        q=None;reason=None
        try:
            q,err,angle=inverse_near(q0,point,r)
        except ValueError as exc:
            reason='solver_failed';entry['solver_error']=repr(exc)
        if q is not None:
            delta=[b-a for a,b in zip(q0,q)]
            excursion=[b-a for a,b in zip(arm['joints'],q)]
            ordinary=reserve(q,arm['limits']);drive_reserve=reserve(q,drive)
            entry.update(joint_degrees=q[:],joint_delta_degrees=delta,
                total_excursion_degrees=excursion,ordinary_reserve_degrees=ordinary,
                drive_reserve_degrees=drive_reserve,position_residual_mm=err,rotation_residual_degrees=angle)
            if not all(math.isfinite(v) for v in q+[err,angle]):
                reason='nonfinite_solution'
            elif max(abs(v) for v in delta)>3:reason='adjacent_joint_delta_over_3degrees'
            elif max(abs(v) for v in excursion)>60:reason='total_joint_excursion_over_60degrees'
            elif min(ordinary,drive_reserve)<4:reason='sampled_joint_reserve_below_4degrees'
        if reason is None:
            entry['decision']='accepted';emit(entry)
            rows.append(dict(joint_degrees=q[:],target_world_mm=point,
                position_residual_mm=err,rotation_residual_degrees=angle,
                ordinary_reserve_degrees=ordinary,drive_reserve_degrees=drive_reserve))
            return q
        # Refine numerical witnesses, not physical commands. The 3/60/4-degree
        # bounds remain unchanged; no clipping, angle wrapping or write retry.
        can_split=(f1>f0 and abs(distance*(f1-f0))>.05 and depth<12 and reason!='nonfinite_solution')
        entry.update(reason=reason,decision='subdivide' if can_split else 'reject');emit(entry)
        if not can_split:
            raise ValueError('Live IK rejected: '+reason+'; candidate details retained in samples.jsonl')
        subdivisions+=1;middle=(f0+f1)/2
        qm=candidate(q0,f0,middle,depth+1)
        return candidate(qm,middle,f1,depth+1)
    q=candidate(arm['joints'][:],0.,0.,0)
    for i in range(1,n+1):q=candidate(q,(i-1)/n,i/n,0)
    limits=dict(minimum=[min([arm['joints'][j]]+[a['joint_degrees'][j] for a in rows])-1 for j in range(6)],
        maximum=[max([arm['joints'][j]]+[a['joint_degrees'][j] for a in rows])+1 for j in range(6)])
    pose=arm['pose'][:];pose[2]=target_z_m
    return dict(start=arm,drive_limits=drive,target_sdk_pose_m_rad=pose,target_world_mm=[v*1000 for v in pose[:3]],
        distance_mm=distance,speed_percent=percent,nominal_speed_mm_s=nominal,
        arm_timeout_seconds=abs(distance)/nominal+2*nominal/(acceleration*1000*percent/100)+15,
        local_ik_rows=rows,live_joint_guard_limits=limits,
        numerical_attempts=attempts,numerical_subdivisions=subdivisions,
        sampling_policy='adaptive_max5mm_and_max3degrees_unchanged_bounds',
        collision_certificate=False)

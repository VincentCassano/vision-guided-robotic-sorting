"""Bounded straight-line calculation from freshly read onsite arm parameters.

No SDK or network operations occur here. Numerical witnesses do not certify
the installed hand, scene, stopping performance or controller interpolation.
"""
import copy
import math
import time
from .high_descent_trial import inverse_near, matrix_angle, reserve
from .pregrasp_sweep import frames
from .real_grasp import number, orientation_angle_rad, rotation


def _six(values, label):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError(label + ': six finite values required')
    return [number(value, label) for value in values]


def _limits(limits, label):
    lower = _six(limits['minimum'], label + ' minimum')
    upper = _six(limits['maximum'], label + ' maximum')
    if any(a >= b for a, b in zip(lower, upper)):
        raise ValueError(label + ': ordered limits required')


def build_follow_segment(arm, drive, target_pose, speed, acceleration, record=None,
                         vertical_limit_mm=120):
    """One same-Z lateral <=120 mm, or an explicitly bounded vertical MoveL.

    XYZ are requested in metres. RPY always come from the fresh arm readback;
    this capability cannot rotate the wrist or command a diagonal descent.
    Vertical distance remains 30..120 mm unless the caller explicitly selects
    the separate 160 mm bound; no other motion or tracking bound changes.
    """
    if type(vertical_limit_mm) is not int or vertical_limit_mm not in (120, 160):
        raise ValueError('Vertical limit must explicitly be120 or160mm')
    start_pose = _six(arm['pose'], 'fresh pose')
    initial_joints = _six(arm['joints'], 'fresh joints')
    requested = _six(target_pose, 'requested target pose')
    _limits(arm['limits'], 'ordinary limits')
    _limits(drive, 'drive limits')
    delta = [(b-a)*1000 for a, b in zip(start_pose[:3], requested[:3])]
    distance = math.sqrt(sum(value*value for value in delta))
    if distance <= .5:
        raise ValueError('Follow displacement <=0.5mm requires caller to skip, not issue MoveL')
    lateral = math.hypot(delta[0], delta[1])
    if abs(delta[2]) <= 1e-6 and lateral <= 120:
        motion_kind = 'same_Z_lateral_at_most120mm'
    elif lateral <= 1e-6 and 30 <= abs(delta[2]) <= vertical_limit_mm:
        motion_kind = 'same_XY_vertical30_to%dmm' % vertical_limit_mm
    else:
        raise ValueError('Only same-Z lateral<=120mm or same-XY vertical30..%dmm is permitted'
                         % vertical_limit_mm)
    speed = number(speed, 'fresh line speed')
    acceleration = number(acceleration, 'fresh line acceleration')
    if speed <= 0 or acceleration <= 0:
        raise ValueError('Positive actual line dynamics required')
    if arm['frame_names'] != {'work': 'World', 'tool': 'Arm_Tip'}:
        raise ValueError('World/Arm_Tip required')
    if any(abs(number(value, 'frame component')) > 1e-9
           for row in arm['frame_signatures'].values()
           for field in ('position', 'euler') for value in row[field].values()):
        raise ValueError('Zero World/Arm_Tip required')
    if any(value <= 0 for value in _six(arm['limits']['speed'], 'joint speed')
           + _six(arm['limits']['acceleration'], 'joint acceleration')):
        raise ValueError('Positive actual ordinary joint dynamics required')
    percent = min(5, int(500/(speed*1000)))
    nominal = speed*1000*percent/100
    if percent < 1 or nominal <= 0 or nominal > 5 or distance/nominal > 60:
        raise ValueError('Actual dynamics cannot support the bounded <=5mm/s command')
    xyz = [value*1000 for value in start_pose[:3]]
    orientation = rotation(start_pose[3:])
    transform = frames(initial_joints)[-1]
    fk_error = math.dist([transform[i][3] for i in range(3)], xyz)
    fk_angle = matrix_angle([row[:3] for row in transform[:3]], orientation)
    if fk_error > 1 or fk_angle > .5:
        raise ValueError('Actual-DH FK does not agree with fresh controller pose')
    rows = []
    attempts = 0
    subdivisions = 0
    began = time.monotonic()

    def emit(row):
        if record is not None:
            record(row)

    def candidate(seed, first_fraction, last_fraction, depth):
        nonlocal attempts, subdivisions
        if attempts >= 4096 or time.monotonic()-began > 45:
            raise ValueError('Live IK calculation budget exceeded; no actuator command')
        attempts += 1
        point = [a+b*last_fraction for a, b in zip(xyz, delta)]
        interval = distance*(last_fraction-first_fraction)
        entry = dict(event='runtime_ik_candidate', attempt=attempts,
            from_fraction=first_fraction, to_fraction=last_fraction,
            interval_mm=interval, depth=depth, seed_joint_degrees=seed[:],
            target_world_mm=point)
        solution = None
        reason = None
        try:
            solution, error, angle = inverse_near(seed, point, orientation)
        except ValueError as exc:
            reason = 'solver_failed'
            entry['solver_error'] = repr(exc)
        if solution is not None:
            joint_delta = [b-a for a, b in zip(seed, solution)]
            excursion = [b-a for a, b in zip(initial_joints, solution)]
            ordinary = reserve(solution, arm['limits'])
            drive_reserve = reserve(solution, drive)
            entry.update(joint_degrees=solution[:], joint_delta_degrees=joint_delta,
                total_excursion_degrees=excursion, ordinary_reserve_degrees=ordinary,
                drive_reserve_degrees=drive_reserve, position_residual_mm=error,
                rotation_residual_degrees=angle)
            if not all(math.isfinite(value) for value in solution+[error, angle]):
                reason = 'nonfinite_solution'
            elif max(abs(value) for value in joint_delta) > 3:
                reason = 'adjacent_joint_delta_over_3degrees'
            elif max(abs(value) for value in excursion) > 60:
                reason = 'total_joint_excursion_over_60degrees'
            elif min(ordinary, drive_reserve) < 4:
                reason = 'sampled_joint_reserve_below_4degrees'
        if reason is None:
            entry['decision'] = 'accepted'
            emit(entry)
            rows.append(dict(joint_degrees=solution[:], target_world_mm=point,
                position_residual_mm=error, rotation_residual_degrees=angle,
                ordinary_reserve_degrees=ordinary, drive_reserve_degrees=drive_reserve))
            return solution
        split = (last_fraction > first_fraction and interval > .05
                 and depth < 12 and reason != 'nonfinite_solution')
        entry.update(reason=reason, decision='subdivide' if split else 'reject')
        emit(entry)
        if not split:
            raise ValueError('Live IK rejected: '+reason+'; complete candidate retained')
        subdivisions += 1
        middle = (first_fraction+last_fraction)/2
        halfway = candidate(seed, first_fraction, middle, depth+1)
        return candidate(halfway, middle, last_fraction, depth+1)

    count = int(math.ceil(distance/5))
    joints = candidate(initial_joints[:], 0., 0., 0)
    for index in range(1, count+1):
        joints = candidate(joints, (index-1)/count, index/count, 0)
    tracking = dict(
        minimum=[min([initial_joints[j]]+[row['joint_degrees'][j] for row in rows])-1
                 for j in range(6)],
        maximum=[max([initial_joints[j]]+[row['joint_degrees'][j] for row in rows])+1
                 for j in range(6)])
    target = requested[:3]+start_pose[3:]
    return dict(guard_kind='follow_line', motion_kind=motion_kind,
        vertical_limit_mm=vertical_limit_mm,
        start=copy.deepcopy(arm), drive_limits=copy.deepcopy(drive),
        target_sdk_pose_m_rad=target, target_world_mm=[value*1000 for value in target[:3]],
        distance_mm=distance, world_delta_mm=delta, fixed_wrist=True,
        requested_rpy_ignored=True, speed_percent=percent, nominal_speed_mm_s=nominal,
        arm_timeout_seconds=distance/nominal+2*nominal/(acceleration*1000*percent/100)+15,
        local_ik_rows=rows, live_joint_guard_limits=tracking,
        numerical_attempts=attempts, numerical_subdivisions=subdivisions,
        sampling_policy='adaptive_max5mm_and_max3degrees_unchanged_bounds',
        start_FK_error_mm=fk_error, start_FK_error_degrees=fk_angle,
        collision_certificate=False, trajectory_connect=0, radius=0, block=False)


def check_follow_segment(arm, plan):
    """Fresh physical wrist, joint branch and straight-line corridor guards."""
    if plan.get('guard_kind') != 'follow_line':
        raise ValueError('Follow-line guard requires its own computed plan')
    if (arm['frame_names'] != plan['start']['frame_names']
            or arm['frame_signatures'] != plan['start']['frame_signatures']
            or arm['limits'] != plan['start']['limits']):
        raise ValueError('Follow frames or complete ordinary limits/dynamics changed')
    joints = _six(arm['joints'], 'actual follow joints')
    pose = _six(arm['pose'], 'actual follow pose')
    if min(reserve(joints, arm['limits']), reserve(joints, plan['drive_limits'])) < 3:
        raise ValueError('Live follow joint reserve below3degrees')
    tracking = plan['live_joint_guard_limits']
    if any(not low <= value <= high for value, low, high in zip(joints,
            tracking['minimum'], tracking['maximum'])):
        raise ValueError('Actual follow joint branch left computed tracking box')
    start = plan['start']['pose']
    end = plan['target_sdk_pose_m_rad']
    direction = [(b-a)*1000 for a, b in zip(start[:3], end[:3])]
    length = math.sqrt(sum(value*value for value in direction))
    if not math.isfinite(length) or length <= .5:
        raise ValueError('Degenerate follow corridor')
    offset = [(b-a)*1000 for a, b in zip(start[:3], pose[:3])]
    projection = sum(a*b for a, b in zip(offset, direction))/length
    perpendicular = math.sqrt(sum((a-projection*b/length)**2
                                  for a, b in zip(offset, direction)))
    if perpendicular > 1 or not -1 <= projection <= length+1:
        raise ValueError('Actual TCP left follow straight-line corridor1mm')
    angle = orientation_angle_rad(pose[3:], start[3:])
    if angle > .005:
        raise ValueError('Physical follow wrist rotation exceeded0.005rad')
    return dict(perpendicular_error_mm=perpendicular, longitudinal_position_mm=projection,
                segment_length_mm=length, physical_rotation_rad=angle)

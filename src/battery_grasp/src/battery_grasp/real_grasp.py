"""Offline World/Arm_Tip grasp geometry. No robot driver or network dependency.

All dimensions are millimetres; RPY are radians, R = Rz Ry Rx.
Only a fixed tool orientation is supported. Geometric checks cover supplied
hand swept spheres, not the arm links, held battery or controller trajectory.
An accepted draft NEVER authorizes hardware execution.
"""
import math


def number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + ': finite number required')
    return float(value)


def vector(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(label + ': three coordinates required')
    return [number(x, label) for x in value]


def rotation(rpy):
    x, y, z = vector(rpy, 'tool_rpy_rad')
    cx, cy, cz = math.cos(x), math.cos(y), math.cos(z)
    sx, sy, sz = math.sin(x), math.sin(y), math.sin(z)
    return [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
            [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
            [-sy, cy*sx, cy*cx]]


def orientation_angle_rad(first_rpy, second_rpy):
    """Physical relative rotation for Rz Ry Rx, independent of Euler branches."""
    first, second = rotation(first_rpy), rotation(second_rpy)
    # Frobenius inner product equals trace(first.T @ second). Comparing
    # Euler components independently is not a bound on physical rotation.
    trace = sum(first[i][j]*second[i][j] for i in range(3) for j in range(3))
    return math.acos(max(-1., min(1., (trace-1.)/2.)))


def transform(matrix, point):
    return [sum(a*b for a, b in zip(row, point)) for row in matrix]


def tool_for_grasp(center_world_mm, offset_tool_mm, rpy):
    center = vector(center_world_mm, 'grasp center')
    offset = transform(rotation(rpy), vector(offset_tool_mm, 'grasp offset'))
    return [a-b for a, b in zip(center, offset)]


def segment_intersects_box(start, end, minimum, maximum):
    """Closed segment vs closed AABB by slab intersection, no sampling gaps."""
    lo, hi = 0., 1.
    for a, b, lower, upper in zip(start, end, minimum, maximum):
        delta = b-a
        if abs(delta) < 1e-12:
            if a < lower or a > upper:
                return False
        else:
            t1, t2 = (lower-a)/delta, (upper-a)/delta
            lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
            if lo > hi:
                return False
    return True


def workspace_errors(start, end, bands, rpy):
    """Check every crossed height interval using its own certified bounds.

    Bands apply to Arm_Tip, at the specified fixed orientation. Their limits
    are constant conservative bounds throughout each entire height interval;
    point measurements or two Y/Z examples are not certified bands.
    """
    cuts = {0., 1.}
    dz = end[2]-start[2]
    if abs(dz) > 1e-12:
        for band in bands:
            for height in (band['z_min_mm'], band['z_max_mm']):
                t = (height-start[2])/dz
                if 0. < t < 1.:
                    cuts.add(t)
    cuts = sorted(cuts)
    problems = []
    for ta, tb in zip(cuts, cuts[1:]):
        mid_z = start[2] + dz*(ta+tb)/2.
        covering = [b for b in bands if b['z_min_mm'] <= mid_z <= b['z_max_mm']]
        if not covering:
            problems.append('No certified height band at Z %.3f mm' % mid_z)
            continue
        for band in covering:  # overlaps use the intersection, never the looser union
            if any(abs(math.atan2(math.sin(a-b), math.cos(a-b))) > .005
                   for a, b in zip(rpy, band['tool_rpy_rad'])):
                problems.append('Workspace orientation does not match the planned tool orientation')
            for t in (ta, tb):
                p = [a+(b-a)*t for a, b in zip(start, end)]
                if not (band['x_min_mm'] <= p[0] <= band['x_max_mm'] and
                        band['y_min_mm'] <= p[1] <= band['y_max_mm']):
                    problems.append('Arm_Tip outside certified X/Y limits at Z %.3f mm' % p[2])
    return sorted(set(problems))


def measured(row, label):
    if not isinstance(row, dict) or row.get('verified') is not True or not isinstance(row.get('source'), str) or not row['source'].strip():
        raise ValueError(label + ': measured/verified record with source required')
    return row


REQUIRED = ('grasp_offset', 'support_plane', 'hand_envelope', 'workspace', 'safe_tool_z',
            'approach_distance_mm', 'lift_distance_mm', 'clearance_mm')
EXECUTION_GAPS = [
    'Fresh on-device preflight and timestamp/frame verification at execution time',
    'Controller IK and full trajectory joint reserves >=3 degrees; arm-link collision checks',
    'Contact stop and release validated on-device; exit/state2/readback is not torque release',
    'Real battery hold, contact geometry and held-object swept envelope validation',
    'Real-arm execution adapter and supervised single-step acceptance',
]


def build_plan(baseline, config, profile, target=None):
    """Return an offline draft and ALL available blockers, without device writes."""
    result = dict(schema_version=1, frame='World', tool='Arm_Tip', units='mm/rad',
                  planning_ready=False, execution_ready=False, hardware_writes=0,
                  blockers=[], execution_gaps=EXECUTION_GAPS[:], waypoints=[],
                  candidate_hand_sequence=[], geometric_scope='fixed-orientation supplied hand envelope only')
    try:
        if not isinstance(config, dict) or type(config.get('schema_version')) is not int or config['schema_version'] != 1:
            raise ValueError('Unsupported geometry config schema')
        if (config.get('frame'),config.get('tool'),config.get('units')) != ('World','Arm_Tip','mm/rad'):
            raise ValueError('Geometry reference/units must be World / Arm_Tip / mm/rad')
        if not isinstance(profile, dict) or type(profile.get('schema_version')) is not int or profile['schema_version'] != 1:
            raise ValueError('Unsupported hand profile schema')
        expected = {'open': [0,0,0,0,0,728], 'prepare': [8000,8000,8000,8000,65535,65535],
                    'battery': [15000,40000,35000,40000,65535,65535]}
        if profile.get('poses') != expected or profile.get('device_identity') != [512,769,151,8193,290,2,0,0]:
            raise ValueError('Hand profile does not match the accepted device/poses')
        validation = profile.get('validation', {})
        if not all(validation.get(k) is True for k in ('empty_hand_one_sequence_feedback_passed',
                   'thumb_faces_palm_confirmed_by_user', 'visual_handshape_accepted_by_user')):
            raise ValueError('Accepted empty-hand/appearance evidence missing')
        result['profile_id'] = profile.get('profile_id')
        # School adaptation ordering is preserved, never use original thumb45000.
        result['candidate_hand_sequence'] = [
            {'phase':'prepare','targets':expected['prepare']},
            {'phase':'close_candidate','targets':[15000,30000,35000,40000,65535,65535]},
            {'phase':'final_index','targets':expected['battery']},
            {'phase':'release_thumb','targets':[0,40000,35000,40000,65535,65535]},
            {'phase':'release_four','targets':[0,0,0,0,0,65535]},
            {'phase':'return_thumb','targets':expected['open']}]
        missing = [key for key in REQUIRED if config.get(key) is None]
        result['blockers'].extend('Missing measured geometry: '+key for key in missing)
        if not isinstance(baseline, dict) or baseline.get('diagnostic_pass') is not True:
            result['blockers'].append('Stationary before/after baseline has not passed')
        arm = baseline.get('analysis', {}).get('current_arm') if isinstance(baseline, dict) else None
        if not arm:
            result['blockers'].append('Missing complete bracket current arm reference')
        if not isinstance(target, dict) or target.get('frame') != 'World' or target.get('point_kind') != 'grasp_center':
            result['blockers'].append('Target must be an explicit World grasp_center; visible surface is not a grasp target')
        if config.get('obstacles') is None:
            result['blockers'].append('Missing inspected obstacle list (empty list requires an explicit clear-scene record)')
        if missing or not arm or not isinstance(target, dict) or target.get('frame') != 'World' or target.get('point_kind') != 'grasp_center' or config.get('obstacles') is None:
            return result
        if baseline.get('analysis', {}).get('frame_names') != {'work':'World','tool':'Arm_Tip'}:
            result['blockers'].append('Baseline reference frames mismatch')
        reserves = arm.get('joint_reserve_degrees')
        if not isinstance(reserves, list) or len(reserves) != 6 or any(number(x,'joint reserve') < 3 for x in reserves):
            result['blockers'].append('Current joint reserve below 3 degrees or unavailable')
        start = vector(arm.get('tcp_position_mm'), 'current TCP')
        rpy = vector(arm.get('tcp_rpy_rad'), 'current tool RPY')
        offset = vector(measured(config['grasp_offset'],'grasp_offset').get('tool_mm'),'grasp offset')
        plane = number(measured(config['support_plane'],'support_plane').get('world_z_mm'),'support plane')
        safe_z = number(measured(config['safe_tool_z'],'safe_tool_z').get('world_z_mm'),'safe tool Z')
        envelope = measured(config['hand_envelope'],'hand_envelope')
        if envelope.get('covers') != ['open','prepare','battery','transitions']:
            raise ValueError('Hand envelope must cover every hand pose and all transitions')
        spheres = envelope.get('spheres')
        if not isinstance(spheres, list) or not spheres or not any(s.get('part') == 'little_finger' for s in spheres if isinstance(s,dict)):
            raise ValueError('Whole-hand swept envelope must explicitly include little_finger')
        sphere_values = []
        for s in spheres:
            radius = number(s.get('radius_mm'),'sphere radius')
            if radius <= 0:
                raise ValueError('Sphere radius must be positive')
            sphere_values.append((transform(rotation(rpy),vector(s.get('tool_center_mm'),'sphere center')),radius))
        workspace = measured(config['workspace'],'workspace')
        if workspace.get('reference') != 'Arm_Tip' or workspace.get('coverage') != 'full_height_intervals_fixed_orientation':
            raise ValueError('Workspace needs certified full height intervals for Arm_Tip; point examples cannot define a boundary')
        bands = workspace.get('bands')
        if not isinstance(bands,list) or not bands:
            raise ValueError('Workspace requires height bands')
        for b in bands:
            for low, high in (('z_min_mm','z_max_mm'),('x_min_mm','x_max_mm'),('y_min_mm','y_max_mm')):
                if number(b.get(low),low) >= number(b.get(high),high):
                    raise ValueError('Workspace interval is empty or reversed')
            vector(b.get('tool_rpy_rad'),'workspace RPY')
        distances = [number(config[k],k) for k in ('approach_distance_mm','lift_distance_mm','clearance_mm')]
        approach, lift, clearance = distances
        if any(d <= 0 for d in distances):
            raise ValueError('Approach/lift/clearance must be positive explicit values')
        obstacles = measured(config['obstacles'],'obstacles').get('boxes')
        if not isinstance(obstacles,list):
            raise ValueError('Obstacles boxes must be a list')
        boxes = []
        for b in obstacles:
            low, high = vector(b.get('min_mm'),'obstacle minimum'), vector(b.get('max_mm'),'obstacle maximum')
            if any(a >= z for a,z in zip(low,high)):
                raise ValueError('Obstacle box is empty or reversed')
            boxes.append((low,high))
        tool = tool_for_grasp(target.get('world_mm'),offset,rpy)
        if safe_z < max(start[2],tool[2]+approach,tool[2]+lift):
            raise ValueError('Safe tool Z must cover current, approach and lift heights')
        points = [('current',start), ('raise_clear',[start[0],start[1],safe_z]),
                  ('transfer_above',[tool[0],tool[1],safe_z]),
                  ('approach',[tool[0],tool[1],tool[2]+approach]),
                  ('grasp',tool), ('lift',[tool[0],tool[1],tool[2]+lift])]
        result['waypoints'] = [dict(phase=phase,tool_world_mm=p,tool_rpy_rad=rpy) for phase,p in points]
        result['grasp_center_world_mm'] = vector(target.get('world_mm'),'grasp center')
        bottom_min = float('inf')
        for (name,a),(next_name,b) in zip(points,points[1:]):
            prefix = name+' -> '+next_name+': '
            result['blockers'].extend(prefix+e for e in workspace_errors(a,b,bands,rpy))
            for offset_s,radius in sphere_values:
                ca,cb = [x+y for x,y in zip(a,offset_s)], [x+y for x,y in zip(b,offset_s)]
                bottom = min(ca[2],cb[2])-radius
                bottom_min = min(bottom_min,bottom)
                if bottom < plane+clearance:
                    result['blockers'].append(prefix+'hand/table clearance violated')
                for low,high in boxes:
                    # Expanded AABB is conservative for sphere swept volume.
                    padding = radius+clearance
                    if segment_intersects_box(ca,cb,[v-padding for v in low],[v+padding for v in high]):
                        result['blockers'].append(prefix+'hand swept envelope intersects obstacle margin')
        result['minimum_hand_table_clearance_mm'] = bottom_min-plane
        result['blockers'] = sorted(set(result['blockers']))
        result['planning_ready'] = not result['blockers']
    except (ValueError,TypeError,KeyError,OverflowError) as exc:
        result['blockers'].append(str(exc))
    return result

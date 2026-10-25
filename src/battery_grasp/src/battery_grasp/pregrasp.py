"""Offline fixed-wrist vertical pregrasp corridor, without contact or actuation."""
from .real_grasp import number, vector, rotation, transform, measured
from .real_grasp import workspace_errors, segment_intersects_box

REQUIRED = ('target_tip','support_plane','hand_envelope','workspace','clearance_mm','obstacles','battery_box')


def build_pregrasp_plan(start, config):
    result = dict(schema_version=1,frame='World',tool='Arm_Tip',units='mm/rad',
        planning_ready=False,execution_ready=False,hardware_writes=0,arm_motion_allowed=False,
        contact_allowed=False,automatic_lift=False,blockers=[],waypoints=[],
        execution_gaps=['Fresh empty-hand/open preflight at runtime',
            'Controller IK, full-route joint reserves >=3 degrees and arm-link/cable collision check',
            'Supervised actual vertical trajectory acceptance'],
        scope='fixed-wrist vertical empty-hand pregrasp only; no grip, no grasp-center inference')
    try:
        if (not isinstance(config,dict) or type(config.get('schema_version')) is not int
                or config['schema_version'] != 1 or (config.get('frame'),config.get('tool'),config.get('units'))
                != ('World','Arm_Tip','mm/rad')):
            raise ValueError('Expected schema1 / World / Arm_Tip / mm/rad')
        a = vector(start.get('tool_world_mm'),'start TCP')
        rpy = vector(start.get('tool_rpy_rad'),'start RPY')
        result['start_reference'] = dict(tool_world_mm=a,tool_rpy_rad=rpy,
            current_pose_verified=False,source=start.get('source'))
        result['blockers'] += ['Missing measured pregrasp geometry: '+key for key in REQUIRED if config.get(key) is None]
        if result['blockers']: return result
        b = vector(measured(config['target_tip'],'target_tip')['world_mm'],'target TCP')
        if any(abs(a[ch]-b[ch]) > 1e-6 for ch in (0,1)) or b[2] >= a[2]:
            raise ValueError('Vertical descent requires identical World X/Y and lower Z')
        clearance = number(config['clearance_mm'],'clearance')
        if clearance <= 0: raise ValueError('Positive explicit clearance required')
        plane = number(measured(config['support_plane'],'support plane')['world_z_mm'],'table Z')
        env = measured(config['hand_envelope'],'hand envelope')
        if env.get('covers') != ['open','prepare','transitions']:
            raise ValueError('Whole-hand envelope must cover open/prepare/transitions')
        spheres = env.get('spheres')
        if not isinstance(spheres,list) or not spheres or not any(isinstance(s,dict) and s.get('part')=='little_finger' for s in spheres):
            raise ValueError('Whole-hand envelope must include little_finger')
        workspace = measured(config['workspace'],'workspace')
        if workspace.get('reference') != 'Arm_Tip' or workspace.get('coverage') != 'full_height_intervals_fixed_orientation':
            raise ValueError('Workspace needs full height intervals at this wrist orientation')
        bands = workspace.get('bands')
        if not isinstance(bands,list) or not bands: raise ValueError('Height bands missing')
        for band in bands:
            for low,high in (('z_min_mm','z_max_mm'),('x_min_mm','x_max_mm'),('y_min_mm','y_max_mm')):
                if number(band.get(low),low) >= number(band.get(high),high):
                    raise ValueError('Height/workspace interval reversed')
            vector(band.get('tool_rpy_rad'),'band orientation')
        obstacles = measured(config['obstacles'],'obstacles')['boxes']
        battery = measured(config['battery_box'],'battery box')
        if not isinstance(obstacles,list): raise ValueError('Explicit inspected obstacle list required')
        boxes = []
        for box in obstacles+[battery]:
            low,high = vector(box.get('min_mm'),'box minimum'),vector(box.get('max_mm'),'box maximum')
            if any(x>=y for x,y in zip(low,high)): raise ValueError('Obstacle/battery box reversed')
            boxes.append((low,high))
        result['waypoints'] = [dict(phase='accepted_hover',tool_world_mm=a,tool_rpy_rad=rpy),
            dict(phase='noncontact_pregrasp',tool_world_mm=b,tool_rpy_rad=rpy)]
        result['drop_mm'] = a[2]-b[2]
        result['hand_transition_location'] = 'hover only; do not change the hand pose during descent'
        result['blockers'] += workspace_errors(a,b,bands,rpy)
        minimum = float('inf')
        for sphere in spheres:
            radius = number(sphere.get('radius_mm'),'sphere radius')
            if radius <= 0: raise ValueError('Positive sphere radius required')
            offset = transform(rotation(rpy),vector(sphere.get('tool_center_mm'),'sphere center'))
            ca,cb = [x+y for x,y in zip(a,offset)],[x+y for x,y in zip(b,offset)]
            minimum = min(minimum,min(ca[2],cb[2])-radius-plane)
            if min(ca[2],cb[2])-radius < plane+clearance:
                result['blockers'].append('Hand/table clearance violated: '+str(sphere.get('part')))
            for low,high in boxes:
                padding = radius+clearance
                if segment_intersects_box(ca,cb,[v-padding for v in low],[v+padding for v in high]):
                    result['blockers'].append('Swept hand intersects battery/obstacle margin: '+str(sphere.get('part')))
        result['minimum_hand_table_clearance_mm'] = minimum
        result['blockers'] = sorted(set(result['blockers']))
        result['planning_ready'] = not result['blockers']
    except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
        result['blockers'].append(str(exc))
    return result

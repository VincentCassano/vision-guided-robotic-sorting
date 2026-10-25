"""ROS-independent, fail-closed stage-4 simulation policy. Coordinates are metres."""
import copy
import math
import uuid

VERSION = 1
CLASSES = ('NORMAL_BLACK', 'DEFECT_RED', 'DEFECT_WHITE')
STEPS = ('SAFE_APPROACH', 'ABOVE_TARGET', 'VERTICAL_DESCEND', 'GRASP',
         'CLOSE_HAND', 'VERTICAL_LIFT', 'SAFE_TRANSFER', 'OPEN_HAND', 'DONE')


class UnsafePlan(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise UnsafePlan(message)


def finite(values):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def inside(point, bounds):
    return len(point) == 3 and finite(point) and all(bounds[k+'_min'] <= point[i] <= bounds[k+'_max'] for i,k in enumerate('xyz'))


def slot(position, cfg):
    require(len(position) == 3 and finite(position), 'Invalid target position')
    rows, cols = cfg['grid']['row_x'], cfg['grid']['column_y']
    row = min(range(3), key=lambda i: abs(position[0]-rows[i]))
    col = min(range(3), key=lambda i: abs(position[1]-cols[i]))
    require(abs(position[0]-rows[row]) <= cfg['grid']['tolerance'] and
            abs(position[1]-cols[col]) <= cfg['grid']['tolerance'], 'Target outside calibrated grid')
    return row, col, 'r{}c{}'.format(row, col)


def validate_objects(objects, cfg):
    require(isinstance(objects, list), 'Objects must be an array')
    ids = set()
    for obj in objects:
        require(isinstance(obj,dict) and type(obj.get('row')) is int and type(obj.get('column')) is int, 'Invalid grid fields')
        row, col, identity = slot(obj['position'], cfg)
        require(obj['id'] == identity and obj['row'] == row and obj['column'] == col, 'Invalid grid identity')
        require(identity not in ids, 'Multiple detections in one slot')
        require(obj['category'] in CLASSES, 'Unknown or unsupported category')
        require(inside(obj['position'], cfg['workspace']), 'Target outside workspace')
        ids.add(identity)


def bin_for(category):
    require(category in CLASSES, 'No sorting rule for category')
    return 'LEFT_UPPER_BIN' if category == 'NORMAL_BLACK' else 'LEFT_LOWER_BIN'


def vertical_clear(target, objects, cfg):
    x, y, z = target['position']
    radius = cfg['motion']['hand_radius']
    separation = radius + cfg['motion']['battery_radius'] + cfg['motion']['clearance']
    for other in objects:
        if other['id'] != target['id']:
            require(math.hypot(x-other['position'][0], y-other['position'][1]) > separation,
                    'Vertical channel overlaps another battery')
    for obstacle in cfg['obstacles']:
        b = obstacle['bounds']
        overlap = b['x_min']-radius <= x <= b['x_max']+radius and b['y_min']-radius <= y <= b['y_max']+radius
        require(not (overlap and b['z_max'] >= z), 'Vertical channel intersects '+obstacle['name'])


def validate_config(cfg):
    require(cfg.get('simulation_only') is True and cfg.get('frame_id') == 'base_link', 'Simulation/base_link required')
    require(finite([cfg['step_delay'],cfg['max_age'],cfg['max_target_shift']]) and cfg['step_delay'] > 0 and cfg['max_age'] > 0 and cfg['max_target_shift'] >= 0, 'Invalid timing/tolerance configuration')
    rotation = cfg['camera']['rotation']
    require(len(rotation) == 3 and all(len(row) == 3 and finite(row) for row in rotation), 'Invalid camera rotation')
    require(all(abs(sum(rotation[i][k]*rotation[j][k] for k in range(3))-(1 if i==j else 0)) < 1e-6 for i in range(3) for j in range(3)), 'Camera rotation not orthonormal')
    determinant = sum(rotation[0][i]*(rotation[1][(i+1)%3]*rotation[2][(i+2)%3]-rotation[1][(i+2)%3]*rotation[2][(i+1)%3]) for i in range(3))
    require(abs(determinant-1) < 1e-6 and len(cfg['camera']['translation']) == 3 and finite(cfg['camera']['translation']), 'Invalid camera rigid transform')
    require(len(cfg['grid']['row_x']) == len(cfg['grid']['column_y']) == 3, 'Expected 3x3 grid')
    require(finite(cfg['grid']['row_x']+cfg['grid']['column_y']), 'Nonfinite grid')
    require(0 < cfg['grid']['tolerance'] < min(abs(a-b) for values in [cfg['grid']['row_x'],cfg['grid']['column_y']] for a,b in zip(values,values[1:]))/2, 'Ambiguous grid tolerance')
    safe = cfg['motion']['safe_z']
    margin = cfg['motion']['clearance']
    require(finite([safe,margin,cfg['motion']['hand_radius'],cfg['motion']['battery_radius']]) and margin > 0 and cfg['motion']['hand_radius'] > 0 and cfg['motion']['battery_radius'] > 0, 'Invalid safety dimensions')
    for obstacle in cfg['obstacles']:
        require(safe >= obstacle['bounds']['z_max']+margin, 'SAFE_Z below obstacle clearance')
    for pose in cfg['bins'].values():
        require(inside(pose, cfg['workspace']) and abs(pose[2]-safe) < 1e-6, 'Drop pose must be at SAFE_Z inside workspace')
    for arm in ('LEFT','RIGHT'):
        require(inside(cfg['arms'][arm]['home'],cfg['arms'][arm]['reach']) and inside(cfg['arms'][arm]['home'],cfg['workspace']) and abs(cfg['arms'][arm]['home'][2]-safe) < 1e-6, 'Home must be reachable and at SAFE_Z')


def validate_transition(start, end, cfg):
    require(inside(start, cfg['workspace']) and inside(end, cfg['workspace']), 'Motion outside workspace')
    safe = cfg['motion']['safe_z']
    horizontal = abs(start[0]-end[0]) > 1e-6 or abs(start[1]-end[1]) > 1e-6
    require(not horizontal or (start[2] >= safe-1e-6 and end[2] >= safe-1e-6 and abs(start[2]-end[2]) < 1e-6), 'Lateral or diagonal motion below SAFE_Z')


def build_sequence(task, cfg):
    validate_config(cfg)
    require(type(task['schema_version']) is int and task['schema_version'] == VERSION and task['frame_id'] == cfg['frame_id'], 'Unsupported task schema/frame')
    objects, target, arm = task['objects'], task['target'], task['arm']
    validate_objects(objects, cfg)
    require(sum(o == target for o in objects) == 1, 'Target not in perception snapshot')
    require(arm in ('LEFT','RIGHT'), 'Unknown arm')
    if arm == 'RIGHT':
        require(cfg['arms']['right_enabled'] and cfg['arms']['right_paths_verified'], 'Right arm not permitted')
    require(task['bin'] == bin_for(target['category']), 'Wrong destination bin')
    columns = {o['column'] for o in objects}
    if target['column'] == 1:
        require(0 not in columns or 2 not in columns, 'Middle blocked by both outer columns')
        require((arm == 'LEFT' and 0 not in columns) or (arm == 'RIGHT' and 2 not in columns), 'Selected arm has no cleared side')
    if target['column'] == 2 and arm == 'LEFT':
        require(0 not in columns and 1 not in columns, 'Left arm cannot skip occupied inner columns')
    if target['column'] == 0:
        require(arm == 'LEFT', 'Left column requires left arm')
    vertical_clear(target, objects, cfg)
    safe = cfg['motion']['safe_z']
    require(all(o['position'][2]+cfg['motion']['clearance'] < safe for o in objects), 'SAFE_Z below battery clearance')
    start = list(task['start_pose'])
    require(abs(start[2]-safe) <= 1e-6, 'Unknown/low starting pose: safe approach not validated')
    x,y,z = target['position']
    above, grasp = [x,y,safe], [x,y,z]
    drop = list(cfg['bins'][task['bin']])
    poses = [start, above, grasp, grasp, grasp, above, drop, drop, drop]
    previous = start
    for pose in poses:
        validate_transition(previous, pose, cfg)
        require(inside(pose, cfg['arms'][arm]['reach']), 'Selected arm cannot reach waypoint')
        previous = pose
    result = copy.deepcopy(task)
    result['steps'] = [{'state':state,'position':pose} for state,pose in zip(STEPS,poses)]
    return result


def validate_sequence(sequence, cfg, current_pose):
    expected = build_sequence(sequence, cfg)
    require(sequence.get('steps') == expected['steps'], 'Sequence differs from validated safe path')
    require(len(current_pose) == 3 and all(abs(a-b) < 1e-6 for a,b in zip(current_pose,sequence['start_pose'])), 'Stale arm start pose')


class Planner:
    def __init__(self, cfg, session=None, require_full_inventory=True):
        validate_config(cfg)
        self.cfg = cfg
        self.session = session or uuid.uuid4().hex
        self.require_full_inventory = require_full_inventory
        self.inventory = None
        self.completed = set()
        self.current = None
        self.poses = {arm:list(cfg['arms'][arm]['home']) for arm in ('LEFT','RIGHT')}
        self.status, self.reason = 'WAITING', 'Waiting for full nine-battery inventory'
        self.last_completion_stamp = -1.0
        self.failed = False

    def observe(self, objects, stamp, now):
        if self.failed or self.current is not None:
            return None
        try:
            require(finite([stamp,now]) and 0 <= now-stamp <= self.cfg['max_age'], 'Stale/future perception')
            validate_objects(objects,self.cfg)
            seen = {o['id']:o for o in objects}
            if self.inventory is None:
                require(len(seen) == 9 if self.require_full_inventory else bool(seen), 'Initial inventory is incomplete or empty')
                self.inventory = copy.deepcopy(seen)
            require(stamp > self.last_completion_stamp, 'Waiting for post-completion camera frame')
            require(not (set(seen) & self.completed), 'Waiting for completed battery to disappear')
            require(set(seen) == set(self.inventory)-self.completed, 'Uncompleted target missing or unexpected target present')
            for identity,obj in seen.items():
                require(obj['category'] == self.inventory[identity]['category'], 'Target class changed')
                require(math.dist(obj['position'],self.inventory[identity]['position']) <= self.cfg['max_target_shift'], 'Target moved from recorded position')
            if not seen:
                self.status,self.reason = 'DONE','All recorded batteries sorted and absent from camera'
                return None
            candidates = sorted(objects,key=lambda o:(o['column'],o['row']))
            failures = []
            for arm in ('LEFT','RIGHT'):
                for target in candidates:
                    task = {'schema_version':VERSION,'frame_id':self.cfg['frame_id'],'task_id':self.session+':'+target['id'],
                            'stamp':stamp,'target':copy.deepcopy(target),'objects':copy.deepcopy(objects),
                            'arm':arm,'bin':bin_for(target['category']),'start_pose':list(self.poses[arm])}
                    try:
                        build_sequence(task,self.cfg)
                    except UnsafePlan as exc:
                        failures.append(str(exc))
                        continue
                    self.current = task
                    self.status,self.reason = 'RUNNING','Executing '+target['id']
                    return task
            raise UnsafePlan('No safe reachable target: '+'; '.join(sorted(set(failures))))
        except (UnsafePlan, KeyError, TypeError, IndexError) as exc:
            self.status,self.reason = 'BLOCKED',str(exc)
            return None

    def finish(self, result):
        if self.current is None or result.get('task_id') != self.current['task_id']:
            return False
        task = self.current
        if result.get('status') != 'DONE':
            self.failed = True
            self.status,self.reason = 'ERROR',result.get('reason','Execution failed')
            return False
        require(result['schema_version'] == VERSION and result['target_id'] == task['target']['id'] and result['arm'] == task['arm'] and result['bin'] == task['bin'], 'Completion metadata mismatch')
        require(result['final_pose'] == self.cfg['bins'][task['bin']], 'Completion not at bin')
        require(finite([result['stamp']]) and result['stamp'] >= task['stamp'], 'Invalid completion timestamp')
        self.completed.add(task['target']['id'])
        self.poses[task['arm']] = list(result['final_pose'])
        self.last_completion_stamp = result['stamp']
        self.current = None
        self.status,self.reason = 'WAITING','Waiting for refreshed scene after drop'
        return True


class SimExecutor:
    """Timer-driven simulated executor shared by ROS and offline verification."""
    def __init__(self,cfg):
        validate_config(cfg)
        self.cfg = cfg
        self.poses = {arm:list(cfg['arms'][arm]['home']) for arm in ('LEFT','RIGHT')}
        self.active,self.latest = None,None
        self.index = 0
        self.seen = set()
        self.failed = False
        self.running = False

    def set_running(self,running):
        self.running = running is True
        if not self.running:
            self.active = None

    def observe(self,snapshot):
        self.latest = copy.deepcopy(snapshot)

    def fresh(self,sequence,now):
        s = self.latest
        require(s is not None and s.get('schema_version') == VERSION and s.get('valid') is True and s.get('frame_id') == self.cfg['frame_id'], 'Invalid/missing live scene')
        require(finite([now,s['stamp']]) and 0 <= now-s['stamp'] <= self.cfg['max_age'], 'Live scene timed out')
        validate_objects(s['objects'],self.cfg)
        actual = {o['id']:o for o in s['objects']}
        require(set(actual) == {o['id'] for o in sequence['objects']}, 'Scene inventory changed during execution')
        for obj in sequence['objects']:
            current = actual[obj['id']]
            require(current['category'] == obj['category'] and math.dist(current['position'],obj['position']) <= self.cfg['max_target_shift'], 'Scene moved or category changed')
        vertical_clear(actual[sequence['target']['id']],s['objects'],self.cfg)
        vertical_clear(sequence['target'],s['objects'],self.cfg)

    def accept(self,sequence,now):
        if not self.running or self.failed or self.active is not None or sequence['task_id'] in self.seen or self.latest is None:
            return False
        validate_sequence(sequence,self.cfg,self.poses[sequence['arm']])
        self.fresh(sequence,now)
        self.active,self.index = copy.deepcopy(sequence),0
        return True

    def fail(self,sequence,reason):
        self.active = None
        self.failed = True
        self.seen.add(sequence['task_id'])
        return {'schema_version':VERSION,'task_id':sequence['task_id'],'status':'ERROR','reason':str(reason)}

    def tick(self,now):
        sequence = self.active
        if not self.running or sequence is None or self.failed:
            return None,None
        try:
            self.fresh(sequence,now)
            step = sequence['steps'][self.index]
            validate_transition(self.poses[sequence['arm']],step['position'],self.cfg)
            self.poses[sequence['arm']] = list(step['position'])
            event = {'schema_version':VERSION,'task_id':sequence['task_id'],'target_id':sequence['target']['id'],
                     'arm':sequence['arm'],'state':step['state'],'position':list(step['position']),'stamp':now}
            self.index += 1
            result = None
            if step['state'] == 'DONE':
                self.seen.add(sequence['task_id'])
                self.active = None
                result = {'schema_version':VERSION,'task_id':sequence['task_id'],'status':'DONE',
                          'target_id':sequence['target']['id'],'arm':sequence['arm'],'bin':sequence['bin'],
                          'final_pose':list(step['position']),'stamp':now}
            return event,result
        except (ValueError,KeyError,TypeError,IndexError) as exc:
            return None,self.fail(sequence,exc)

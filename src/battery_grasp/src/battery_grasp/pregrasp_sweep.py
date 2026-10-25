"""Conservative continuous sweep of supplied rigid parts on affine joint cells.

Pure file/math processing. Supplied part boxes are in actual measured MDH
frames, NOT arbitrary URDF link frames. A conditional numerical result never
certifies installed geometry, scene completeness, controller interpolation,
start state, stopping or device execution.
"""
import itertools
import math
from .real_grasp import number, rotation, vector
from .taught_dh_model import EXPECTED_DH
from .taught_path import reserves
from .vertical_corridor import accepted_native_result, NATIVE_REPORT_SHA

REQUIRED_COMPONENTS=('arm_1','arm_2','arm_3','arm_4','arm_5','arm_6',
                     'whole_hand','little_finger','wrist_camera_adapter','external_cables')


def product(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def frames(joints):
    if not isinstance(joints,(list,tuple)) or len(joints)!=6:raise ValueError('Six joint angles required')
    joints=[number(x,'joint angle') for x in joints]
    r=rotation([0,math.pi/4,0]);t=[r[i]+[0.] for i in range(3)]+[[0.,0.,0.,1.]]
    result=[t]
    for q,(alpha,a,d,offset) in zip(joints,EXPECTED_DH):
        alpha=math.radians(alpha/1000);a/=1000;d/=1000;theta=math.radians(q+offset/1000)
        c,s,ca,sa=math.cos(theta),math.sin(theta),math.cos(alpha),math.sin(alpha)
        t=product(t,[[c,-s,0,a],[s*ca,c*ca,-sa,-sa*d],[s*sa,c*sa,ca,ca*d],[0,0,0,1]])
        result.append(t)
    return result


def box(low,high):
    low,high=vector(low,'box minimum'),vector(high,'box maximum')
    if any(a>=b for a,b in zip(low,high)):raise ValueError('Box must have positive extent')
    return low,high


def corners(low,high):
    return list(itertools.product(*zip(low,high)))


def world_box(joints,part):
    t=frames(joints)[part['mdh_frame']]
    points=[[sum(t[i][j]*p[j] for j in range(3))+t[i][3] for i in range(3)]
            for p in corners(part['local_min_mm'],part['local_max_mm'])]
    return [min(p[i] for p in points) for i in range(3)],[max(p[i] for p in points) for i in range(3)]


def motion_bound_mm(start,end,frame,radius):
    """Every point in a local radius for any q between componentwise endpoints.

    MDH joint i rotates after its own a/alpha translation; its d is along its
    rotation axis. Downstream reach is at most sum(hypot(a_k,d_k)) + radius.
    Sequential one-joint replacements and triangle inequality bound arbitrary
    configurations inside the joint box, not just endpoint Cartesian chords.
    """
    if type(frame) is not int or not 0<=frame<=6:raise ValueError('MDH frame must be integer0..6')
    radius=number(radius,'local enclosing radius')
    if radius<0:raise ValueError('Negative radius forbidden')
    if len(start)!=6 or len(end)!=6:raise ValueError('Six joint angles required')
    start=[number(v,'start angle') for v in start];end=[number(v,'end angle') for v in end]
    bound=0.
    for i in range(frame):
        reach=radius+sum(math.hypot(EXPECTED_DH[k][1],EXPECTED_DH[k][2])/1000 for k in range(i+1,frame))
        angle=min(math.pi,math.radians(abs(end[i]-start[i])))
        bound+=2*reach*math.sin(angle/2)
    return bound


def intersect(a,b):
    return all(a[0][i]<=b[1][i] and b[0][i]<=a[1][i] for i in range(3))


def sweep_cell(start,end,part,obstacles,margin,padding_tolerance=1.,max_depth=12):
    """Adaptive conservative bounds; intersections remain potential, not proof.

    Affine q is the engineering curve under review, not controller MoveJ/MoveL.
    Refinement tightens bounds only. No obstacle is skipped at sampled gaps.
    """
    margin=number(margin,'explicit obstacle clearance')
    tolerance=number(padding_tolerance,'numerical padding tolerance')
    if margin<0 or tolerance<=0:raise ValueError('Invalid clearance/refinement tolerance')
    if type(max_depth) is not int or not 0<=max_depth<=16:raise ValueError('Invalid refinement depth')
    rho=max(math.sqrt(sum(v*v for v in p)) for p in corners(part['local_min_mm'],part['local_max_mm']))
    allowance=part['geometry_allowance_mm']
    cells=[]
    def visit(a,b,lo,hi,depth):
        bound=motion_bound_mm(a,b,part['mdh_frame'],rho)
        low,high=world_box(a,part);padding=bound+allowance+margin
        swept=([v-padding for v in low],[v+padding for v in high])
        hits=[o['id'] for o in obstacles if intersect(swept,(o['world_min_mm'],o['world_max_mm']))]
        if hits and bound>tolerance and depth<max_depth:
            mid=[(v+w)/2 for v,w in zip(a,b)];tm=(lo+hi)/2
            visit(a,mid,lo,tm,depth+1);visit(mid,b,tm,hi,depth+1);return
        cells.append(dict(t_start=lo,t_end=hi,minimum_mm=swept[0],maximum_mm=swept[1],
            motion_padding_mm=bound,obstacle_ids=hits,
            conditional_result='potential_intersection' if hits else 'disjoint',
            refinement_limit_reached=bool(hits and bound>tolerance and depth==max_depth)))
    visit(start,end,0.,1.,0)
    return dict(part_id=part['id'],cells=cells,conditional_disjoint_pass=all(not c['obstacle_ids'] for c in cells),
                controller_interpolation_verified=False,physical_collision_verified=False,execution_ready=False)


def read_geometry(config):
    if not isinstance(config,dict) or type(config.get('schema_version')) is not int or config['schema_version']!=1:
        raise ValueError('Sweep schema1 required')
    if config.get('frame')!='World' or config.get('part_frames')!='measured_MDH_frames_0_to_6_mm':
        raise ValueError('World and actual MDH part frames required')
    if config.get('hand_pose_targets')!=[8000,8000,8000,8000,65535,65535]:
        raise ValueError('Only fixed historical PREPARE shape supported')
    missing=[];parts=[]
    for item in config.get('parts') or []:
        if not isinstance(item,dict) or not isinstance(item.get('id'),str) or not item['id'].strip():
            raise ValueError('Part id required')
        if type(item.get('mdh_frame')) is not int or not 0<=item['mdh_frame']<=6:
            raise ValueError('Actual MDH attachment frame required')
        if item.get('component') not in REQUIRED_COMPONENTS:raise ValueError('Unsupported part component')
        if item.get('origin')!='installed_measured_geometry' or not isinstance(item.get('source'),str) or not item['source'].strip():
            raise ValueError('Installed measured source required; nominal URDF cannot become MDH geometry')
        low,high=box(item.get('local_min_mm'),item.get('local_max_mm'))
        allowance=number(item.get('geometry_allowance_mm'),'geometry allowance')
        if allowance<0:raise ValueError('Geometry allowance cannot be negative')
        parts.append(dict(item,local_min_mm=low,local_max_mm=high,geometry_allowance_mm=allowance))
    if len({p['id'] for p in parts})!=len(parts):raise ValueError('Duplicate part ids')
    components={p['component'] for p in parts}
    missing.extend('installed_geometry:'+c for c in REQUIRED_COMPONENTS if c not in components)
    obstacles=[]
    if config.get('obstacles') is None:
        missing.append('World_obstacle_geometry:workboard_battery_other_arm_body_and_environment')
    else:
        if not isinstance(config['obstacles'],list):raise ValueError('Obstacles must be a list or unknown')
        if not isinstance(config.get('obstacles_source'),str) or not config['obstacles_source'].strip():
            raise ValueError('Explicit scene source required, including empty scene')
        for item in config['obstacles']:
            if not isinstance(item,dict) or not isinstance(item.get('id'),str) or not item['id'].strip():raise ValueError('Obstacle id required')
            low,high=box(item.get('world_min_mm'),item.get('world_max_mm'))
            obstacles.append(dict(item,world_min_mm=low,world_max_mm=high))
        if len({o['id'] for o in obstacles})!=len(obstacles):raise ValueError('Duplicate obstacles')
    clearance=config.get('clearance_mm')
    if clearance is None:missing.append('explicit_clearance_and_error_budget')
    else:
        clearance=number(clearance,'clearance')
        if clearance<=0:raise ValueError('Positive explicit clearance required')
    return parts,obstacles,clearance,missing


def prepare_sweep(raw,config):
    source=accepted_native_result(raw);parts,obstacles,clearance,missing=read_geometry(config)
    result=dict(schema_version=1,scope='supplied_rigid_parts_vs_supplied_World_boxes_on_affine_joint_cells',
        source_native_numerical_stage_reused=True,source_native_report_sha256=NATIVE_REPORT_SHA,
        hardware_writes=0,network_connection_requested=False,execution_ready=False,arm_motion_allowed=False,hand_motion_allowed=False,
        current_start_known=False,current_hand_pose_known=False,current_controller_configuration_verified=False,
        automatically_return_to_historical_anchor=False,reference_face='acrylic_workboard_top',reference_gap_mm=14,
        reference_gap_is_scene_clearance=False,table_world_z_derived=False,
        installed_geometry_physically_verified=False,scene_completeness_verified=False,
        controller_interpolation_verified=False,self_collision_verified=False,stop_release_verified=False,
        numerical_geometry_supplied=not missing,conditional_sweep_numerical_pass=None,missing=missing,route_cells=[],part_results=[],
        unknowns=['Installed mount/shape and cable geometry','World scene including workboard and battery',
                  'Self collision and other-arm/body constraints','Actual controller interpolation and tracking',
                  'Fresh current arm/hand/holding state and controller configuration','Physical stopping/release'],
        curve_assumption='Only piecewise affine joint configurations between recorded samples; not a claimed controller trajectory')
    n=source['native_assessment'];c=source['candidate']
    # Both route cell sets are retained; rigid PREPARE is an explicit assumption.
    for route_name,key in (('joint_candidate','joint_path'),('vertical_candidate','vertical_path')):
        rows=n[key]
        for index,(a,b) in enumerate(zip(rows,rows[1:])):
            qa,qb=a['joint_degrees'],b['joint_degrees']
            for q in (qa,qb):
                if min(reserves(q,c['ordinary_limits'])+reserves(q,c['drive_limits']))<3:
                    raise ValueError('Source joint cell reserve below3degrees')
            # Affine endpoint ranges prove this curve's joint limits, not its Cartesian path.
            result['route_cells'].append(dict(route=route_name,index=index,joint_start_degrees=qa,joint_end_degrees=qb,
                affine_joint_reserve_pass=True,controller_joint_reserve_verified=False,physical_sweep_result=None))
            if not missing:
                for part in parts:
                    sweep=sweep_cell(qa,qb,part,obstacles,clearance)
                    sweep.update(route=route_name,route_cell_index=index)
                    result['part_results'].append(sweep)
    if not missing:
        result['conditional_sweep_numerical_pass']=all(p['conditional_disjoint_pass'] for p in result['part_results'])
    result['outcome']='installed_geometry_pending_no_hardware_connection' if missing else 'conditional_sweep_calculated_no_execution'
    return result

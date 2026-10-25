"""Offline localization -> taught noncontact Arm_Tip mapping -> checked route.

The reference feature remains a visible surface point, not a grasp center.
No network, SDK, execution permit or actuator is provided by this module.
"""
import copy
import math
from .real_grasp import vector, rotation, transform, measured, workspace_errors, segment_intersects_box
from .pregrasp import build_pregrasp_plan


def select_surface(document, index):
    if (document.get('schema_version') != 1 or document.get('serial') != '344322075008'
            or document.get('world_frame') != 'World'
            or document.get('camera_mount_unchanged_user_confirmed') is not True
            or document.get('world_point_kind') != 'visible_surface_not_tcp_target'
            or not isinstance(document.get('world_transform_sha256'),str)
            or len(document['world_transform_sha256']) != 64):
        raise ValueError('Expected fixed head D435 World surface capture')
    rows = document.get('detections')
    if not isinstance(rows,list) or type(index) is not int or not 0 <= index < len(rows):
        raise ValueError('Explicit target index required')
    selected = rows[index]
    if selected.get('depth_valid') is not True or selected.get('visual_confirmed') is not True:
        raise ValueError('Selected battery needs valid depth and temporal visual confirmation')
    point = vector(selected.get('position_world_mm'),'visible surface')
    optical = vector(selected.get('position_optical_m'),'optical surface')
    matrix = document.get('world_from_camera')
    if not isinstance(matrix,list) or len(matrix) != 4:
        raise ValueError('World matrix missing')
    if any(not isinstance(row,list) or len(row)!=4 for row in matrix):
        raise ValueError('World matrix shape')
    for row in matrix:
        for v in row:
            if type(v) not in (int,float) or not math.isfinite(v): raise ValueError('Invalid matrix value')
    if any(abs(a-b)>1e-9 for a,b in zip(matrix[3],[0,0,0,1])): raise ValueError('World matrix last row')
    r = [row[:3] for row in matrix[:3]]
    if any(abs(sum(r[k][i]*r[k][j] for k in range(3))-(1 if i==j else 0))>1e-6
           for i in range(3) for j in range(3)):
        raise ValueError('World rotation is not orthonormal')
    det = r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0])
    if abs(det-1)>1e-6: raise ValueError('World matrix reflection')
    converted = [1000*(sum(a*b for a,b in zip(row[:3],optical))+row[3]) for row in matrix[:3]]
    if any(abs(a-b)>.001 for a,b in zip(converted,point)):
        raise ValueError('World point does not match recorded optical point/transform')
    if type(selected.get('class_id')) is not int or not isinstance(selected.get('class_name'),str):
        raise ValueError('Target class missing')
    return dict(index=index,class_id=selected['class_id'],class_name=selected['class_name'],
        surface_world_mm=point,point_kind='visible_surface_not_grasp_center',
        serial=document['serial'],world_transform_sha256=document['world_transform_sha256'])


def derive_reference(target, upper, lower):
    for point in (upper,lower):
        if point.get('accepted') is not True or point.get('hand_target') != [8000,8000,8000,8000,65535,65535]:
            raise ValueError('Both taught points must pass prepared empty-hand brackets')
        if point.get('clearance_source') != 'operator_measured_minimum_whole_hand_to_scene_mm':
            raise ValueError('Whole-hand clearance source missing')
        if type(point.get('clearance_mm')) not in (int,float) or not math.isfinite(point['clearance_mm']):
            raise ValueError('Measured clearance required')
    if upper['clearance_mm'] < 50 or lower['clearance_mm'] < 10:
        raise ValueError('This capture protocol requires upper>=50mm and lower>=10mm scene clearance')
    a,b = vector(upper.get('tool_world_mm'),'upper TCP'), vector(lower.get('tool_world_mm'),'lower TCP')
    ra,rb = vector(upper.get('tool_rpy_rad'),'upper RPY'), vector(lower.get('tool_rpy_rad'),'lower RPY')
    if math.dist(a[:2],b[:2])>.5 or b[2]>=a[2]:
        raise ValueError('Taught lower must be lower World Z and same X/Y within0.5mm')
    if any(abs(math.atan2(math.sin(x-y),math.cos(x-y)))>.005 for x,y in zip(ra,rb)):
        raise ValueError('Fixed wrist changed between taught points')
    feature = vector(target.get('surface_world_mm'),'reference surface')
    return dict(schema_version=1,reference_target=copy.deepcopy(target),
        surface_to_pregrasp_tip_world_mm=[x-y for x,y in zip(b,feature)],
        fixed_tool_rpy_rad=rb,upper_tool_world_mm=a,lower_tool_world_mm=b,
        scope='same inert battery, same orientation, same visible feature and unchanged camera',
        mapping_is_grasp_center=False,automatic_execution_allowed=False,
        endpoint_capture_does_not_certify_swept_path=True)


def build_localized_plan(capture, geometry, target=None):
    result = dict(schema_version=1,planning_ready=False,execution_ready=False,hardware_writes=0,
        arm_motion_allowed=False,blockers=[],waypoints=[],scope='offline localized noncontact pregrasp')
    try:
        if capture.get('schema_version') != 1 or capture.get('diagnostic_pass') is not True:
            raise ValueError('Passed combined target/endpoint capture required')
        mapping = derive_reference(capture['target'],capture['upper'],capture['lower'])
        chosen = copy.deepcopy(target or capture['target'])
        ref = mapping['reference_target']
        for key in ('class_id','class_name','serial','world_transform_sha256','point_kind'):
            if chosen.get(key) != ref.get(key): raise ValueError('New target differs in '+key)
        if target is not None and chosen.get('same_inert_battery_and_orientation_confirmed') is not True:
            raise ValueError('New feature requires same battery/orientation confirmation')
        surface = vector(chosen.get('surface_world_mm'),'selected surface')
        tip = [a+b for a,b in zip(surface,mapping['surface_to_pregrasp_tip_world_mm'])]
        current = mapping['upper_tool_world_mm']
        aligned = [tip[0],tip[1],current[2]]
        cfg = copy.deepcopy(geometry)
        if cfg.get('battery_box') is not None:
            battery = measured(cfg['battery_box'],'reference battery box')
            lo,hi = vector(battery.get('min_mm'),'battery min'),vector(battery.get('max_mm'),'battery max')
            if any(a>=b for a,b in zip(lo,hi)):
                raise ValueError('Reference battery box reversed')
            original_surface = vector(ref['surface_world_mm'],'original surface')
            if any(not a-3 <= p <= b+3 for p,a,b in zip(original_surface,lo,hi)):
                raise ValueError('Reference surface is outside measured reference battery box')
            delta = [a-b for a,b in zip(surface,original_surface)]
            cfg['battery_box']['min_mm'] = [a+b for a,b in zip(lo,delta)]
            cfg['battery_box']['max_mm'] = [a+b for a,b in zip(hi,delta)]
        cfg['target_tip'] = dict(verified=True,source='taught pregrasp offset + explicit World surface',world_mm=tip)
        start = dict(tool_world_mm=aligned,tool_rpy_rad=mapping['fixed_tool_rpy_rad'],source='captured upper, target X/Y alignment candidate')
        vertical = build_pregrasp_plan(start,cfg)
        result.update(mapping=mapping,selected_target=chosen,vertical_plan=vertical,
            evaluated_battery_box=cfg.get('battery_box'),
            target_tip_world_mm=tip,drop_mm=current[2]-tip[2],
            current_pose_verified=False,target_is_current_at_execution=False,
            execution_gaps=vertical['execution_gaps'],hand_transition_location='accepted hover before any arm path')
        result['execution_gaps'] = result['execution_gaps']+[
            'Fresh target and unchanged camera/feature mapping; absolute vision accuracy not certified',
            'Recorded two endpoints are not a certified arm/hand/cable trajectory']
        result['blockers'] = vertical['blockers'][:]
        result['waypoints'] = [dict(phase='captured_upper',tool_world_mm=current,tool_rpy_rad=mapping['fixed_tool_rpy_rad']),
            dict(phase='aligned_hover',tool_world_mm=aligned,tool_rpy_rad=mapping['fixed_tool_rpy_rad']),
            dict(phase='noncontact_pregrasp',tool_world_mm=tip,tool_rpy_rad=mapping['fixed_tool_rpy_rad'])]
        if vertical['planning_ready']:
            env = measured(cfg['hand_envelope'],'hand envelope')
            bands = measured(cfg['workspace'],'workspace')['bands']
            plane = measured(cfg['support_plane'],'support plane')['world_z_mm']
            clearance = cfg['clearance_mm']
            boxes = measured(cfg['obstacles'],'obstacles')['boxes']+[measured(cfg['battery_box'],'battery box')]
            result['blockers'] += workspace_errors(current,aligned,bands,mapping['fixed_tool_rpy_rad'])
            for sphere in env['spheres']:
                offset = transform(rotation(mapping['fixed_tool_rpy_rad']),vector(sphere['tool_center_mm'],'sphere'))
                ca,cb = [[x+y for x,y in zip(p,offset)] for p in (current,aligned)]
                padding = sphere['radius_mm']+clearance
                if min(ca[2],cb[2])-sphere['radius_mm'] < plane+clearance:
                    result['blockers'].append('Horizontal hand/table clearance: '+str(sphere['part']))
                for box in boxes:
                    if segment_intersects_box(ca,cb,[v-padding for v in box['min_mm']],[v+padding for v in box['max_mm']]):
                        result['blockers'].append('Horizontal hand/battery/obstacle sweep: '+str(sphere['part']))
        result['blockers'] = sorted(set(result['blockers']))
        result['planning_ready'] = vertical['planning_ready'] and not result['blockers']
        return result
    except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
        result['blockers'].append(str(exc))
        return result

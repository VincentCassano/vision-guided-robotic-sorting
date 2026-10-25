"""Live ray/plane task mapping for the same upright shell in equal-height recesses.

The reference is already recorded. Its point is not refitted. The plane normal
is an estimate from all recorded reference cylinder axes, not a surveyed board
normal or a replacement hand-eye calibration. No drivers or motion authority.
"""
import copy
import math
import numpy as np
from known_cylinder_capture import summarize
from visual_task_anchor import saved_target, target_candidate


def make_plane(anchor, reference_observation, world_from_camera, confirmed):
    if confirmed is not True:
        raise ValueError('Same upright measured shell, equal-height fully seated recesses required')
    summary=reference_observation.get('burst_summary')
    frames=reference_observation.get('burst_frames',[])
    if (anchor.get('reference_burst')!=summary or summary.get('burst_qualified') is not True
            or len(frames)<8 or len(frames)!=summary['frame_count']):
        raise ValueError('Unchanged complete accepted reference observation required')
    transform=np.asarray(world_from_camera,float)
    if transform.shape!=(4,4) or not np.isfinite(transform).all():
        raise ValueError('Frozen world_from_camera required')
    axes=np.asarray([f['axis_from_top_into_body_world'] for f in frames],float)
    if axes.shape!=(len(frames),3) or not np.isfinite(axes).all():
        raise ValueError('All original reference cylinder directions required')
    norms=np.linalg.norm(axes,axis=1)
    if np.any(norms<1e-9):raise ValueError('Invalid reference cylinder direction')
    axes=axes/norms[:,None]
    down=np.mean(axes,axis=0)
    if np.linalg.norm(down)<.9:raise ValueError('Reference cylinder directions inconsistent')
    down/=np.linalg.norm(down)
    spread=np.degrees(np.arccos(np.clip(axes@down,-1.,1.)))
    point=np.asarray(anchor['reference_top_center_world_m'],float)
    if point.shape!=(3,) or not np.isfinite(point).all():raise ValueError('Invalid reference point')
    return dict(schema_version=1,method='live_camera_ray_intersection_with_equal_top_height_plane',
        plane_point_world_m=point.tolist(),axis_from_top_into_body_world=down.tolist(),
        plane_normal_source='arithmetic_mean_of_all_original_reference_cylinder_axes',
        normal_reference_frame_count=len(frames),normal_reference_spread_max_degrees=float(max(spread)),
        plane_orientation_independently_measured=False,
        original_reference_observation_sha256=anchor['reference_observation_sha256'],
        original_camera_context=copy.deepcopy(anchor['camera_context']),
        world_from_camera=transform.tolist(),same_upright_shell_equal_seated_top_plane_confirmed=True,
        old_reference_point_refitted=False,slot_lookup=False,grid_snap=False,
        absolute_position_accepted=False,motion_authorized=False,execution_ready=False)


def project_observation(frame, plane):
    if frame.get('geometry_candidate_available') is not True:
        raise ValueError('Original per-frame rim/body image gates must pass')
    chosen=frame['geometry']['selected_candidate']
    camera_point=np.asarray(chosen['top_center_camera_m'],float)
    transform=np.asarray(plane['world_from_camera'],float)
    normal=np.asarray(plane['axis_from_top_into_body_world'],float)
    origin=transform[:3,3]
    direction=transform[:3,:3]@camera_point
    denominator=float(normal@direction)
    if not np.isfinite(direction).all() or abs(denominator)<1e-9:
        raise ValueError('Camera ray parallel to task plane')
    scale=float(normal@(np.asarray(plane['plane_point_world_m'])-origin)/denominator)
    if not math.isfinite(scale) or scale<=0 or not .1<=scale*camera_point[2]<=2.:
        raise ValueError('Plane intersection behind camera or outside original0.1-2m Z range')
    point=origin+scale*direction
    raw=np.asarray(frame['top_center_world_m'],float)
    body=point+.5*float(frame['geometry']['total_height_mm'])/1000*normal
    return dict(top_center_world_m=point.tolist(),geometric_body_center_world_m=body.tolist(),
        ray_scale=scale,raw_free3D_to_plane_correction_mm=float(np.linalg.norm(point-raw)*1000),
        scope='same_upright_shell_equal_seated_top_plane_only',
        height_inferred_from_task_plane=True,independent_depth_measured=False,
        raw_free3D_position_replaced_in_source_record=False,motion_authorized=False,execution_ready=False)


def plane_summary(frames, elapsed):
    # Construct a separately labelled method input. Original frames/3D values
    # remain untouched and are archived with their separate original verdict.
    method_frames=[]
    for frame in frames:
        row=copy.deepcopy(frame)
        projection=frame.get('equal_height_plane_projection')
        if projection is None:
            row['geometry_candidate_available']=False
        else:
            for key in ('top_center_world_m','geometric_body_center_world_m'):
                row[key]=projection[key][:]
            # Plane support is not the old fitted3D cloud support. Do not copy
            # an old per-frame FIT verdict onto a new plane-constrained point.
            row['top_center_inside_FIT_support']=False
            row['geometric_body_center_inside_FIT_support']=False
        method_frames.append(row)
    result=summarize(method_frames,elapsed)
    result.update(scope='equal_height_plane_live_repeatability_not_free3D_or_absolute_accuracy',
        method='live_camera_ray_intersection_with_equal_top_height_plane',
        original_free3D_gate_pass_not_implied=True,height_inferred_from_task_plane=True,
        FIT_support_for_projected_points_not_evaluated=True)
    return result


def task_target(point, anchor, context, plane, summary=None):
    result=(target_candidate(point,anchor,context) if summary is None
            else saved_target(summary,anchor,context))
    result.update(scope='continuous_live_ray_plane_to_fixed_wrist_pregrasp_TCP_candidate',
        method='live_camera_ray_intersection_with_equal_top_height_plane',
        equal_height_plane=copy.deepcopy(plane),
        independent_depth_measured=False,height_inferred_from_task_plane=True,
        same_upright_shell_equal_height_recesses_only=True,
        free3D_accuracy_or_gate_pass_not_implied=True,motion_authorized=False,execution_ready=False)
    return result

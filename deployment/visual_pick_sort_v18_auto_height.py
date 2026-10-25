"""Upright-circle height evidence, keeping the old unconstrained result intact.

The shell is upright on this board. Solve its translation from all observed rim
pixels with that common axis, instead of refitting its axis in every frame.
Only the height classifier uses this new measurement; accepted ray/plane XY,
the measured two support levels and their existing thresholds remain unchanged.
This method still requires field acceptance and is not a free3D certificate.
"""
import copy
import numpy as np
from scipy.optimize import least_squares

from visual_pick_sort_v17_auto_height import infer_support as original_support_rules
from visual_pick_sort_v17_auto_height import observed_rise
from visual_equal_height_plane import project_observation,plane_summary
from visual_pick_sort_v16_plane import support_plane


def upright_measurement(frame,base):
    if frame.get('geometry_candidate_available') is not True:
        raise ValueError('Original image geometry is unavailable')
    geometry=frame['geometry']
    pixels=np.asarray(geometry['contour_detail']['contour_px'],float)
    k=np.asarray(base['original_camera_context']['K'],float)
    transform=np.asarray(base['world_from_camera'],float)
    normal=np.asarray(base['axis_from_top_into_body_world'],float)
    initial=np.asarray(geometry['selected_candidate']['top_center_camera_m'],float)
    radius=float(geometry['diameter_mm'])/2000.
    if (pixels.ndim!=2 or pixels.shape[1]!=2 or len(pixels)<20
            or k.shape!=(3,3) or transform.shape!=(4,4) or normal.shape!=(3,)
            or initial.shape!=(3,) or not .1<initial[2]<2. or not 0<radius<.1
            or not all(np.isfinite(v).all() for v in (pixels,k,transform,normal,initial))):
        raise ValueError('Complete original rim, camera, radius and common board axis required')
    normal=transform[:3,:3].T@normal
    length=float(np.linalg.norm(normal))
    if length<1e-9:raise ValueError('Common upright axis is degenerate')
    normal/=length
    inverse=np.linalg.inv(k);rays=np.c_[pixels,np.ones(len(pixels))]

    def unpack(parameters):
        return np.array([parameters[0],parameters[1],np.exp(parameters[2])])

    def residual(parameters):
        point=unpack(parameters);distance=float(normal@point)
        cone=(distance**2*np.eye(3)
            -distance*(np.outer(normal,point)+np.outer(point,normal))
            +(float(point@point)-radius**2)*np.outer(normal,normal))
        conic=inverse.T@cone@inverse
        magnitude=float(np.linalg.norm(conic))
        if magnitude<1e-20:raise ValueError('Circle projection is degenerate')
        conic/=magnitude
        gradient=2*(rays@conic.T)[:,:2]
        denominator=np.linalg.norm(gradient,axis=1)
        if np.any(denominator<1e-12):raise ValueError('Rim gradient is degenerate')
        return np.einsum('ij,ij->i',rays@conic,rays)/denominator

    solved=least_squares(residual,[initial[0],initial[1],np.log(initial[2])],
        bounds=([-2.,-2.,np.log(.1)],[2.,2.,np.log(2.)]),loss='linear',
        max_nfev=150,ftol=1e-10,xtol=1e-10,gtol=1e-10)
    errors=np.abs(residual(solved.x));point=unpack(solved.x)
    if not solved.success or not np.isfinite(point).all():
        raise ValueError('Common-axis circle translation did not converge')
    singular=np.linalg.svd(solved.jac,compute_uv=False)
    if len(singular)!=3 or singular[-1]<=singular[0]*np.finfo(float).eps*max(solved.jac.shape):
        raise ValueError('Observed rim does not constrain all three translation coordinates')
    error=float(np.percentile(errors,90))
    if error>2.:
        raise ValueError('Common-axis observed rim residual %.3fpx exceeds original2px'%error)
    world=transform[:3,:3]@point+transform[:3,3]
    measured=dict(geometry_candidate_available=True,top_center_world_m=world.tolist())
    return dict(method='all_observed_rim_pixels_common_upright_axis_known_radius_translation',
        qualified=True,top_center_camera_m=point.tolist(),top_center_world_m=world.tolist(),
        observed_rise_mm=observed_rise(measured,base),radius_mm=radius*1000,
        common_axis_camera=normal.tolist(),common_axis_world=base['axis_from_top_into_body_world'][:],
        common_axis_source='unchanged_historical_board_axis_estimate_not_surveyed_normal',
        observed_rim_pixels=len(pixels),observed_rim_residual_p90_px=error,
        observed_rim_residual_max_px=float(max(errors)),original_rim_limit_px=2.,
        solver_nfev=int(solved.nfev),translation_jacobian_singular_values=singular.tolist(),
        all_rim_pixels_used=True,robust_loss_or_pixel_rejection=False,
        per_frame_axis_refitted=False,coordinate_smoothing=False,
        absolute_accuracy_accepted=False,execution_ready=False)


def height_evidence(frames):
    # Retain inputs, not just a claimed new height. The motion process repeats
    # this measurement from these original contours before accepting a target.
    return [dict(frame_number=f['frame_number'],timestamp_ms=f['timestamp_ms'],
        geometry_candidate_available=f.get('geometry_candidate_available') is True,
        geometry=copy.deepcopy(f.get('geometry')),
        top_center_world_m=copy.deepcopy(f.get('top_center_world_m'))) for f in frames]


def infer_support(evidence,base,elapsed):
    converted=[];measurements=[];errors=[]
    for i,row in enumerate(evidence):
        item=copy.deepcopy(row)
        try:
            measurement=upright_measurement(row,base)
            item['top_center_world_m']=measurement['top_center_world_m'][:]
            measurements.append(dict(frame_index=i+1,measurement=measurement))
        except (ValueError,TypeError,KeyError,OverflowError,FloatingPointError,np.linalg.LinAlgError) as exc:
            item['geometry_candidate_available']=False
            errors.append(dict(frame_index=i+1,error=str(exc)))
        converted.append(item)
    decision=original_support_rules(converted,base,elapsed)
    raw=original_support_rules(evidence,base,elapsed)
    decision.update(method='common_upright_axis_RGB_circle_translation_to_measured_board_support',
        upright_circle_measurements=measurements,upright_circle_measurement_errors=errors,
        original_unconstrained_height_decision=raw,
        height_estimator_changed=True,support_thresholds_changed=False,
        fixed_axis_assumes_shell_upright=True,new_height_method_field_accepted=False)
    if errors:
        decision['reject_reasons']=[r for r in decision['reject_reasons']
            if r!='auto_height_missing_original_frame_geometry']
        decision['reject_reasons'].append('upright_circle_measurement_unavailable')
        decision.update(qualified=False,selected_surface=None)
    return decision


def automatic_projection(frames,base,elapsed):
    evidence=height_evidence(frames);decision=infer_support(evidence,base,elapsed)
    method_frames=copy.deepcopy(frames);selected=None;rise=None
    digest=decision['support_height_model_sha256']
    raw_rejected=[i+1 for i,f in enumerate(frames) if f.get('geometry_candidate_available') is not True]
    projection_errors=[]
    if decision['qualified']:
        selected,rise,digest=support_plane(base,decision['selected_surface'])
        for i,frame in enumerate(method_frames):
            try:
                frame['equal_height_plane_projection']=project_observation(frame,selected)
                frame['equal_height_plane_projection'].update(
                    scope='upright_measured_shell_automatically_selected_known_board_support',
                    automatic_support_selection=True,support_surface=decision['selected_surface'])
            except (ValueError,KeyError,TypeError,FloatingPointError,OverflowError,np.linalg.LinAlgError) as exc:
                frame.pop('equal_height_plane_projection',None)
                frame['auto_height_projection_error']=str(exc)
                projection_errors.append(dict(frame_index=i+1,error=str(exc)))
        summary=plane_summary(method_frames,elapsed)
        # plane_summary historically uses a missing-geometry flag for a
        # missing projection. Keep those two failures separate here.
        summary['reject_reasons']=[r for r in summary['reject_reasons']
            if r!='one_or_more_frames_without_selected_geometry']
        if projection_errors:summary['reject_reasons'].append('one_or_more_plane_projections_unavailable')
    else:
        # No plane exists yet. Do not run a plane summarizer that deliberately
        # marks every unprojected frame as if its raw geometry were missing.
        summary=dict(frame_count=len(frames),elapsed_seconds=elapsed,
            all_frames_retained=True,frame_quality_pass=False,burst_qualified=False,
            top_center_world_m=None,geometric_body_center_world_m=None,
            reject_reasons=[],plane_projection_attempted=False)
    if raw_rejected:summary['reject_reasons'].append('one_or_more_frames_without_selected_geometry')
    for reason in decision['reject_reasons']:
        if reason not in summary['reject_reasons']:summary['reject_reasons'].append(reason)
    if summary['reject_reasons']:summary.update(frame_quality_pass=False,burst_qualified=False)
    summary.update(automatic_support_selection=True,automatic_height_decision=decision,
        raw_geometry_rejected_frame_indices=raw_rejected,plane_projection_errors=projection_errors,
        unknown_height_or_ambiguity_falls_back_to_recess=False,
        scope='upright_axis_automatic_two_measured_board_supports_not_free3D_acceptance')
    return summary,selected,rise,digest,decision,evidence,method_frames

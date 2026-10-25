"""Compare both measured support planes against every original observed rim.

The unconstrained height result and its 3mm failure remain separate. This
discrete model comparison requires all frames to agree, bounded height-domain
evidence, original2px rim quality and3mm current Cartesian repeatability.
Different model centers are not independent metrological error references.
No frame/pixel deletion, coordinate smoothing, slot lookup or robot access.
"""
import copy
import math
import numpy as np
from scipy.optimize import least_squares

from known_cylinder_capture import MIN_FRAMES,MIN_SECONDS,MAX_REPEATABILITY_MM
from visual_pick_sort_v18_auto_height import (height_evidence,observed_rise,
    upright_measurement,infer_support as original_support)
from visual_pick_sort_v16_plane import support_model,support_plane
from visual_equal_height_plane import project_observation,plane_summary


METHOD='every_original_rim_two_measured_support_plane_hypotheses_v22_fix4'


def fit_support(row,plane):
    if row.get('geometry_candidate_available') is not True:
        raise ValueError('Original rim/body image geometry unavailable')
    geometry=row['geometry'];pixels=np.asarray(geometry['contour_detail']['contour_px'],float)
    k=np.asarray(plane['original_camera_context']['K'],float)
    transform=np.asarray(plane['world_from_camera'],float)
    normal=np.asarray(plane['axis_from_top_into_body_world'],float)
    origin=np.asarray(plane['plane_point_world_m'],float)
    initial=np.asarray(row['top_center_world_m'],float)
    radius=float(geometry['diameter_mm'])/2000.
    if (pixels.ndim!=2 or pixels.shape[1]!=2 or len(pixels)<20
            or k.shape!=(3,3) or transform.shape!=(4,4)
            or any(v.shape!=(3,) for v in (normal,origin,initial))
            or not 0<radius<.1 or abs(float(normal[2]))<1e-9
            or not all(np.isfinite(v).all() for v in (pixels,k,transform,normal,origin,initial))):
        raise ValueError('Complete finite rim/camera/known-radius/plane evidence required')
    normal=normal/np.linalg.norm(normal)
    camera_normal=transform[:3,:3].T@normal
    camera_normal/=np.linalg.norm(camera_normal)
    inverse=np.linalg.inv(k);rays=np.c_[pixels,np.ones(len(pixels))]

    def unpack(xy):
        world=np.array([xy[0],xy[1],origin[2]-(normal[0]*(xy[0]-origin[0])
            +normal[1]*(xy[1]-origin[1]))/normal[2]])
        return world,transform[:3,:3].T@(world-transform[:3,3])

    def residual(xy):
        world,point=unpack(xy)
        distance=float(camera_normal@point)
        cone=(distance**2*np.eye(3)
            -distance*(np.outer(camera_normal,point)+np.outer(point,camera_normal))
            +(float(point@point)-radius**2)*np.outer(camera_normal,camera_normal))
        conic=inverse.T@cone@inverse;magnitude=float(np.linalg.norm(conic))
        if magnitude<1e-20:raise ValueError('Known-support circle projection is degenerate')
        conic/=magnitude;gradient=2*(rays@conic.T)[:,:2]
        denominator=np.linalg.norm(gradient,axis=1)
        if np.any(denominator<1e-12):raise ValueError('Observed-rim gradient is degenerate')
        return np.einsum('ij,ij->i',rays@conic,rays)/denominator

    solved=least_squares(residual,initial[:2],bounds=([-2.,-2.],[2.,2.]),
        loss='linear',max_nfev=150,ftol=1e-10,xtol=1e-10,gtol=1e-10)
    world,point=unpack(solved.x);errors=residual(solved.x)
    singular=np.linalg.svd(solved.jac,compute_uv=False)
    if (not solved.success or not all(np.isfinite(v).all() for v in (world,point,errors,singular))
            or not .1<point[2]<2. or len(singular)!=2
            or singular[-1]<=singular[0]*np.finfo(float).eps*max(solved.jac.shape)):
        raise ValueError('Observed rim does not constrain both support-plane coordinates')
    return dict(top_center_world_m=world.tolist(),top_center_camera_m=point.tolist(),
        residual_rms_px=float(np.sqrt(np.mean(errors**2))),
        residual_p90_px=float(np.percentile(np.abs(errors),90)),
        residual_max_px=float(max(np.abs(errors))),observed_rim_pixels=len(pixels),
        all_rim_pixels_used=True,robust_loss_or_pixel_rejection=False,
        support_plane_jacobian_singular_values=singular.tolist(),solver_nfev=int(solved.nfev),
        axis_and_radius_refitted=False)


def infer_support(evidence,base,elapsed):
    old=original_support(evidence,base,elapsed)
    model,digest=support_model();gap=float(model['board_top_above_recess_bottom_mm'])
    result=dict(method=METHOD,qualified=False,selected_surface=None,
        frame_count=len(evidence),elapsed_seconds=elapsed,measured_support_levels_mm=[0.,gap],
        support_height_model_sha256=digest,operator_height_selection=False,
        arbitrary_unknown_height_supported=False,all_frames_required=True,frames_dropped=False,
        all_original_rim_pixels_used=True,coordinate_smoothing=False,
        original_free3D_gate_pass_not_implied=True,original_height_decision=old,
        continuous_height_repeatability_is_diagnostic=True,
        original_continuous_height_repeatability_limit_mm=MAX_REPEATABILITY_MM,
        final_cartesian_repeatability_limit_mm=MAX_REPEATABILITY_MM,
        original_rim_limit_px=2.,hypotheses=[],reject_reasons=[],
        absolute_model_center_difference_is_diagnostic=True,
        absolute_continuous_height_residual_is_diagnostic=True,
        support_policy_authorization='explicit_user_20261024_all_frame_two_plane_agreement',
        model_change_consistency_reference='first_original_frame_of_this_burst',
        model_change_consistency_limit_mm=MAX_REPEATABILITY_MM,
        new_support_method_field_accepted=False)
    # Preserve these old numerical values as diagnostics, not a claimed pass.
    for key in ('observed_rise_by_frame_mm','observed_rise_median_mm','height_repeatability_max_mm',
            'nearest_support_residual_mm','per_frame_support_votes','nearest_support'):
        if key in old:result[key]=copy.deepcopy(old[key])
    reasons=result['reject_reasons']
    if len(evidence)<MIN_FRAMES:reasons.append('support_fewer_than8_frames')
    if elapsed is None or not math.isfinite(elapsed) or elapsed<MIN_SECONDS:
        reasons.append('support_observation_below2s')
    if old.get('height_errors') or old.get('upright_circle_measurement_errors'):
        reasons.append('support_missing_original_geometry_or_upright_measurement')
    if not evidence:return result
    numbers=[f['frame_number'] for f in evidence];stamps=[f['timestamp_ms'] for f in evidence]
    if any(b<=a for a,b in zip(numbers,numbers[1:])) or any(b<=a for a,b in zip(stamps,stamps[1:])):
        reasons.append('support_frames_not_strictly_fresh')
    if (stamps[-1]-stamps[0])/1000.<1.:reasons.append('support_camera_span_below1s')
    votes=old.get('per_frame_support_votes',[]);winner=old.get('nearest_support')
    median=old.get('observed_rise_median_mm');spread=old.get('height_repeatability_max_mm')
    if (len(votes)!=len(evidence) or winner not in ('recess','board_top')
            or any(v!=winner for v in votes)):
        reasons.append('support_continuous_evidence_disagrees_on_two_levels')
    if median is None or spread is None or abs(median-gap/2.)<=spread:
        reasons.append('support_height_envelope_reaches_level_boundary')
    # Explicitly authorized by the user on 2026-10-24 after seeing the
    # 6.6--7.4mm residual and the possible wrong-height/collision risk.
    # Preserve the rejected free-height result; known-plane selection still
    # requires every original frame's pixel AND height votes to agree below.
    result['original_absolute_height_domain_rejected']=(
        old.get('nearest_support_residual_mm',math.inf)>=gap/2.)
    selected_centers=[];errors=[];pixel_votes=[];ray_differences=[];difference_vectors=[]
    for i,row in enumerate(evidence):
        record=dict(frame_index=i+1,frame_number=row['frame_number'],fits={})
        try:
            for surface in ('recess','board_top'):
                plane,_,_=support_plane(base,surface)
                record['fits'][surface]=fit_support(row,plane)
            fits=record['fits'];a=fits['recess']['residual_rms_px'];b=fits['board_top']['residual_rms_px']
            numerical_tie=64*np.finfo(float).eps*max(1.,a,b)
            selected=None if abs(a-b)<=numerical_tie else ('recess' if a<b else 'board_top')
            record.update(selected_surface=selected,score_gap_px=abs(a-b),
                numerical_tie_tolerance_px=float(numerical_tie))
            pixel_votes.append(selected)
            if selected is None:reasons.append('support_pixel_hypotheses_indistinguishable')
            elif selected!=winner:reasons.append('support_pixel_and_height_evidence_disagree')
            else:
                fitted=fits[selected]
                if fitted['residual_p90_px']>2.:reasons.append('support_selected_rim_residual_exceeds_original2px')
                plane,_,_=support_plane(base,selected);projected=project_observation(row,plane)
                difference_vector=(np.asarray(projected['top_center_world_m'])
                    -np.asarray(fitted['top_center_world_m']))*1000.
                difference=float(np.linalg.norm(difference_vector))
                record['current_ray_plane_vs_rim_fit_mm']=difference
                record['current_ray_plane_minus_rim_fit_vector_mm']=difference_vector.tolist()
                ray_differences.append(difference);selected_centers.append(fitted['top_center_world_m'])
                difference_vectors.append(difference_vector)
        except (ValueError,TypeError,KeyError,OverflowError,FloatingPointError,np.linalg.LinAlgError) as exc:
            record['error']=str(exc);errors.append(dict(frame_index=i+1,error=str(exc)))
        result['hypotheses'].append(record)
    result.update(pixel_support_votes=pixel_votes,hypothesis_errors=errors,
        ray_plane_vs_rim_fit_mm=ray_differences)
    if len(difference_vectors)==len(evidence):
        # The two approximations have different absolute centers. Check their
        # temporal changes against the first actual frame, without subtracting
        # this difference from any point or changing the old task binding.
        relative=[float(np.linalg.norm(v-difference_vectors[0])) for v in difference_vectors]
        result.update(model_change_consistency_by_frame_mm=relative,
            model_change_consistency_max_mm=max(relative),
            first_frame_model_difference_vector_mm=difference_vectors[0].tolist(),
            raw_coordinates_corrected_or_smoothed=False)
        if max(relative)>MAX_REPEATABILITY_MM:
            reasons.append('support_models_frame_change_disagreement_exceeds3mm')
    if errors:reasons.append('support_one_or_more_hypothesis_fits_unavailable')
    if len(selected_centers)!=len(evidence):reasons.append('support_not_all_frames_select_same_level')
    else:
        centers=np.asarray(selected_centers,float);center=np.median(centers,axis=0)
        maximum=float(max(np.linalg.norm(centers-center,axis=1))*1000.)
        result['support_constrained_center_repeatability_max_mm']=maximum
        if maximum>MAX_REPEATABILITY_MM:reasons.append('support_constrained_center_repeatability_exceeds3mm')
    result['reject_reasons']=list(dict.fromkeys(reasons))
    if not result['reject_reasons']:
        result.update(qualified=True,selected_surface=winner,
            selected_support_rise_mm=0. if winner=='recess' else gap)
    return result


def automatic_projection(frames,base,elapsed):
    evidence=height_evidence(frames);decision=infer_support(evidence,base,elapsed)
    method_frames=copy.deepcopy(frames);selected=None;rise=None
    digest=decision['support_height_model_sha256'];projection_errors=[]
    raw_rejected=[i+1 for i,f in enumerate(frames) if f.get('geometry_candidate_available') is not True]
    if decision['qualified']:
        selected,rise,digest=support_plane(base,decision['selected_surface'])
        for i,frame in enumerate(method_frames):
            try:
                frame['equal_height_plane_projection']=project_observation(frame,selected)
                frame['equal_height_plane_projection'].update(
                    scope='upright_measured_shell_pixel_selected_known_board_support_v22',
                    automatic_support_selection=True,support_surface=decision['selected_surface'])
            except (ValueError,TypeError,KeyError,OverflowError,FloatingPointError,np.linalg.LinAlgError) as exc:
                frame.pop('equal_height_plane_projection',None)
                projection_errors.append(dict(frame_index=i+1,error=str(exc)))
        summary=plane_summary(method_frames,elapsed)
        if projection_errors:summary['reject_reasons'].append('one_or_more_plane_projections_unavailable')
    else:
        summary=dict(frame_count=len(frames),elapsed_seconds=elapsed,all_frames_retained=True,
            frame_quality_pass=False,burst_qualified=False,top_center_world_m=None,
            geometric_body_center_world_m=None,reject_reasons=[],plane_projection_attempted=False)
    if raw_rejected:summary['reject_reasons'].append('one_or_more_frames_without_selected_geometry')
    summary['reject_reasons']=list(dict.fromkeys(summary['reject_reasons']+decision['reject_reasons']))
    if summary['reject_reasons']:summary.update(frame_quality_pass=False,burst_qualified=False)
    summary.update(automatic_support_selection=True,automatic_height_decision=decision,
        raw_geometry_rejected_frame_indices=raw_rejected,plane_projection_errors=projection_errors,
        unknown_height_or_ambiguity_falls_back_to_recess=False,
        scope='pixel_hypothesis_selection_of_two_measured_board_supports_not_free3D_acceptance')
    return summary,selected,rise,digest,decision,evidence,method_frames

"""Automatically identify either measured support on the current acrylic board.

Known-size cylinder RGB geometry provides independent height evidence. Every
retained frame must agree; neither a slot index nor an operator level is used.
This does not certify free3D accuracy or arbitrary unknown support heights.
"""
import copy
import math
import numpy as np

from known_cylinder_capture import MIN_FRAMES,MIN_SECONDS,MAX_REPEATABILITY_MM
from visual_equal_height_plane import project_observation,plane_summary
from visual_pick_sort_v16_plane import support_model,support_plane


def height_evidence(frames):
    return [dict(frame_number=f['frame_number'],timestamp_ms=f['timestamp_ms'],
        geometry_candidate_available=f.get('geometry_candidate_available') is True,
        top_center_world_m=copy.deepcopy(f.get('top_center_world_m'))) for f in frames]


def observed_rise(row,base):
    if row.get('geometry_candidate_available') is not True:
        raise ValueError('Original cylinder rim/body image geometry did not qualify')
    point=np.asarray(row['top_center_world_m'],float)
    origin=np.asarray(base['plane_point_world_m'],float)
    normal=np.asarray(base['axis_from_top_into_body_world'],float)
    if (any(v.shape!=(3,) or not np.isfinite(v).all() for v in (point,origin,normal))
            or abs(float(normal[2]))<1e-9):
        raise ValueError('Finite original3D height evidence and vertical plane normal required')
    # P lies on the reference plane translated by this World-Z displacement.
    value=float((normal@(point-origin))/normal[2]*1000.)
    if not math.isfinite(value):raise ValueError('Non-finite observed support height')
    return value


def infer_support(evidence,base,elapsed):
    model,digest=support_model();gap=float(model['board_top_above_recess_bottom_mm'])
    result=dict(method='all_frame_RGB_cylinder_height_to_measured_board_support',
        qualified=False,selected_surface=None,frame_count=len(evidence),elapsed_seconds=elapsed,
        measured_support_levels_mm=[0.,gap],support_height_model_sha256=digest,
        decision_boundary_mm=gap/2.,median_residual_limit_mm=gap/2.,
        residual_limit_source='half_the_measured_support_separation_not_absolute_accuracy',
        height_repeatability_limit_mm=MAX_REPEATABILITY_MM,
        all_frames_required=True,frames_dropped=False,coordinate_smoothing=False,
        operator_height_selection=False,arbitrary_unknown_height_supported=False,
        original_free3D_gate_pass_not_implied=True,reject_reasons=[])
    reasons=result['reject_reasons']
    if len(evidence)<MIN_FRAMES:reasons.append('auto_height_fewer_than8_frames')
    if elapsed is None or not math.isfinite(elapsed) or elapsed<MIN_SECONDS:
        reasons.append('auto_height_observation_below_original2s')
    heights=[];errors=[]
    for i,row in enumerate(evidence):
        try:heights.append(observed_rise(row,base))
        except (ValueError,TypeError,KeyError,OverflowError,FloatingPointError) as exc:
            errors.append(dict(frame_index=i+1,error=str(exc)))
    result['height_errors']=errors;result['observed_rise_by_frame_mm']=heights
    if errors:reasons.append('auto_height_missing_original_frame_geometry')
    if not evidence:return result
    numbers=[f['frame_number'] for f in evidence];stamps=[f['timestamp_ms'] for f in evidence]
    if any(b<=a for a,b in zip(numbers,numbers[1:])) or any(b<=a for a,b in zip(stamps,stamps[1:])):
        reasons.append('auto_height_frames_not_strictly_fresh')
    if (stamps[-1]-stamps[0])/1000.<1.:
        reasons.append('auto_height_camera_span_below_original1s')
    if errors or not heights:return result
    median=float(np.median(heights));spread=max(abs(h-median) for h in heights)
    votes=['recess' if abs(h)<abs(h-gap) else 'board_top' for h in heights]
    winner='recess' if abs(median)<abs(median-gap) else 'board_top'
    nearest=0. if winner=='recess' else gap;residual=abs(median-nearest)
    result.update(observed_rise_median_mm=median,height_repeatability_max_mm=spread,
        per_frame_support_votes=votes,nearest_support=winner,nearest_support_residual_mm=residual,
        classification_boundary_distance_mm=abs(median-gap/2.))
    if spread>MAX_REPEATABILITY_MM:reasons.append('auto_height_repeatability_exceeds_original3mm')
    if any(v!=winner for v in votes):reasons.append('auto_height_frames_disagree_on_support')
    if abs(median-gap/2.)<=spread:reasons.append('auto_height_uncertainty_reaches_support_boundary')
    if residual>=gap/2.:reasons.append('auto_height_outside_measured_support_hypotheses')
    if not reasons:result.update(qualified=True,selected_surface=winner,selected_support_rise_mm=nearest)
    return result


def automatic_projection(frames,base,elapsed):
    evidence=height_evidence(frames);decision=infer_support(evidence,base,elapsed)
    method_frames=copy.deepcopy(frames);selected=None;rise=None;digest=decision['support_height_model_sha256']
    if decision['qualified']:
        selected,rise,digest=support_plane(base,decision['selected_surface'])
        for frame in method_frames:
            try:
                frame['equal_height_plane_projection']=project_observation(frame,selected)
                frame['equal_height_plane_projection'].update(
                    scope='upright_measured_shell_automatically_selected_known_board_support',
                    automatic_support_selection=True,support_surface=decision['selected_surface'])
            except (ValueError,KeyError,TypeError,FloatingPointError,OverflowError,np.linalg.LinAlgError) as exc:
                frame.pop('equal_height_plane_projection',None)
                frame['auto_height_projection_error']=str(exc)
    else:
        for frame in method_frames:frame.pop('equal_height_plane_projection',None)
    summary=plane_summary(method_frames,elapsed)
    for reason in decision['reject_reasons']:
        if reason not in summary['reject_reasons']:summary['reject_reasons'].append(reason)
    if not decision['qualified']:summary.update(frame_quality_pass=False,burst_qualified=False)
    summary.update(automatic_support_selection=True,automatic_height_decision=decision,
        unknown_height_or_ambiguity_falls_back_to_recess=False,
        scope='automatic_two_measured_board_supports_not_arbitrary_height_or_free3D_acceptance')
    return summary,selected,rise,digest,decision,evidence,method_frames

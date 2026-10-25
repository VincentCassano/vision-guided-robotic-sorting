"""Live burst records and relative-distance arithmetic; no model fitting or drivers.

Eight observations over at least two seconds must all qualify. Rejected frames
stay in the record. Repeatability and a relative distance never certify absolute
position accuracy or authorize actuator motion.
"""
import math
import numpy as np


MIN_FRAMES = 8
MIN_SECONDS = 2.
TIMEOUT_SECONDS = 15.
MAX_REPEATABILITY_MM = 3.  # Development sampling rule, not absolute accuracy.


def begin(now):
    return dict(started_monotonic=now, frames=[], minimum_frames=MIN_FRAMES,
                minimum_seconds=MIN_SECONDS, timeout_seconds=TIMEOUT_SECONDS)


def observe(capture, result, now):
    capture['frames'].append(result)
    elapsed = now-capture['started_monotonic']
    count = len(capture['frames'])
    if elapsed < TIMEOUT_SECONDS and (elapsed < MIN_SECONDS or count < MIN_FRAMES):
        return None
    return summarize(capture['frames'], elapsed)


def summarize(frames, elapsed_seconds=None):
    """Arithmetic on retained observations, without refitting their geometry.

    Recovery cannot reconstruct a lost monotonic completion time. The frame
    quality result is kept separate from the original timed burst decision.
    """
    if not frames:
        raise ValueError('No retained frames')
    count = len(frames)
    reasons = []
    if count < MIN_FRAMES:
        reasons.append('fewer_than_8_frames_before_timeout')
    numbers = [f['frame_number'] for f in frames]
    stamps = [f['timestamp_ms'] for f in frames]
    if any(b <= a for a, b in zip(numbers, numbers[1:])) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        reasons.append('camera_frames_not_strictly_fresh_and_increasing')
    if (stamps[-1]-stamps[0])/1000 < 1.:
        reasons.append('camera_observation_span_below_1_second')
    rejected = [i+1 for i, f in enumerate(frames) if not f['geometry_candidate_available']]
    if rejected:
        reasons.append('one_or_more_frames_without_selected_geometry')
    summary = dict(scope='live_repeatability_only_not_absolute_accuracy',
        elapsed_seconds=elapsed_seconds, frame_count=count, rejected_frame_indices=rejected,
        all_frames_retained=True, maximum_repeatability_mm=MAX_REPEATABILITY_MM,
        repeatability_threshold_source='new_development_sampling_default',
        execution_ready=False)
    if not rejected:
        boxes = [next(r['detection']['bbox'] for r in f['candidate_rows']
                      if r['detection_id'] == f['selected_detection_id']) for f in frames]
        first = boxes[0]
        for box in boxes[1:]:
            intersection = max(0, min(first[0]+first[2], box[0]+box[2])-max(first[0], box[0]))*max(
                0, min(first[1]+first[3], box[1]+box[3])-max(first[1], box[1]))
            overlap = intersection/max(first[2]*first[3]+box[2]*box[3]-intersection, 1e-9)
            if overlap < .5:
                reasons.append('target_moved_or_changed_during_burst')
                break
        methods = {f['geometry']['contour_detail']['selection'] for f in frames}
        if len(methods) != 1:
            reasons.append('rim_measurement_method_changed_during_burst')
        for point in ('top_center', 'geometric_body_center'):
            values = np.asarray([f[point+'_world_m'] for f in frames], float)
            median = np.median(values, axis=0)
            radial = np.linalg.norm(values-median, axis=1)*1000
            summary[point+'_world_m'] = median.tolist()
            summary[point+'_repeatability_max_mm'] = float(max(radial))
            summary[point+'_repeatability_p95_mm'] = float(np.percentile(radial, 95))
            summary[point+'_all_inside_FIT_support'] = all(f[point+'_inside_FIT_support'] for f in frames)
            if max(radial) > MAX_REPEATABILITY_MM:
                reasons.append(point+'_repeatability_exceeds_3mm')
        summary['lower_half_inferred'] = any(f['geometry']['contour_detail'].get('lower_half_inferred', False) for f in frames)
    summary.update(frame_quality_pass=not reasons,
        burst_qualified=not reasons and elapsed_seconds is not None and elapsed_seconds >= MIN_SECONDS,
        completion_wall_time_known=elapsed_seconds is not None, reject_reasons=reasons)
    return summary


def relative_pair(a, b, reference_spacing_mm):
    delta = (np.asarray(b['top_center_world_m'])-np.asarray(a['top_center_world_m']))*1000
    xy = float(math.hypot(delta[0], delta[1]))
    error = xy-reference_spacing_mm
    # If each endpoint met the original 5mm positional bound, the triangle
    # inequality would allow 10mm separation error. This necessary check is
    # not sufficient to establish that either endpoint actually met 5mm.
    bound = 10.
    return dict(scope='two_equal_height_recesses_relative_scale_and_height_only',
        reference_spacing_mm=reference_spacing_mm,
        reference_source='operator_supplied_center_spacing_with_equal_recess_bottom_heights',
        observed_delta_world_mm=delta.tolist(), observed_horizontal_distance_mm=xy,
        observed_3D_distance_mm=float(np.linalg.norm(delta)),
        horizontal_distance_error_mm=float(error), height_difference_mm=float(delta[2]),
        necessary_pair_bound_mm=bound, bound_source='2_times_original_5mm_endpoint_bound',
        relative_check_pass=bool(abs(error) <= bound and abs(delta[2]) <= bound),
        absolute_object_5mm_accuracy_accepted=False,
        both_top_centers_inside_FIT_support=a['top_center_all_inside_FIT_support'] and b['top_center_all_inside_FIT_support'],
        camera_transform_refitted=False, coordinates_snapped_or_rescaled=False,
        execution_ready=False)

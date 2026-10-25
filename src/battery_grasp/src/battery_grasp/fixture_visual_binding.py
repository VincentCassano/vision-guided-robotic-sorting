"""Corroborate a measured adjacent fixture slot using its recorded RGB-D view.

The caller owns unchanged-camera checks, physical fixture confirmation, and all
motion authority. This module provides neither a grasp center nor calibration
acceptance. Its depth budget is an empirical local-surface consistency budget,
not a statistical confidence interval or an absolute calibration guarantee.
"""
import math
from .vision_numeric import depth_spread_check


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + ': finite number required')
    return float(value)


def _vector(values, count, label):
    if not isinstance(values, (list, tuple)) or len(values) != count:
        raise ValueError(label + ': incorrect vector length')
    return [_number(value, label) for value in values]


def _camera(document, label):
    if not isinstance(document, dict):
        raise ValueError(label + ': document required')
    width, height = document.get('width'), document.get('height')
    if type(width) is not int or type(height) is not int or min(width, height) <= 0:
        raise ValueError(label + ': positive integer image dimensions required')
    intrinsic = document['intrinsics']
    if not isinstance(intrinsic, dict):
        raise ValueError(label + ': intrinsics object required')
    if intrinsic.get('width') != width or intrinsic.get('height') != height:
        raise ValueError(label + ': intrinsic/image dimensions differ')
    fx, fy, ppx, ppy = [_number(intrinsic[name], label + ' ' + name)
                        for name in ('fx', 'fy', 'ppx', 'ppy')]
    if fx <= 0 or fy <= 0 or not (0 <= ppx < width and 0 <= ppy < height):
        raise ValueError(label + ': positive focal lengths and in-image principal point required')
    coefficients = _vector(intrinsic['coeffs'], 5, label + ' distortion coefficients')
    scale = _number(document['depth_scale_m'], label + ' depth scale')
    if scale <= 0:
        raise ValueError(label + ': positive depth scale required')
    matrix = document['world_from_camera']
    if not isinstance(matrix, (list, tuple)) or len(matrix) != 4:
        raise ValueError(label + ': homogeneous matrix required')
    matrix = [_vector(row, 4, label + ' matrix row') for row in matrix]
    if matrix[3] != [0., 0., 0., 1.]:
        raise ValueError(label + ': homogeneous matrix last row differs')
    return dict(width=width, height=height, fx=fx, fy=fy, ppx=ppx, ppy=ppy,
                coefficients=coefficients, depth_scale_m=scale,
                world_from_camera=matrix)


def _box(row, camera, label):
    if not isinstance(row, dict):
        raise ValueError(label + ': detection object required')
    x, y, width, height = _vector(row['bbox'], 4, label + ' bbox')
    u, v = _number(row['u'], label + ' u'), _number(row['v'], label + ' v')
    valid = (width > 0 and height > 0 and x >= 0 and y >= 0
             and x + width <= camera['width'] and y + height <= camera['height']
             and 0 <= u < camera['width'] and 0 <= v < camera['height'])
    center_matches = u == x + width / 2 and v == y + height / 2
    # These are the inclusive pixel-center limits used by depth_preview.measure:
    # depth[max(0,int(v)-3):min(H,int(v)+4), max(0,int(u)-3):min(W,int(u)+4)].
    patch = dict(u_min=max(0, int(u)-3), u_max=min(camera['width']-1, int(u)+3),
                 v_min=max(0, int(v)-3), v_max=min(camera['height']-1, int(v)+3))
    return dict(bbox=[x, y, width, height], center_px=[u, v],
                bbox_complete_in_image=valid, center_equals_bbox_center=center_matches,
                sampled_depth_patch_inclusive_px=patch)


def _observation(row, camera, label):
    result = _box(row, camera, label)
    optical = _vector(row['position_optical_m'], 3, label + ' optical point')
    depth = _number(row['depth_m'], label + ' depth')
    spread = _number(row['depth_spread_m'], label + ' depth spread')
    if not .15 <= depth <= 3.0 or optical[2] <= 0:
        raise ValueError(label + ': recorded depth outside depth-preview range')
    u, v = result['center_px']
    reconstructed = [(u-camera['ppx']) / camera['fx'] * depth,
                     (v-camera['ppy']) / camera['fy'] * depth, depth]
    # rs2_deproject_pixel_to_point returns float32 coordinates, whereas saved
    # depth_m is a Python double. Eight float32 roundings conservatively cover
    # the input casts, subtract/divide/multiply and returned coordinate cast.
    # This floating-point arithmetic allowance is separate from scene budgets.
    epsilon = 2.0 ** -23
    rounding_factor = 8 * epsilon / (1 - 8 * epsilon)
    rounding_budget = [rounding_factor * max(abs(a), abs(b), depth)
                       for a, b in zip(optical, reconstructed)]
    residual = [a-b for a, b in zip(optical, reconstructed)]
    result.update(position_optical_m=optical, depth_m=depth, depth_spread_m=spread,
                  pinhole_reconstructed_optical_m=reconstructed,
                  optical_pinhole_residual_m=residual,
                  float32_deprojection_rounding_budget_m=rounding_budget,
                  optical_matches_depth_and_pixel=all(abs(a) <= b for a, b
                                                     in zip(residual, rounding_budget)))
    return result


def _inside_patch(pixel, observation):
    patch = observation['sampled_depth_patch_inclusive_px']
    return (observation['bbox_complete_in_image']
            and observation['center_equals_bbox_center']
            and patch['u_min'] <= pixel[0] <= patch['u_max']
            and patch['v_min'] <= pixel[1] <= patch['v_max'])


def _overlaps_patch(footprint, observation):
    patch = observation['sampled_depth_patch_inclusive_px']
    return (observation['bbox_complete_in_image']
            and observation['center_equals_bbox_center']
            and max(footprint['u_min'], patch['u_min']) <= min(footprint['u_max'], patch['u_max'])
            and max(footprint['v_min'], patch['v_min']) <= min(footprint['v_max'], patch['v_max']))


def qualify_fixture_observation(reference, document, old_index, new_index, shift_mm):
    """Return every observation gate; malformed input raises ValueError/KeyError.

    A successful result only corroborates the explicitly indexed, same-height
    adjacent fixture slot. Motion must use the measured +/-116 mm fixture shift,
    never the detected visible surface as the battery axis or a grasp center.
    """
    if type(shift_mm) is not int or shift_mm not in (-116, 116):
        raise ValueError('Fixture shift must be an explicitly signed integer +/-116 mm')
    old_camera, new_camera = _camera(reference, 'reference'), _camera(document, 'current')
    old_rows, new_rows = reference['detections'], document['detections']
    if not isinstance(old_rows, list) or not isinstance(new_rows, list):
        raise ValueError('Detection lists required')
    if (type(old_index) is not int or not 0 <= old_index < len(old_rows)
            or type(new_index) is not int or not 0 <= new_index < len(new_rows)):
        raise ValueError('Explicit reference/current detection indices required')
    old_row, new_row = old_rows[old_index], new_rows[new_index]
    old = _observation(old_row, old_camera, 'reference detection')
    new = _observation(new_row, new_camera, 'current detection')
    failures, gates = [], {}

    def gate(name, passed):
        gates[name] = bool(passed)
        if not passed:
            failures.append(name)

    gate('same_intrinsics_depth_scale_and_world_transform', old_camera == new_camera)
    gate('zero_distortion_required_for_pinhole_prediction',
         all(value == 0 for camera in (old_camera, new_camera)
             for value in camera['coefficients']))
    for label, row, observation in (('reference', old_row, old), ('current', new_row, new)):
        gate(label + '_confirmed_valid_black_cyl_battery',
             row.get('class_name') == 'black_cyl_battery'
             and row.get('visual_confirmed') is True and row.get('depth_valid') is True)
        gate(label + '_bbox_complete_in_image', observation['bbox_complete_in_image'])
        gate(label + '_center_equals_bbox_center', observation['center_equals_bbox_center'])
        gate(label + '_optical_depth_pixel_consistency', observation['optical_matches_depth_and_pixel'])
        observation['depth_spread_check']=depth_spread_check(observation['depth_spread_m'])
        gate(label + '_depth_spread_0_to_5mm', observation['depth_spread_check']['pass'])

    # Camera displacement is R_world_from_camera.T * [signed shift, 0, 0].
    # Thus its three components are the first ROW of R_world_from_camera.
    world_x_in_camera = old_camera['world_from_camera'][0][:3]
    expected_optical = [a + b * shift_mm / 1000
                        for a, b in zip(old['position_optical_m'], world_x_in_camera)]
    positive_prediction = expected_optical[2] > 0
    gate('predicted_optical_depth_positive', positive_prediction)
    expected_pixel = None
    predicted_reference_patch = None
    center_inside_current_patch = False
    matching_indices = []
    candidates = []
    depth_residual = None
    if positive_prediction:
        expected_pixel = [new_camera['fx'] * (expected_optical[0] / expected_optical[2]) + new_camera['ppx'],
                          new_camera['fy'] * (expected_optical[1] / expected_optical[2]) + new_camera['ppy']]
        if not all(math.isfinite(value) for value in expected_pixel):
            raise ValueError('Finite fixture projection required')
        gate('predicted_pixel_inside_image',
             0 <= expected_pixel[0] < new_camera['width']
             and 0 <= expected_pixel[1] < new_camera['height'])
        center_inside_current_patch = _inside_patch(expected_pixel, new)
        source_patch = old['sampled_depth_patch_inclusive_px']
        projected_corners = []
        source_corners = []
        for u in (source_patch['u_min'], source_patch['u_max']):
            for v in (source_patch['v_min'], source_patch['v_max']):
                source_corners.append([u, v])
                optical_corner = [(u-old_camera['ppx']) / old_camera['fx'] * old['depth_m'],
                                  (v-old_camera['ppy']) / old_camera['fy'] * old['depth_m'],
                                  old['depth_m']]
                moved = [a+b*shift_mm/1000 for a, b in zip(optical_corner, world_x_in_camera)]
                if moved[2] <= 0:
                    raise ValueError('Positive depth required for every projected sampling-patch corner')
                projected = [new_camera['fx'] * (moved[0]/moved[2]) + new_camera['ppx'],
                             new_camera['fy'] * (moved[1]/moved[2]) + new_camera['ppy']]
                if not all(math.isfinite(value) for value in projected):
                    raise ValueError('Finite projected sampling-patch corner required')
                projected_corners.append(projected)
        predicted_reference_patch = dict(
            u_min=min(row[0] for row in projected_corners),
            u_max=max(row[0] for row in projected_corners),
            v_min=min(row[1] for row in projected_corners),
            v_max=max(row[1] for row in projected_corners),
            source_pixel_center_corners=source_corners,
            projected_pixel_center_corners=projected_corners,
            corner_depth_source='reference_depth_m_constant_local_surface')
        gate('predicted_reference_and_current_depth_footprints_overlap',
             _overlaps_patch(predicted_reference_patch, new))
        for index, candidate in enumerate(new_rows):
            if (candidate.get('class_name') != 'black_cyl_battery'
                    or candidate.get('visual_confirmed') is not True
                    or candidate.get('depth_valid') is not True):
                continue
            box = _box(candidate, new_camera, 'candidate ' + str(index))
            matches = _overlaps_patch(predicted_reference_patch, box)
            candidates.append(dict(index=index, overlaps_predicted_reference_depth_footprint=matches,
                                   predicted_center_inside_depth_patch=_inside_patch(expected_pixel, box), **box))
            if matches:
                matching_indices.append(index)
        depth_residual = new['depth_m'] - expected_optical[2]
    else:
        gate('predicted_pixel_inside_image', False)
        gate('predicted_reference_and_current_depth_footprints_overlap', False)
    gate('unique_matching_confirmed_candidate_is_selected', matching_indices == [new_index])
    depth_budget = ((old['depth_spread_m'] + new['depth_spread_m']) / 2
                    + max(old_camera['depth_scale_m'], new_camera['depth_scale_m']))
    gate('local_surface_depth_consistency',
         depth_residual is not None and depth_budget >= 0 and abs(depth_residual) <= depth_budget)
    height_ratio = None
    width_ratio = None
    if old['bbox'][3] > 0:
        height_ratio = new['bbox'][3] * new['depth_m'] / (old['bbox'][3] * old['depth_m'])
    if old['bbox'][2] > 0:
        width_ratio = new['bbox'][2] * new['depth_m'] / (old['bbox'][2] * old['depth_m'])
    gate('depth_normalized_bbox_height_ratio_0_8_to_1_2',
         height_ratio is not None and .8 <= height_ratio <= 1.2)
    return dict({'pass': not failures}, schema_version=1, failures=failures, gates=gates,
                scope='vision_corroborated_fixture_slot_not_grasp_center_or_global_calibration',
                measured_fixture_world_X_shift_mm=shift_mm,
                reference_index=old_index, selected_index=new_index,
                reference_observation=old, current_observation=new,
                expected_optical_m=expected_optical, expected_pixel_uv=expected_pixel,
                predicted_reference_depth_footprint_px=predicted_reference_patch,
                current_depth_footprint_px=new['sampled_depth_patch_inclusive_px'],
                selected_center_minus_predicted_center_px=(
                    [a-b for a, b in zip(new['center_px'], expected_pixel)]
                    if expected_pixel is not None else None),
                predicted_center_inside_current_patch_diagnostic_only=center_inside_current_patch,
                expected_depth_m=expected_optical[2], depth_residual_m=depth_residual,
                local_surface_depth_budget_m=depth_budget,
                depth_budget_rule='(old_spread+new_spread)/2 + max(old_scale,new_scale)',
                depth_budget_interpretation='empirical_local_surface_consistency_not_statistical_confidence_or_absolute_calibration',
                sampling_rule='project_reference_actual_7x7_pixel_center_corners_at_recorded_depth_then_require_overlap_with_current_actual_7x7_patch',
                footprint_overlap_rule='inclusive_axis_bounds_nonempty_intersection_no_added_pixel_margin',
                predicted_center_inside_current_patch_is_rejection_gate=False,
                matching_indices=matching_indices, candidate_patch_observations=candidates,
                depth_normalized_bbox_height_ratio=height_ratio,
                depth_normalized_bbox_width_ratio=width_ratio,
                width_ratio_is_rejection_gate=False,
                control_target_source='reference_TCP_plus_measured_fixture_shift_not_detected_surface',
                global_calibration_accepted=False, grasp_center_observed=False,
                hardware_or_motion_authority_granted=False)

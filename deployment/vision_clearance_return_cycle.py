"""One explicitly authorized empty vertical retreat; no hand or lateral writes."""
import copy
import hashlib
import math

from battery_grasp.real_grasp import orientation_angle_rad
from high_descent_prepare_worker import same_pose
from return_after_release_worker import (
    SAFETY_CONFIGURATION, arm_shape, metadata_signature, model_signature,
    prepare_readback, strict_json)
from vision_cycle_hand import PREPARE
from vision_cycle_supported_release import SupportedCycle


CLEARANCE_SUCCESS = 'completed_empty_vertical_retreat_top_clearance_confirmed'
CLEARANCE_INSUFFICIENT = 'completed_empty_vertical_retreat_clearance_insufficient'
CLEARANCE_PENDING = 'completed_empty_vertical_retreat_measurement_pending'
CLEARANCE_OUTCOMES = (CLEARANCE_SUCCESS, CLEARANCE_INSUFFICIENT, CLEARANCE_PENDING)
SOURCE_SUCCESS = 'completed_taught_descent_grasp_release_empty_return'
SOURCE_PAYLOAD_SHA256 = {
    'worker': 'be92378e63de416d7340b4ee0563a26d44f0a6892c9fc028267673afd2120393',
    'parent': '7b2c185da1b86a09b9fe70829cca69f96da94d904c23bb9adc516af30005a359',
}


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Finite numeric value required: ' + name)
    return value


def _integer(value, expected, name):
    if type(value) is not int or value != expected:
        raise ValueError('Unexpected source integer: ' + name)


def _pose_vector(value, name):
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError('Complete source pose required: ' + name)
    for component in value:
        _number(component, name)
    return value


def _same_cartesian(a, b):
    return (a['frame_signatures'] == b['frame_signatures']
        and a['limits'] == b['limits']
        and math.dist(a['pose'][:3], b['pose'][:3]) <= .0005
        and orientation_angle_rad(a['pose'][3:], b['pose'][3:]) <= .005)


def _prepare_profile(row, profile):
    prepare_readback(row)
    if (row['identity'] != profile['device_identity']
            or row.get('speeds') != [65535] * 6
            or row.get('force_targets') != [0] * 5
            or row.get('current_limit_ma') != profile['current_limit_ma']
            or any(current > limit for current, limit in
                zip(row['current'], profile['current_limit_ma']))):
        raise ValueError('Complete identity/current/configuration PREPARE feedback required')


def read_clearance_context(folder, profile):
    """Accept the captured bytes of the reviewed 181625 three-segment cycle."""
    current_parent = strict_json((folder / 'parent_result.json').read_bytes())
    records = current_parent['clearance_source']
    documents = {}
    for role in ('worker', 'parent'):
        record = records[role + '_report']
        name = role + '_result.json'
        captured_path = 'clearance_source/' + name
        raw = (folder / captured_path).read_bytes()
        if (type(record.get('byte_count')) is not int
                or record['byte_count'] != len(raw)
                or record.get('captured_path') != captured_path
                or record.get('sha256') != hashlib.sha256(raw).hexdigest()
                or hashlib.sha256(raw).hexdigest() != SOURCE_PAYLOAD_SHA256[role]):
            raise ValueError('Clearance source is not the exact reviewed 181625 payload: ' + role)
        documents[role] = strict_json(raw)
    old = documents['worker']
    parent = documents['parent']
    _integer(old.get('schema_version'), 1, 'schema_version')
    _integer(parent.get('schema_version'), 1, 'parent.schema_version')
    _integer(parent.get('worker_return_code'), 0, 'worker_return_code')
    if (old.get('scope') != 'fresh_taught_shell_descent_supported_grasp_release_empty_return'
            or parent.get('scope') != old['scope']
            or old.get('outcome') != SOURCE_SUCCESS
            or parent.get('outcome') != SOURCE_SUCCESS
            or parent.get('worker_result') != old
            or old.get('phase') != 'completed'
            or parent.get('exceptions') != []
            or parent.get('worker_cleanup_timeout') is not False
            or old.get('exceptions') != [] or old.get('cleanup_errors') != []
            or old.get('arm_stop_attempt') is not None
            or old.get('arm_command_attempted') is not True
            or old.get('saved_goal_source_accepted') is not True
            or old.get('saved_goal_source_context_accepted') is not True
            or old.get('saved_goal_automatic50_requested') is not True
            or old.get('same_goal_scene_reconfirmed_by_operator') is not True
            or old.get('explicit_supported_opening_requested') is not True
            or old.get('supervised_cycle_feedback_completed') is not True
            or old.get('load_lift_or_transfer_performed') is not False
            or old.get('vision_used') is not False
            or old.get('camera_extrinsics_used_for_motion') is not False
            or old.get('hand_shape') != 'PREPARE'):
        raise ValueError('Clean completed saved-goal cycle required for this empty retreat')
    for name, expected in (('arm_actuator_writes', 3), ('hand_register_writes', 2),
            ('controller_configuration_writes', 0), ('hand_bus_configuration_writes', 0)):
        _integer(old.get(name), expected, name)
    expected_inputs = (
        (9, 'await_saved_goal_lift', 'u', 'saved_goal_empty_noncontact_and_up50_sweep_clear'),
        (4, 'await_descend', 'd', 'same_corridor_scene_and_stop_confirmed'),
        (5, 'await_grasp', 'g', 'only_intended_shell_contact_and_closure_clear'),
        (6, 'await_supported_release', 'r', 'shell_fully_supported_and_prepare_release_clear'),
        (7, 'await_empty_return', 'u', 'shell_on_board_hand_empty_and_corridor_clear'))
    events = old.get('operator_inputs')
    if not isinstance(events, list) or len(events) != len(expected_inputs):
        raise ValueError('Complete original onsite permission sequence required')
    if parent.get('operator_inputs') != events:
        raise ValueError('Clearance source parent/worker permissions differ')
    for event, (index, phase, value, flag) in zip(events, expected_inputs):
        _integer(event.get('index'), index, 'permission.index')
        if event.get('phase') != phase or event.get('value') != value or event.get(flag) is not True:
            raise ValueError('Original supported cycle permission missing: ' + flag)
    goal = old['taught_goal']
    upper = old['upper']
    final = old['final_arm']
    for arm in (goal, upper, final):
        arm_shape(arm)
    gap = _number(old.get('board_clearance_mm'), 'board_clearance_mm')
    if gap < 10:
        raise ValueError('Original goal board-gap confirmation missing')
    height = (upper['pose'][2] - goal['pose'][2]) * 1000
    if (abs(height - 50) > .5
            or math.dist(goal['pose'][:2], upper['pose'][:2]) > .0005
            or orientation_angle_rad(goal['pose'][3:], upper['pose'][3:]) > .005
            or not _same_cartesian(upper, final)):
        raise ValueError('Original final feedback is outside the observed 50mm return corridor')
    segments = old.get('arm_segments')
    phases = ('lifting_saved_goal_50mm', 'descending', 'returning_empty')
    if not isinstance(segments, list) or len(segments) != 3:
        raise ValueError('Exactly three original arm segments required')
    for segment, phase, sign in zip(segments, phases, (1, -1, 1)):
        if segment.get('phase') != phase:
            raise ValueError('Original motion phase order differs')
        plan = segment['plan']
        start = plan['start']
        endpoint = segment['endpoint']
        arm_shape(start)
        arm_shape(endpoint)
        target = _pose_vector(plan.get('target_sdk_pose_m_rad'), 'segment.target')
        distance = _number(plan.get('distance_mm'), 'segment.distance_mm')
        if (start['frame_signatures'] != goal['frame_signatures']
                or start['limits'] != goal['limits']
                or endpoint['frame_signatures'] != goal['frame_signatures']
                or endpoint['limits'] != goal['limits']
                or abs(distance - sign * 50) > .6
                or abs(distance - (target[2] - start['pose'][2]) * 1000) > 1e-6
                or math.dist(target[:2], start['pose'][:2]) > 1e-9
                or target[3:] != start['pose'][3:]
                or math.dist(endpoint['pose'][:3], target[:3]) > .0005
                or orientation_angle_rad(endpoint['pose'][3:], target[3:]) > .005):
            raise ValueError('Original vertical segment endpoint/corridor evidence invalid')
    if (not _same_cartesian(segments[0]['plan']['start'], goal)
            or not _same_cartesian(segments[0]['endpoint'], upper)
            or not _same_cartesian(segments[1]['plan']['start'], upper)
            or not _same_cartesian(segments[1]['endpoint'], goal)
            or not _same_cartesian(segments[2]['plan']['start'], goal)
            or not same_pose(segments[2]['endpoint'], final)):
        raise ValueError('Original vertical segment continuity/final feedback differs')
    if old['configuration'] != profile['observed_configuration']:
        raise ValueError('Original hand configuration differs from the measured profile')
    _prepare_profile(old['final_hand'], profile)
    _prepare_profile(old['last_preflight'], profile)
    metadata_signature(old['sdk_environment'])
    model_signature(old['fresh_model'])
    return dict(worker=old, parent=parent, source_records=records)


def confirm_hand_top_clearance(sequence, final):
    """Defer actual measurement until the parent has observed worker cleanup."""
    sequence.report.update(
        final_arm=copy.deepcopy(final),
        final_hand=copy.deepcopy(sequence.report['last_preflight']),
        hand_top_clearance_measurement_basis='vertical_lowest_whole_hand_surface_above_current_battery_top_horizontal_plane',
        hand_top_clearance_required_mm=20,
        hand_top_clearance_pending_after_worker_exit=True,
        hand_above_target_top_confirmed=False, lateral_transfer_ready=False,
        full_pregrasp_execution_ready=False, execution_ready=False,
        physical_stop_verified=False, torque_release_verified=False,
        historical_geometry_does_not_authorize_lateral_motion=True)
    sequence.save()
    return None


class ClearanceReturnCycle(SupportedCycle):
    def __init__(self, *args, clearance_context, **kwargs):
        super().__init__(*args, **kwargs)
        if (not self.taught_only or not self.allow_protected_opening
                or self.enable_arm_protection or self.resume_report is not None
                or self.completed_corridor is not None):
            raise ValueError('Empty clearance retreat requires isolated taught-only read-existing-configuration mode')
        self.clearance_context = clearance_context

    def run(self):
        initial, tip = self.preflight(None, None)
        old = self.clearance_context['worker']
        final = old['final_arm']
        if (not same_pose(initial, final) or initial['limits'] != final['limits']
                or metadata_signature(self.report['sdk_environment']) != metadata_signature(old['sdk_environment'])
                or model_signature(self.report['fresh_model']) != model_signature(old['fresh_model'])
                or self.report['configuration'] != old['configuration']
                or tip != old['live_tip_parameters']):
            raise RuntimeError('Current final pose/device changed; no automatic old-point alignment')
        if any(self.report['arm_safety_reads'][name] != old['arm_safety_reads'][name]
                for name in SAFETY_CONFIGURATION):
            raise RuntimeError('Actual safety/install/drive differs from the successful source')
        _prepare_profile(self.report['last_preflight'], self.profile)
        goal = old['taught_goal']
        target_z = goal['pose'][2] + .100
        additional_mm = (target_z - initial['pose'][2]) * 1000
        if not 30 <= additional_mm <= 120:
            raise RuntimeError('Requested empty clearance retreat is outside the existing vertical corridor bounds')
        self.authority.prepare_feedback_pass = True
        self.report.update(clearance_source_accepted=True,
            clearance_source_records=self.clearance_context['source_records'],
            initial_arm=initial, taught_goal=copy.deepcopy(goal),
            original_upper=copy.deepcopy(old['upper']),
            previous_final=copy.deepcopy(final),
            board_clearance_mm=old['board_clearance_mm'],
            board_clearance_is_not_battery_clearance=True,
            historical_clearances_reused=False,
            clearance_goal_source='reviewed_successful_cycle_taught_goal_Z_plus100mm',
            clearance_target_world_mm=[initial['pose'][0] * 1000, initial['pose'][1] * 1000, target_z * 1000],
            clearance_requested_additional_up_mm=additional_mm,
            clearance_total_height_above_taught_goal_mm=100,
            clearance_measurement_expected_tip=copy.deepcopy(tip),
            hand_above_target_top_confirmed=False, lateral_transfer_ready=False,
            no_hand_or_configuration_writes=True, automatic_lateral_motion=False,
            vision_used=False, camera_extrinsics_used_for_motion=False,
            load_lift_or_transfer_performed=False, no_automatic_old_point_alignment=True)
        self.save()
        event = self.wait_input(10, 'await_clearance_lift', initial, ('u',))
        if event.get('empty_hand_upward_clearance_sweep_confirmed') is not True:
            raise RuntimeError('Fresh empty-hand full upward swept-volume clearance not confirmed')
        self.plan = None
        reached = self.execute_vertical(initial, tip, target_z, 'lifting_empty_clearance')
        self.report['phase'] = 'final_cycle_preflight'
        self.save()
        final, final_tip = self.preflight(reached, PREPARE)
        if final_tip != tip or final['limits'] != initial['limits']:
            raise RuntimeError('Actual dynamics/tip changed after the empty retreat')
        _prepare_profile(self.report['last_preflight'], self.profile)
        self.report.update(final_arm=final, upper=copy.deepcopy(final),
            final_hand=copy.deepcopy(self.report['last_preflight']),
            clearance_actual_additional_up_mm=(final['pose'][2] - initial['pose'][2]) * 1000,
            clearance_actual_height_above_taught_goal_mm=(final['pose'][2] - goal['pose'][2]) * 1000,
            hand_shape='PREPARE', arm_feedback_observed=True,
            execution_ready=False, physical_stop_verified=False,
            torque_release_verified=False, lateral_transfer_ready=False)
        self.save()
        confirm_hand_top_clearance(self, final)
        self.report.update(outcome=CLEARANCE_PENDING,
            phase='completed', supervised_empty_vertical_retreat_feedback_completed=True,
            full_pregrasp_execution_ready=False, autonomous_other_targets_ready=False,
            load_lift_or_transfer_performed=False, execution_ready=False,
            functional_validation='onsite_empty_vertical_retreat_feedback_completed_actual_top_clearance_pending')
        self.authority._completed = True
        self.save()

"""Continue the reviewed completed descent without repeating it or recapturing.

The saved terminal is the successful full feedback endpoint of the actual MoveL,
not the later fault sample and not the older taught wrist.  Fresh device/pose
qualification and a new onsite g/r/u sequence are still required in this run.
"""
import copy
import hashlib
import math
from pathlib import Path

from battery_grasp.localized_pregrasp import select_surface
from battery_grasp.real_grasp import orientation_angle_rad
from high_descent_prepare_worker import same_pose
from return_after_release_worker import (
    SAFETY_CONFIGURATION, arm_shape, metadata_signature, model_signature,
    strict_json)
from vision_clearance_return_cycle import _integer, _number, _prepare_profile
from vision_cycle_hand import PREPARE
from vision_cycle_sequence import CALIBRATION_SHA
from vision_cycle_supported_release import SupportedCycle
from vision_high_return_cycle import MEASUREMENT_BASIS, SCENE_FLAG, pose_comparison, verify_bound_descent


TERMINAL_SUCCESS = 'completed_visual_grasp_release_empty_return_from_saved_terminal'
SOURCE_SCOPE = 'same_qualified_upper_new_wrist_visual_grasp_cycle'
SOURCE_ERROR = "RuntimeError('Observed terminal pose differs from taught target; no grasp')"
SOURCE_PAYLOAD_SHA256 = {
    'worker': 'b26d981052a533890ebdde61ac5f22eb1c7b7e39f85053abcab2987965501792',
    'parent': '56fa5826c1a7e89e03cada88f85ae0f83804d5170c7d6a75a41859724fd215c0',
}
VISION_SHA256 = {
    'detections.json': '2248660e19982b7f12cd7f599cfbd006f8fe9b1976c3ea9b733badbbd3eda36a',
    'original.png': '320f556afcdf118f2d40073441ac0c2101f38810e7d501195ddb28d490a37c6e',
    'detections.png': '9c3cef3dbd01eec790e76ee314e600817ec155e1fb7f89b8c26d867fb50e9d40',
    'aligned_depth_u16.png': '66da4f112882d97a4a95fe8c373222ff589dfeb3117b75f9a7a8bf074fcb0180',
}
BINDING_SHA256 = '18ac33db0ddfe821f53a8ffffbfbb9e39b64e7c84fd9383d5f78f1a88542013a'


def _captured(folder, record, relative, digest):
    raw = (folder / relative).read_bytes()
    if (record.get('captured_path') != relative
            or type(record.get('byte_count')) is not int
            or record['byte_count'] != len(raw)
            or record.get('sha256') != hashlib.sha256(raw).hexdigest()
            or record['sha256'] != digest):
        raise ValueError('Reviewed terminal source bytes differ: ' + relative)
    return raw


def read_terminal_context(folder, profile):
    """Accept only the exact 192922 reports and their actual RGB-D capture."""
    current_parent = strict_json((folder / 'parent_result.json').read_bytes())
    records = current_parent['terminal_resume_source']
    documents = {}
    for role in ('worker', 'parent'):
        documents[role] = strict_json(_captured(folder, records[role + '_report'],
            'terminal_resume_source/' + role + '_result.json', SOURCE_PAYLOAD_SHA256[role]))
    old, parent = documents['worker'], documents['parent']
    for document in (old, parent):
        _integer(document.get('schema_version'), 1, 'source.schema_version')
        if document.get('scope') != SOURCE_SCOPE or document.get('outcome') != 'aborted_no_further_writes':
            raise ValueError('Expected the reviewed same-slot terminal-comparison refusal')
    _integer(parent.get('worker_return_code'), 1, 'source.worker_return_code')
    if (parent.get('worker_result') != old or parent.get('exceptions') != []
            or parent.get('worker_cleanup_timeout') is not False
            or old.get('cleanup_errors') != [] or old.get('phase') != 'fault'
            or len(old.get('exceptions', [])) != 1
            or old['exceptions'][0].get('error') != SOURCE_ERROR
            or old.get('permission_index') != 4
            or old.get('arm_stop_attempt') != {
                'result': 0, 'physical_stop_verified': False, 'hand_stop_or_release': False}
            or old.get('arm_command_attempted') is not True
            or old.get('high_return_source_context_accepted') is not True
            or old.get('high_return_source_accepted') is not True
            or old.get('high_return_same_scene_reconfirmed_by_operator') is not True
            or old.get('vision_used') is not True
            or old.get('previous_vision_reused') is not False
            or old.get('automatic_lateral_motion') is not False
            or old.get('moves') != []):
        raise ValueError('Terminal source fault, cleanup or actuator history differs')
    for key, value in (('arm_actuator_writes', 1), ('hand_register_writes', 0),
            ('controller_configuration_writes', 0), ('hand_bus_configuration_writes', 0)):
        _integer(old.get(key), value, key)
    events = old.get('operator_inputs')
    if (not isinstance(events, list) or len(events) != 2
            or [row.get('index') for row in events] != [3, 4]
            or [row.get('value') for row in events] != ['b', 'd']
            or [row.get('phase') for row in events] != ['await_vision', 'await_descend']
            or any(row.get(SCENE_FLAG) is not True for row in events)
            or events[0].get('same_taught_stationary_shell_confirmed') is not True
            or events[1].get('same_corridor_scene_and_stop_confirmed') is not True
            or [row for row in parent.get('operator_inputs', []) if row.get('index') in (3, 4)] != events):
        raise ValueError('Original same-scene visual and descent permissions required')
    qualification = old.get('high_return_start_qualification', {})
    checks = qualification.get('checks')
    if (not isinstance(checks, dict) or not checks
            or any(value is not True for value in checks.values())
            or qualification.get('failed_checks') != []
            or qualification.get('rejected_wait_sample_used_as_anchor') is not False
            or qualification.get('tolerances_unchanged') is not True):
        raise ValueError('Original high-return full-device start was not qualified')
    confirmation = old['high_return_operator_confirmation']
    bundled = strict_json(Path(__file__).with_name(
        'clearance_183637_operator_confirmation.json').read_bytes())
    if (confirmation != bundled or confirmation.get('measurement_basis') != MEASUREMENT_BASIS
            or _number(confirmation.get('measured_clearance_mm'), 'measured_clearance_mm') != 25.3
            or _number(confirmation.get('minimum_reported_clearance_mm'), 'minimum_reported_clearance_mm') != 25.2):
        raise ValueError('Original upper has no reviewed 25.3mm whole-hand top-clearance evidence')
    if old['configuration'] != profile['observed_configuration']:
        raise ValueError('Source hand configuration differs from the measured profile')
    _prepare_profile(old['last_preflight'], profile)
    metadata_signature(old['sdk_environment'])
    model_signature(old['fresh_model'])
    if any(name not in old.get('arm_safety_reads', {}) for name in SAFETY_CONFIGURATION):
        raise ValueError('Complete source protection/installation/drive readback required')
    segments = old.get('arm_segments')
    if not isinstance(segments, list) or len(segments) != 1 or segments[0].get('phase') != 'descending':
        raise ValueError('Exactly one completed real descending segment required')
    segment = segments[0]
    plan, terminal, upper, goal = segment['plan'], segment['endpoint'], old['upper'], old['taught_goal']
    start, target = plan['start'], plan['target_sdk_pose_m_rad']
    for arm in (start, terminal, upper, goal):
        arm_shape(arm)
        if arm['limits'] != upper['limits'] or arm['frame_signatures'] != upper['frame_signatures']:
            raise ValueError('Source completed segment frames/limits changed')
    if (old.get('plan') != plan or not same_pose(start, upper)
            or not isinstance(target, list) or len(target) != 6
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in target)
            or target[:2] != start['pose'][:2] or target[3:] != start['pose'][3:]
            or abs(target[2] - goal['pose'][2]) > 1e-9
            or not 30 <= -(target[2] - start['pose'][2]) * 1000 <= 120
            or math.dist(terminal['pose'][:3], target[:3]) > .0005
            or orientation_angle_rad(terminal['pose'][3:], target[3:]) > .005
            or plan.get('speed_percent') != 2 or plan.get('nominal_speed_mm_s') != 5.0
            or abs(_number(plan.get('distance_mm'), 'distance_mm')
                - (target[2] - start['pose'][2]) * 1000) > 1e-6):
        raise ValueError('Completed source endpoint differs from its actual commanded target')
    if (plan['drive_limits']['minimum'] != old['arm_safety_reads']['Get_Joint_Drive_Min_Pos'][1]
            or plan['drive_limits']['maximum'] != old['arm_safety_reads']['Get_Joint_Drive_Max_Pos'][1]
            or old['current_qualified_upper_reference'] != upper):
        raise ValueError('Original segment drive limits or qualified upper changed')
    raw_vision = {}
    for name, digest in VISION_SHA256.items():
        raw_vision[name] = _captured(folder, records['vision_files'][name], 'vision/' + name, digest)
    source_vision, current_vision = parent['vision_source'], current_parent['vision_source']
    if (source_vision.get('captured_in_this_invocation') is not True
            or source_vision.get('detections_sha256') != VISION_SHA256['detections.json']
            or current_vision.get('captured_in_this_invocation') is not False
            or current_vision.get('detections_sha256') != source_vision['detections_sha256']):
        raise ValueError('Saved original capture provenance cannot be reported as a new capture')
    binding = strict_json(_captured(folder, records['visual_binding'], 'visual_binding.json', BINDING_SHA256))
    document = strict_json(raw_vision['detections.json'])
    target_surface = select_surface(document, events[0]['target']['index'])
    if (binding != old['visual_binding'] or target_surface != binding['target']
            or target_surface != events[0]['target'] or target_surface != parent.get('target')
            or target_surface['class_name'] != 'black_cyl_battery'
            or target_surface['world_transform_sha256'] != CALIBRATION_SHA
            or binding.get('scope') != 'fresh_visual_same_qualified_upper_same_unmoved_shell_only'
            or binding.get('same_unmoved_slot_shell_mounts_and_scene_reconfirmed') is not True
            or binding.get('automatic_other_slot_transfer') is not False
            or binding.get('automatic_lateral_motion') is not False
            or binding.get('visual_captured_this_invocation') is not True
            or binding.get('tool_rpy_rad') != upper['pose'][3:]
            or math.dist([v / 1000 for v in binding['mapped_TCP_mm']], target[:3]) > .0005):
        raise ValueError('Original real RGB-D target and same-slot visual binding differ')
    return dict(worker=old, parent=parent, source_records=records,
        terminal=copy.deepcopy(terminal), qualified_upper=copy.deepcopy(upper),
        segment=copy.deepcopy(segment), visual_binding=copy.deepcopy(binding),
        confirmation=copy.deepcopy(confirmation))


class TerminalResumeCycle(SupportedCycle):
    def __init__(self, *args, terminal_context, **kwargs):
        super().__init__(*args, **kwargs)
        if (self.taught_only or not self.allow_protected_opening
                or self.enable_arm_protection or self.resume_report is not None
                or self.completed_corridor is not None):
            raise ValueError('Terminal continuation requires isolated visual read-existing-configuration mode')
        self.terminal_context = terminal_context
        self.qualified_upper = copy.deepcopy(terminal_context['qualified_upper'])

    def wait_input(self, index, phase, arm, values, teaching=False):
        event = super().wait_input(index, phase, arm, values, teaching=teaching)
        if index in (5, 7) and event.get(SCENE_FLAG) is not True:
            raise RuntimeError('Current same shell/slot/scene and upper-clearance evidence not reconfirmed')
        return event

    def run(self):
        context, old = self.terminal_context, self.terminal_context['worker']
        initial, tip = self.preflight(None, PREPARE)
        arm_shape(initial)
        detail = pose_comparison(initial, context['terminal'])
        checks = dict(completed_descent_endpoint_pose=detail['matched'],
            full_limits=detail['full_limits_equal'],
            sdk_identity=metadata_signature(self.report['sdk_environment']) == metadata_signature(old['sdk_environment']),
            actual_DH_and_identity=model_signature(self.report['fresh_model']) == model_signature(old['fresh_model']),
            hand_configuration=self.report['configuration'] == old['configuration'],
            tip_parameters=tip == old['live_tip_parameters'])
        checks.update({name: self.report['arm_safety_reads'][name] == old['arm_safety_reads'][name]
            for name in SAFETY_CONFIGURATION})
        self.report['terminal_resume_start_qualification'] = dict(
            anchor_source='successful_full_feedback_endpoint_of_actual_descending_MoveL',
            current_vs_completed_endpoint=detail, checks=checks,
            failed_checks=[name for name, passed in checks.items() if not passed],
            rejected_fault_sample_used_as_anchor=False, old_taught_wrist_used_as_target=False,
            source_fault_sample_diagnostic_only=copy.deepcopy(old.get('last_arm_state')),
            no_automatic_old_point_alignment=True, tolerances_unchanged=True)
        self.save()
        failed = self.report['terminal_resume_start_qualification']['failed_checks']
        if failed:
            raise RuntimeError('Saved-terminal start qualification failed: ' + ','.join(failed)
                + '; actual/source differences saved; no automatic old-point alignment')
        _prepare_profile(self.report['last_preflight'], self.profile)
        verify_bound_descent(self, initial, self.qualified_upper,
            context['visual_binding']['mapped_TCP_mm'], context['segment']['plan'])
        self.report.update(scope='continued_same_slot_visual_grasp_release_empty_return',
            terminal_resume_source_accepted=True, terminal_resume_source_records=context['source_records'],
            actual_terminal_reference=copy.deepcopy(context['terminal']),
            inherited_completed_descent_segment=copy.deepcopy(context['segment']),
            source_descent_is_not_this_invocation_actuator_write=True,
            taught_goal=copy.deepcopy(old['taught_goal']), upper=copy.deepcopy(self.qualified_upper),
            current_qualified_upper_reference=copy.deepcopy(self.qualified_upper),
            high_return_operator_confirmation=copy.deepcopy(context['confirmation']),
            visual_binding=copy.deepcopy(context['visual_binding']),
            visual_binding_origin='reviewed_192922_source_capture_not_this_invocation',
            visual_binding_created=False, previous_vision_reused=True,
            vision_captured_this_invocation=False, vision_used=True,
            vision_binding_scope='continued_same_unmoved_slot_from_completed_visual_descent',
            camera_extrinsics_used_for_motion=False,
            historical_target_or_corridor_reused=True,
            manual_reteaching_required=False, manual_upper_required=False,
            board_clearance_mm=old['board_clearance_mm'], historical_clearances_reused=True,
            board_clearance_is_not_battery_clearance=True, battery_side_clearance_mm=None,
            board_clearance_source='original_noncontact_endpoint_pending_new_g_same_scene_confirmation',
            measured_upper_whole_hand_top_clearance_mm=25.3,
            measured_upper_clearance_variation_mm=.1,
            measured_upper_minimum_reported_clearance_mm=25.2,
            hand_top_clearance_measurement_basis=MEASUREMENT_BASIS,
            hand_above_target_top_confirmed=False, lateral_transfer_ready=False,
            automatic_other_slot_transfer=False, automatic_lateral_motion=False,
            no_automatic_old_point_alignment=True,
            source_stop_ACK_is_not_physical_stop=True)
        self.authority.prepare_feedback_pass = True
        self.plan = None
        self.save()
        self.finish_from_pregrasp(initial, self.qualified_upper, tip)
        self.report['outcome'] = TERMINAL_SUCCESS
        self.report['continued_without_repeat_descent'] = True
        self.report['inherited_visual_capture_not_recaptured'] = True
        self.save()

    def verify_return_clearance(self, final):
        for index in (5, 7):
            rows = [row for row in self.report.get('operator_inputs', []) if row.get('index') == index]
            if len(rows) != 1 or rows[0].get(SCENE_FLAG) is not True:
                raise RuntimeError('Current same-scene confirmation missing before return acceptance')
        from vision_high_return_cycle import verify_qualified_upper_return
        return verify_qualified_upper_return(self, final, self.qualified_upper)

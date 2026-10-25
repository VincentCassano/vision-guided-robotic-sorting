"""Fresh-start visual pickup or source-qualified held-shell black-bin delivery.

Uses actual044001 hand/wrist/lift evidence. No reset or configuration writes.
"""
import argparse
import copy
import ctypes
import fcntl
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys
import time
import traceback
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/battery_grasp/src'))
from inspect_left_arm import QUERIES, snapshot
from new_grasp_reference_worker_v1 import (SDK_DEFAULT, NATIVE_SHA, arm_from_queries,
    bracket_delta, json_evidence, source_audit, vector)
from arm_protection_transport import ProtectionClient
from visual_pick_sort_v15_hand import GroupHandBus
from school_device_identity import identity_fields
from grasp_rebuild_records_v1 import arm_reference, hand_snapshot, targets
from sdk_environment import collect_sdk_environment
from visual_run_configuration_v3 import (observe_configuration, bind_or_require_frames,
    require_current_configuration)
from visual_pick_sort_v19_source import accepted_request, bin_rows, SUCCESS_SHA
from visual_pick_sort_v18_hand import open_posture
from high_trial_tip_readonly import collect as collect_tip_parameters


class HandHealthPending(Exception):
    """A bounded stationary wait ended without granting automatic motion."""



def encode(value):
    # Get_Install_Pose returns c_float scalars rather than Python floats.
    # Preserve their actual .value recursively before JSON or vector checks.
    if isinstance(value, ctypes._SimpleCData):
        return encode(value.value)
    if isinstance(value, ctypes.Structure):
        return {name: encode(getattr(value, name)) for name, *_ in value._fields_}
    if isinstance(value, bytes):
        return value.decode('utf8', errors='replace')
    if isinstance(value, (list, tuple, ctypes.Array)):
        return [encode(item) for item in value]
    if isinstance(value, dict):
        return {key: encode(item) for key, item in value.items()}
    return value


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_bytes())


def delivery(path):
    inventory = {}
    for line in path.read_text(encoding='ascii').splitlines():
        if not line.strip():
            continue
        expected, name = line.split('  ', 1)
        item = (ROOT/name).resolve()
        if (ROOT not in item.parents or name in inventory or len(expected) != 64
                or sha(item) != expected):
            raise ValueError('Delivery file differs: '+name)
        inventory[name] = expected
    if 'deployment/visual_pick_sort_v19_worker.py' not in inventory:
        raise ValueError('Motion worker is missing from delivery manifest')
    return inventory


def pose_delta(a, b):
    # bracket_delta also computes SO(3), avoiding Euler wraparound comparisons.
    return bracket_delta(a, b)


def match_arm(actual, expected, label):
    delta = pose_delta(expected, actual)
    if (not delta['frame_signatures_unchanged'] or not delta['limits_unchanged']
            or max(abs(v) for v in delta['joint_degrees']) > .08
            or delta['position_distance_m'] > .0005
            or delta['orientation_angle_rad'] > .005):
        raise RuntimeError(label+': current state differs from this recorded state; '+repr(delta))
    return delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--manifest', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    robot = bus = run = lock = None
    active_motion = False
    stop_attempted = False
    external_drag = False
    mode = None
    connected = None
    report = dict(schema_version=1, scope='automatic_visual_support_postdescent_grouped_v19',
        timestamp_utc=datetime.now(timezone.utc).isoformat(), phase='preflight',
        outcome='not_completed', physical_object_success=False, execution_ready=False,
        arm_actuator_writes=0, hand_register_writes=0, arm_stop_writes=0,
        controller_configuration_writes=0, physical_stop_verified=False,
        old_fault_last_arm_state_used=False, offline_project_tests_executed=False,
        SDK_cleanup_finished=False, exceptions=[], cleanup_errors=[], sdk_calls=[],
        motion_requests=[], hand_commands=[], arm_samples=[],
        hand_actual_arrival_required=False,normal_status5_is_fault=False,
        initial_open_actual_arrival_required=True,object_absence_verified=False,
        manual_empty_confirmation_required=False,
        pregrasp_after_descent_only=True,grouped_PRE6_GRIP3_OPEN5=True,
        tolerances=dict(joints_degrees=.08, endpoint_m=.0005,
            orientation_rad=.005, lift_xy_m=.001, ordinary_reserve_degrees=3,
            drive_reserve_degrees=3))

    def save():
        temporary = output.with_name(output.name+'.pending')
        temporary.write_bytes((json.dumps(json_evidence(report), ensure_ascii=False,
            indent=2, allow_nan=False)+'\n').encode('utf8'))
        temporary.replace(output)

    def append(kind, row):
        nonlocal external_drag
        if row.get('event') == 'external_drag_teach_notification_not_hand_ack':
            external_drag = True
        if connected is not None and row.get('event') == 'external_arm_telemetry_ignored_by_manual_hand_tool':
            event=row['notification'];index=len(connected['events'])
            if index>=3 or event.get('trajectory_state') is not True or event.get('trajectory_connect')!=[1,1,0][index]:
                connected['error']='Connected completion notifications differ from exact1/1/0'
            else:connected['events'].append(copy.deepcopy(event))
        path = output.with_name(output.stem+'_'+kind+'.jsonl')
        with path.open('a', encoding='utf8', newline='\n') as stream:
            stream.write(json.dumps(json_evidence(row), ensure_ascii=False,
                allow_nan=False)+'\n')

    def phase(name):
        report['phase'] = name
        save()
        print('PHASE:', name, flush=True)

    def check_abort():
        if (output.parent/'abort.json').exists():
            report['parent_abort_request_file'] = str(output.parent/'abort.json')
            raise RuntimeError('Parent requested this action stop; no automatic continuation')

    def call(name, *values, **keywords):
        allowed = set(QUERIES) | {'Get_Joint_Drive_Min_Pos', 'Get_Joint_Drive_Max_Pos',
            'Get_Joint_EN_State', 'Get_Joint_Err_Flag', 'Get_Install_Pose',
            'Get_Joint_Current', 'Get_Joint_Temperature', 'Get_Joint_Voltage',
            'Movel_Cmd', 'Movej_Cmd', 'Movej_P_Cmd', 'Move_Stop_Cmd'}
        if name not in allowed:
            raise RuntimeError('SDK capability not granted: '+name)
        row = dict(method=name, monotonic=time.monotonic(), args=values, kwargs=keywords)
        report['sdk_calls'].append(row)
        try:
            raw = getattr(robot, name)(*values, **keywords)
            row['return_type'] = type(raw).__name__
            if isinstance(raw, (tuple, list)):
                row['return_element_types'] = [type(item).__name__ for item in raw]
            row['result'] = encode(raw)
            append('sdk', row)
            return row['result']
        except BaseException as exc:
            row['error'] = repr(exc)
            append('sdk', row)
            raise

    class Reads:
        def __getattr__(self, name):
            if name not in QUERIES:
                raise AttributeError(name)
            return lambda: call(name)

    def protection(label):
        client = ProtectionClient(lambda row: append('protection', dict(label=label, **row)))
        result = {key: getattr(client, key)() for key in
            ('Get_Collision_Stage', 'Get_Self_Collision_Enable')}
        expected = {'Get_Collision_Stage': [0, 8], 'Get_Self_Collision_Enable': [0, True]}
        report[label] = result
        if result != expected:
            raise RuntimeError('Current protection must remain collision8 / selfTrue: '+repr(result))
        return result

    def good_query(name, size):
        raw = call(name)
        if (not isinstance(raw, list) or len(raw) != size+1
                or type(raw[0]) is not int or raw[0] != 0):
            raise RuntimeError('Required SDK read unavailable: '+name+' '+repr(raw))
        return raw[1:]

    def full_arm(label):
        raw = snapshot(Reads())
        report.setdefault('arm_full_queries', []).append(dict(label=label, raw=raw))
        state = arm_from_queries(raw)
        bind_or_require_frames(run, state['frame_signatures'])
        report['last_arm_state'] = copy.deepcopy(state)
        save()
        return state

    def live_arm(template):
        if external_drag:
            raise RuntimeError('External drag teaching occurred during automatic motion')
        raw = call('Get_Current_Arm_State')
        report['last_arm_raw'] = copy.deepcopy(raw)
        if (not isinstance(raw, list) or len(raw) != 5
                or type(raw[0]) is not int or raw[0] != 0):
            raise RuntimeError('Current arm communication is not qualified: '+repr(raw))
        q, p = vector(raw[1], 6, 'live joints'), vector(raw[2], 6, 'live pose')
        if any(type(v) is not int or v != 0 for v in raw[3:]):
            raise RuntimeError('Actual arm/system fault; no automatic recovery: '+repr(raw[3:]))
        result = copy.deepcopy(template)
        result.update(joints=q, pose=p, arm_error=0, system_error=0)
        result['reserve'] = [min(v-a, b-v) for v, a, b in
            zip(q, result['limits']['minimum'], result['limits']['maximum'])]
        report['last_arm_state'] = copy.deepcopy(result)
        sample = dict(monotonic=time.monotonic(), phase=report['phase'], arm=result)
        report['arm_samples'].append(sample)
        append('sdk', dict(event='fresh_arm_sample', **sample))
        return result

    def hand_row(label):
        row = bus.refresh()
        report.setdefault('hand_snapshots', []).append(dict(label=label, row=row))
        hand_snapshot(row, label)
        if row.get('configuration_matched') is not True or row.get('blockers'):
            raise RuntimeError('Current hand configuration/working reads unavailable')
        if external_drag:
            raise RuntimeError('External drag teaching observed during action stage')
        save()
        return row

    def check_targets(current, expected, label):
        if len(current) != 6 or any(abs(a-b) > 2 for a, b in zip(current, expected)):
            raise RuntimeError(label+': current target registers differ; '+repr(current))

    def hand_working(expected):
        # Use the same validated transport and live working registers. Keep all
        # six actual target/position/current/status values, including normal5.
        check_abort()
        io = bus._io
        if io is None or io.broken or bus._logging_error:
            raise RuntimeError('Current hand connection/logging is unavailable')
        row = dict(identity=io.read(1000, 8), targets=io.read(1135, 6), positions=io.read(1145, 6),
            current=io.read(1105, 5)+[io.single(1110)], status=io.read(1085, 6),
            monotonic=time.monotonic())
        report['last_hand_working'] = row
        append('hand', dict(event='motion_hand_working_registers', **row))
        check_targets(row['targets'], expected, 'hand target readback')
        if (any(type(v) is not int or v not in (0, 1, 2, 5) for v in row['status'])
                or any(c < 0 or c > limit for c, limit in
                    zip(row['current'], bus._profile['current_limit_ma']))):
            raise RuntimeError('Actual hand protection/current failure; normal status5 is permitted')
        if external_drag:
            raise RuntimeError('External drag teaching observed during action stage')
        return row

    def wait_stationary_hand_health(initial,expected,label):
        # This is read-only continuation of the SAME acknowledged target,
        # never an override of a protection state or a grip retry. A transient
        # hardware status3 must actually disappear before any arm movement.
        allowed={'prepare_after_descent':'preparing_after_descent_at_visual_target',
            'grip_together':'grouped_grip_then_immediate180mm_lift'}
        original_phase=report['phase']
        if (active_motion or allowed.get(label)!=original_phase
                or not report.get('qualified_descent_endpoint')):
            raise RuntimeError('Read-only hand settling is limited to this stationary postdescent closure')
        anchor=copy.deepcopy(report['qualified_descent_endpoint'])
        event=dict(source='actual_controller_status3_after_acknowledged_postdescent_group',
            label=label,original_phase=original_phase,initial=copy.deepcopy(initial),
            expected_targets=expected[:],anchor_source='qualified_descent_endpoint_not_fault_last_arm_state',
            started_monotonic=time.monotonic(),samples=[],resumed=False,
            arm_writes_during_wait=0,hand_writes_during_wait=0,protection_clear_writes=0,
            target_retry_writes=0,manual_continue_key_required=False)
        report.setdefault('stationary_hand_health_events',[]).append(event)
        phase('waiting_stationary_hand_health_without_writes')
        print('手部暂时保护：原位读取等待，不重发、不清保护；恢复后自动续接。Ctrl+C可中止。',flush=True)
        previous=None;healthy_since=None;healthy_groups=0
        deadline=event['started_monotonic']+45.
        event['automatic_resume_window_seconds']=45.
        while True:
            check_abort()
            if time.monotonic()>=deadline:
                event.update(automatic_continuation_disabled=True,wait_window_elapsed=True,
                    finished_monotonic=time.monotonic())
                save()
                raise HandHealthPending('Hand protection did not recover in the stationary read-only window')
            io=bus._io
            if io is None or io.broken or bus._logging_error or external_drag:
                raise RuntimeError('Hand wait lost the original communication/exclusive scope')
            row=dict(identity=io.read(1000,8),targets=io.read(1135,6),
                positions=io.read(1145,6),status_before=io.read(1085,6),
                current=io.read(1105,5)+[io.single(1110)],status=io.read(1085,6),
                monotonic=time.monotonic())
            check_targets(row['targets'],expected,'unchanged acknowledged targets during read-only wait')
            codes=row['status_before']+row['status']
            if (any(type(v) is not int or v not in (0,1,2,3,5) for v in codes)
                    or any(c<0 or c>limit for c,limit in zip(row['current'],bus._profile['current_limit_ma']))):
                event['nonrecoverable_live_read']=copy.deepcopy(row);save()
                raise RuntimeError('A new force/status/current fault prevents this read-only continuation: '+repr(row))
            actual=live_arm(anchor);require_normal(actual,drive)
            row['arm_unchanged']=match_arm(actual,anchor,'arm stationary while hand protection settles')
            steady=all(v in (2,5) for v in codes)
            positions_steady=(previous is None or
                max(abs(a-b) for a,b in zip(row['positions'],previous['positions']))<=150)
            if steady and positions_steady:
                if healthy_since is None:healthy_since=time.monotonic()
                healthy_groups+=1
            else:healthy_since=None;healthy_groups=0
            row['healthy_groups']=healthy_groups
            event['samples'].append(copy.deepcopy(row))
            report['last_hand_working']=copy.deepcopy(row)
            append('hand',dict(event='stationary_hand_health_read_only',**row));save()
            if healthy_groups>=3 and time.monotonic()-healthy_since>=1. and time.monotonic()<deadline:
                fresh=stable_current('after_stationary_hand_health')
                match_arm(fresh,anchor,'qualified descent retained throughout hand settling')
                require_normal(fresh,drive)
                row['terminal_status_after_arm_bracket']=io.read(1085,6)
                append('hand',dict(event='stationary_hand_health_terminal_status',
                    status=row['terminal_status_after_arm_bracket'],monotonic=time.monotonic()))
                if any(v not in (2,5) for v in row['terminal_status_after_arm_bracket']):
                    healthy_since=None;healthy_groups=0;previous=row
                    save();continue
                if time.monotonic()>=deadline:
                    event.update(automatic_continuation_disabled=True,wait_window_elapsed=True,
                        finished_monotonic=time.monotonic());save()
                    raise HandHealthPending('Hand health qualification exceeded its automatic resume window')
                event.update(resumed=True,finished_monotonic=time.monotonic(),
                    final_healthy_hand=copy.deepcopy(row),fresh_arm=copy.deepcopy(fresh),
                    healthy_group_count=healthy_groups,
                    resume_basis='three_real_healthy_stationary_reads_no_target_arrival_requirement',
                    original_status3_preserved=True)
                phase(original_phase)
                print('手部已自行恢复且健康回读通过，继续本轮提起投放。',flush=True)
                return row
            previous=row

    def hand_protection_read(label=None,expected=None):
        # Explicit target writes need no arrival wait, but never chain a new
        # motor command across an actual protection3/4 or overcurrent report.
        io = bus._io
        state = dict(identity=io.read(1000, 8), status=io.read(1085, 6),
            current=io.read(1105, 5)+[io.single(1110)], monotonic=time.monotonic())
        append('hand', dict(event='between_finger_protection_read', **state))
        report['last_hand_protection_read'] = state
        if (any(type(v) is not int or v not in (0, 1, 2, 5) for v in state['status'])
                or any(c < 0 or c > limit for c, limit in
                    zip(state['current'], bus._profile['current_limit_ma']))):
            names=['thumb','index','middle','ring','little','opposition']
            detail=dict(status=state['status'],sampled_current_ma=state['current'],
                current_limits_ma=bus._profile['current_limit_ma'],
                abnormal_status_channels=[dict(channel=i,name=names[i],status=s)
                    for i,s in enumerate(state['status']) if s not in (0,1,2,5)],
                status3_means_current_protection_not_normal_stuck5=True,
                later_sampled_current_does_not_disprove_prior_protection=True,
                phase=report['phase'],loaded_lift_commanded=any(
                    r.get('phase')=='lifting_live_grip180mm' for r in report['motion_requests']))
            report['hand_fault_detail']=detail
            save()
            only_status3=(any(v==3 for v in state['status'])
                and all(type(v) is int and v in (0,1,2,3,5) for v in state['status'])
                and all(0<=c<=limit for c,limit in zip(state['current'],bus._profile['current_limit_ma'])))
            if (only_status3 and expected is not None
                    and label in ('prepare_after_descent','grip_together') and not active_motion):
                return wait_stationary_hand_health(state,expected,label)
            raise RuntimeError('Hand current/status protection: '+repr(detail))
        if external_drag or bus._logging_error:
            raise RuntimeError('Exclusive hand command/recording scope lost')
        return state

    def write_shape(desired,before,label):
        desired=targets(desired);current=targets(before)
        io=bus._io
        if io is None or io.broken or bus._logging_error:
            raise RuntimeError('Current grouped hand connection is unavailable')
        if label=='prepare_after_descent':
            if report['phase']!='preparing_after_descent_at_visual_target' or not report.get('qualified_descent_endpoint'):
                raise RuntimeError('PRE6 is allowed only after this actual visual descent')
        elif label=='grip_together':
            if report['phase']!='grouped_grip_then_immediate180mm_lift' or not report.get('prepare_after_descent_targets'):
                raise RuntimeError('GRIP3 requires this postdescent PRE6')
        elif label=='normal_bin_open':
            if report['phase']!='opening_five_flex_at_current_black_bin' or not report.get('qualified_bin_endpoint'):
                raise RuntimeError('Five-flex release requires this actual final bin endpoint')
        elif label=='initial_stationary_open':
            if (mode not in ('park_empty','visual_cycle') or active_motion
                    or report['phase']!='opening_current_five_flex_before_arm_motion'
                    or request['scene_permission'].get('initial_stationary_group_open_authorized') is not True
                    or request['scene_permission'].get('initial_opening_and_inert_shell_drop_zone_clear') is not True):
                raise RuntimeError('Initial OPEN5 requires stationary arm and authorized clear opening/drop zone')
        elif label=='empty_travel_open':
            if mode not in ('park_empty','visual_cycle') or report.get('initial_open_posture',{}).get('qualified') is not True:
                raise RuntimeError('Travel requires fresh actual open-position qualification')
        else:raise RuntimeError('Unsupported hand phase')
        address,values=io.groups[label];count=len(values)
        if desired[:count]!=list(values) or any(abs(a-b)>2 for a,b in zip(desired[count:],current[count:])):
            raise RuntimeError('Grouped target values or retained channels differ')
        command=dict(label=label,address=address,num=count,channels=list(range(count)),
            targets=list(values),monotonic=time.monotonic(),attempted=False,acknowledged=False,
            one_group_packet=True,actual_arrival_waited=False)
        report['hand_commands'].append(command)
        if any(abs(a-b)>2 for a,b in zip(current,desired)):
            check_abort()
            if external_drag:raise RuntimeError('External teaching during automatic control')
            bus._serial+=1;io.grant_group(label,bus._serial)
            command['attempted']=True;report['hand_register_writes']+=1
            io.write_group(address,list(values));command['acknowledged']=True
            readback=io.read(1135,6);command['target_readback']=readback
            check_targets(readback,desired,'One complete grouped target write')
            hand_protection_read(label,desired)
        else:command['skipped_matching_targets']=True
        report[label+'_targets']=desired[:]
        report['current_hand_targets']=desired[:]
        append('hand',dict(event='grouped_shape_command',**command));save()
        return desired[:]

    def check_drive(state, drive):
        reserve = [min(v-a, b-v) for v, a, b in zip(state['joints'],
            drive['minimum'], drive['maximum'])]
        state['drive_reserve'] = reserve
        if any(v < 3 for v in reserve):
            raise RuntimeError('Current drive-limit reserve below3 degrees: '+repr(reserve))

    def require_normal(state, drive):
        check_drive(state, drive)
        if any(v < 3 for v in state['reserve']):
            raise RuntimeError('Current ordinary-limit reserve below3 degrees: '+repr(state['reserve']))

    def stable_current(label):
        before = full_arm(label+'_before')
        time.sleep(.15)
        after = full_arm(label+'_after')
        report[label+'_bracket_delta'] = match_arm(after, before, label+' stable bracket')
        return after

    def motion_endpoint(actual, destination, expected_joint_arm=None):
        expected=copy.deepcopy(actual)
        if expected_joint_arm is not None:
            return match_arm(actual,expected_joint_arm,'recorded upper endpoint')
        expected['pose']=destination[:]
        delta=pose_delta(expected,actual)
        if delta['position_distance_m']>.0005 or delta['orientation_angle_rad']>.005:
            raise RuntimeError('Current motion endpoint differs from its own command')
        return delta

    def move(method,destination,beginning,drive,hand_targets,label):
        nonlocal active_motion
        allowed={'parking_empty_at_verified_upper':'Movej_Cmd',
            'aligning_live_visual_target_at_high':'Movel_Cmd',
            'descending_live_visual_target':'Movel_Cmd','lifting_live_grip180mm':'Movel_Cmd'}
        if allowed.get(label)!=method:raise RuntimeError('No capability for this motion phase')
        expected_joint=None
        if label=='parking_empty_at_verified_upper':
            if mode not in ('park_empty','visual_cycle') or destination!=success['qualified_upper']['joints']:
                raise RuntimeError('Empty parking must use this actual successful upper joint pose')
            expected_joint=success['qualified_upper']
        elif label=='aligning_live_visual_target_at_high':
            if destination!=candidate['hover_tcp_candidate_m_rad']:raise RuntimeError('Hover target differs')
        elif label=='descending_live_visual_target':
            if destination!=candidate['pregrasp_tcp_candidate_m_rad']:raise RuntimeError('Descent target differs')
        else:
            if (destination[:2]!=beginning['pose'][:2] or destination[3:]!=beginning['pose'][3:]
                    or abs(destination[2]-beginning['pose'][2]-.180)>1e-6):
                raise RuntimeError('Loaded lift must be this fresh pose World-Z plus180mm')
        phase(label)
        if expected_joint is not None:
            close=(max(abs(a-b) for a,b in zip(beginning['joints'],destination))<=.08
                and pose_delta(expected_joint,beginning)['position_distance_m']<=.0005
                and pose_delta(expected_joint,beginning)['orientation_angle_rad']<=.005)
        else:
            expected=copy.deepcopy(beginning);expected['pose']=destination[:]
            close=(pose_delta(expected,beginning)['position_distance_m']<=.0005
                and pose_delta(expected,beginning)['orientation_angle_rad']<=.005)
        if close:
            if label=='parking_empty_at_verified_upper':
                report['empty_travel_open_parallel_with_move']=False
                report['parking_hand_open_before_motion']=True
            report.setdefault('skipped_matching_endpoints',[]).append(label);save();return beginning
        request=dict(method=method,destination=destination[:],v=5,trajectory_connect=0,r=0,
            block=False,start=copy.deepcopy(beginning),phase=label)
        report['motion_requests'].append(request);save();check_abort()
        active_motion=True;report['arm_actuator_writes']+=1
        began=time.monotonic()
        raw=call(method,destination,5,trajectory_connect=0,r=0,block=False)
        if type(raw) is not int or raw!=0:raise RuntimeError('Motion ACK refused or uncertain; no retry')
        if label=='parking_empty_at_verified_upper':
            report['empty_travel_open_parallel_with_move']=False
            report['parking_hand_open_before_motion']=True
        last_hand=time.monotonic();previous=None;quiet_since=None;groups=0
        while time.monotonic()-began<120:
            check_abort();state=live_arm(beginning);require_normal(state,drive)
            if method=='Movel_Cmd':
                angle=pose_delta(beginning,state)['orientation_angle_rad']
                if angle>.005:raise RuntimeError('Fixed wrist changed during current Cartesian motion')
                if label=='aligning_live_visual_target_at_high':
                    direction=[b-a for a,b in zip(beginning['pose'][:3],destination[:3])]
                    squared=sum(v*v for v in direction)
                    fraction=(sum((state['pose'][i]-beginning['pose'][i])*direction[i] for i in range(3))/squared
                        if squared>1e-18 else 0.)
                    near=[beginning['pose'][i]+max(0.,min(1.,fraction))*direction[i] for i in range(3)]
                    if math.dist(state['pose'][:3],near)>.001:
                        raise RuntimeError('Current high alignment left its actual1mm line corridor')
                elif (math.dist(state['pose'][:2],beginning['pose'][:2])>.001
                        or not min(beginning['pose'][2],destination[2])-.001<=state['pose'][2]<=max(beginning['pose'][2],destination[2])+.001):
                    raise RuntimeError('Current vertical command left its own1mm corridor')
            if expected_joint is not None:
                delta=pose_delta(expected_joint,state)
                reached=(max(abs(a-b) for a,b in zip(state['joints'],destination))<=.08
                    and delta['position_distance_m']<=.0005 and delta['orientation_angle_rad']<=.005)
            else:
                expected=copy.deepcopy(state);expected['pose']=destination[:];delta=pose_delta(expected,state)
                reached=delta['position_distance_m']<=.0005 and delta['orientation_angle_rad']<=.005
            quiet=previous is not None and max(abs(a-b) for a,b in zip(state['joints'],previous['joints']))<=.02 and math.dist(state['pose'][:3],previous['pose'][:3])<=.0002
            if reached and quiet:
                if quiet_since is None:quiet_since=time.monotonic()
                groups+=1
            else:quiet_since=None;groups=0
            if time.monotonic()-last_hand>=1:
                hand_working(hand_targets);last_hand=time.monotonic()
            if groups>=2 and time.monotonic()-quiet_since>=.1:
                final=live_arm(beginning);require_normal(final,drive)
                motion_endpoint(final,destination,expected_joint)
                if label=='lifting_live_grip180mm' and final['pose'][2]-beginning['pose'][2]<.1795:
                    raise RuntimeError('Actual lift endpoint is below179.5mm')
                active_motion=False
                request.update(qualified_endpoint=copy.deepcopy(final),elapsed_seconds=time.monotonic()-began,
                    intermediate_full_configuration_bracket=False,object_retention_sensor_verified=False)
                save();return final
            previous=state;save();time.sleep(.1)
        raise RuntimeError('Motion did not reach its current command endpoint within120 seconds')

    def deliver(beginning,drive,hand_targets):
        nonlocal active_motion,connected
        phase('connected_school_black_bin_transfer')
        rows,offset=bin_rows();report['bin_task_offset']=offset
        # Consume the completed preceding MoveL's notification while it still
        # belongs to that single motion, before creating the connected context.
        hand_protection_read()
        current=live_arm(beginning);match_arm(current,beginning,'loaded transfer start')
        connected=dict(events=[],error=None,acknowledgements=[],flags=[1,1,0])
        report['connected_transfer']=connected
        report.update(intermediate_endpoint_waits=False,keyboard_waits_while_loaded=0,
            blend_radius=0,nonzero_corner_velocity_guaranteed=False)
        check_abort();active_motion=True
        for row,flag in zip(rows,[1,1,0]):
            check_abort()
            command=dict(method='Movej_P_Cmd',destination=row['pose'][:],v=10,
                trajectory_connect=flag,r=0,block=False,phase=row['phase'])
            report['motion_requests'].append(command);report['arm_actuator_writes']+=1;save()
            rc=call('Movej_P_Cmd',row['pose'][:],10,trajectory_connect=flag,r=0,block=False)
            connected['acknowledgements'].append(rc)
            if type(rc) is not int or rc!=0:raise RuntimeError('Connected transfer ACK refused; no retry/queue flush')
        target=rows[-1]['pose'];began=time.monotonic();last_hand=began
        previous=None;quiet_since=None;groups=0
        while time.monotonic()-began<120:
            check_abort();state=live_arm(beginning);require_normal(state,drive)
            if connected.get('error'):raise RuntimeError(connected['error'])
            expected=copy.deepcopy(state);expected['pose']=target[:];delta=pose_delta(expected,state)
            reached=delta['position_distance_m']<=.0005 and delta['orientation_angle_rad']<=.005
            quiet=previous is not None and max(abs(a-b) for a,b in zip(state['joints'],previous['joints']))<=.02 and math.dist(state['pose'][:3],previous['pose'][:3])<=.0002
            if reached and quiet:
                if quiet_since is None:quiet_since=time.monotonic()
                groups+=1
            else:quiet_since=None;groups=0
            if time.monotonic()-last_hand>=1:
                hand_working(hand_targets);last_hand=time.monotonic()
            if groups>=3 and time.monotonic()-quiet_since>=.3:
                hand_working(hand_targets)
                if connected.get('error') or len(connected['events'])!=3:
                    raise RuntimeError('Final bin endpoint lacks complete ordered1/1/0 notifications')
                final=stable_current('final_bin_endpoint');require_normal(final,drive)
                motion_endpoint(final,target);active_motion=False
                report['qualified_bin_endpoint']=copy.deepcopy(final)
                connected=None
                phase('opening_five_flex_at_current_black_bin')
                opened=write_shape(drop,hand_targets,'normal_bin_open')
                current_hand=hand_row('final opened hand')
                check_targets(current_hand['snapshot']['targets'],drop,'final bin opening')
                report.update(final_arm=final,final_hand=current_hand,phase='completed',
                    outcome='black_bin_motion_and_open_completed_waiting_operator',
                    actual_pickup_and_correct_bin_not_inferred_from_arm_or_ACK=True,
                    return_to_scan_automatic=False)
                save();return final
            previous=state;save();time.sleep(.1)
        raise RuntimeError('Connected black-bin transfer endpoint deadline exceeded')

    try:
        save()
        report.update(request_file=str(args.request.resolve()), request_sha256=sha(args.request))
        request = read_json(args.request)
        mode=request.get('mode')
        report.update(mode=mode,request_file=str(args.request.resolve()),
            request_sha256=sha(args.request),delivery_manifest_sha256=sha(args.manifest),
            delivery_source_sha256=delivery(args.manifest.resolve()))
        if request.get('manifest_sha256')!=report['delivery_manifest_sha256']:
            raise ValueError('Request manifest SHA differs')
        success,candidate=accepted_request(request)
        report.update(success_source_sha256=SUCCESS_SHA,accepted_candidate=candidate,
            current_start_is_fresh_not_historical=True)
        wrist_arm=arm_reference(success['wrist_record']['arm'],'actual successful wrist',3)
        start_hand=hand_snapshot(success['start_record']['hand'],'recorded source hand')
        wrist_hand=hand_snapshot(success['wrist_record']['hand'],'recorded source pregrasp')
        grip=targets(success['grip_targets']);prepare=targets(success['prepare_targets'])
        drop=[0,0,0,0,0,grip[5]]
        lock=(ROOT/'captures/hand_stage.lock').open('a')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        report['sdk_provenance'] = source_audit(SDK_DEFAULT)
        # Raw configuration GETs accept a text stream, unlike ProtectionClient
        # and ManualHandBus, whose log arguments are callbacks.
        with output.with_name(output.stem+'_configuration.jsonl').open(
                'a', encoding='utf8', newline='\n') as configuration_wire:
            run = observe_configuration(configuration_wire)
        report['frozen_configuration'] = run
        protection('protection_before')
        tip=collect_tip_parameters(lambda row: append('tip_parameters',row))
        report['live_tip_parameters']=tip
        if (tip['line_speed_m_s']>.25 or tip['line_acc_m_s2']>1.6
                or tip['angular_speed_rad_s']>.6 or tip['angular_acc_rad_s2']>4):
            raise RuntimeError('Actual tip speed/acceleration exceeds recorded limits')
        package = types.ModuleType('battery_visual_pick_sort_v19_scoped_sdk')
        package.__path__ = [str(SDK_DEFAULT)]
        sys.modules[package.__name__] = package
        sdk = importlib.import_module(package.__name__+'.robotic_arm')
        robot = sdk.Arm.__new__(sdk.Arm)
        sdk.Arm.__init__(robot, sdk.RM65, '169.254.128.18')
        report['sdk_socket_state_raw'] = encode(robot.Arm_Socket_State())
        if type(report['sdk_socket_state_raw']) is not int or report['sdk_socket_state_raw'] != 0:
            raise RuntimeError('Current SDK socket unavailable')
        metadata = collect_sdk_environment(robot)
        report['sdk_environment'] = metadata
        if (not metadata.get('metadata_complete') or metadata.get('api_version') != '4.3.1'
                or metadata.get('loaded_native', {}).get('sha256') != NATIVE_SHA
                or metadata.get('controller_run_mode_raw', {}).get('data') != [1]):
            raise RuntimeError('Reviewed native/API/real run mode differs')
        initial = stable_current('fresh_startup')
        if (initial['frame_signatures'] != wrist_arm['frame_signatures']
                or initial['limits'] != wrist_arm['limits']):
            raise RuntimeError('Fresh frame/joint configuration differs from this cycle records')
        drive = dict(minimum=vector(good_query('Get_Joint_Drive_Min_Pos', 1)[0], 6, 'drive min'),
            maximum=vector(good_query('Get_Joint_Drive_Max_Pos', 1)[0], 6, 'drive max'))
        if any(a >= b for a, b in zip(drive['minimum'], drive['maximum'])):
            raise RuntimeError('Malformed drive limits')
        report['drive_limits'] = drive
        enabled = good_query('Get_Joint_EN_State', 1)[0]
        errors, brakes = good_query('Get_Joint_Err_Flag', 2)
        if enabled != [1]*6 or errors != [0]*6 or brakes != [0]*6:
            raise RuntimeError('Actual joint enable/error/brake state is not ready')
        install = good_query('Get_Install_Pose', 3)
        if any(abs(a-b) > .001 for a, b in zip(vector(install, 3, 'installation'), [0, 45, 0])):
            raise RuntimeError('Actual installation differs from reviewed0/45/0')
        for name in ('Get_Joint_Current', 'Get_Joint_Temperature', 'Get_Joint_Voltage'):
            report.setdefault('arm_safety_reads', {})[name] = vector(good_query(name, 1)[0], 6, name)
        bus = GroupHandBus(lambda row: append('hand', row),prepare,grip)
        connection = bus.connect_read()
        report['hand_connect_read'] = connection
        hand = hand_snapshot(connection, 'fresh hand startup')
        if connection.get('configuration_matched') is not True or connection.get('blockers'):
            raise RuntimeError('Actual hand full configuration differs')
        # This is a new exclusive run. Compare immutable identity to history;
        # the live transport enforces diagnostic transitions within this run.
        if any(identity_fields(hand.get('identity'))!=identity_fields(saved.get('identity'))
                for saved in (start_hand, wrist_hand)):
            raise RuntimeError('Recorded/fresh static hand identity or live diagnostic differs')
        for key in ('current_limit_ma', 'speeds', 'force_targets'):
            if hand.get(key) != wrist_hand.get(key) or hand.get(key) != start_hand.get(key):
                raise RuntimeError('Recorded/fresh hand configuration differs: '+key)
        # Configuration/hand reads may take seconds; scope all later writes to
        # another healthy current bracket after those reads, not its old sample.
        now = stable_current('after_hand_connection')
        match_arm(now, initial, 'exclusive startup while hand was being read')
        initial = now
        check_abort()
        report['fresh_startup'] = dict(arm=initial, hand=connection,
            source='fresh_healthy_current_bracket_not_fault_last_state')
        require_normal(initial,drive)
        if mode=='deliver_held':
            report['held_source_current_match']=match_arm(initial,success['qualified_upper'],
                'actual successful180mm held upper')
            check_targets(hand['targets'],grip,'current successful held grip')
            deliver(initial,drive,grip)
        else:
            # The operator requested posture-based handling, without repeated
            # e confirmations. No inference that an object is physically absent.
            posture=open_posture(hand)
            report['initial_open_posture_before']=posture
            if not posture['qualified']:
                phase('opening_current_five_flex_before_arm_motion')
                bus._io.groups['initial_stationary_open']=(1135,(0,0,0,0,0))
                opened=write_shape([0,0,0,0,0,hand['targets'][5]],hand['targets'],
                    'initial_stationary_open')
                deadline=time.monotonic()+12.
                while True:
                    current=hand_working(opened);posture=open_posture(current)
                    report.setdefault('initial_open_position_reads',[]).append(copy.deepcopy(posture))
                    if posture['qualified']:break
                    if time.monotonic()>=deadline:
                        raise RuntimeError('OPEN5 sent but actual five positions are not open; no arm motion; '+repr(posture))
                after_open=stable_current('after_initial_stationary_open')
                match_arm(after_open,initial,'arm stationary during initial OPEN5')
                initial=after_open;hand=current
            report['initial_open_posture']=posture
            report['object_absence_verified']=False
            save()
            travel=hand['targets'][:]
            parked=move('Movej_Cmd',success['qualified_upper']['joints'][:],initial,drive,
                travel,'parking_empty_at_verified_upper')
            # Only the freshly open posture travels; PRE is after descent.
            report['travel_hand_targets']=travel[:]
            if mode=='park_empty':
                report.update(phase='completed',outcome='empty_hand_at_recorded_scan_upper',
                    qualified_scan_upper=parked,final_hand=hand_row('open scan hand'))
            else:
                high=move('Movel_Cmd',candidate['hover_tcp_candidate_m_rad'][:],parked,
                    drive,travel,'aligning_live_visual_target_at_high')
                seated=move('Movel_Cmd',candidate['pregrasp_tcp_candidate_m_rad'][:],high,
                    drive,travel,'descending_live_visual_target')
                report['qualified_descent_endpoint']=copy.deepcopy(seated)
                phase('preparing_after_descent_at_visual_target')
                write_shape(prepare,travel,'prepare_after_descent')
                after_prepare=live_arm(seated);match_arm(after_prepare,seated,'arm unchanged during postdescent PRE6')
                phase('grouped_grip_then_immediate180mm_lift')
                write_shape(grip,prepare,'grip_together')
                fresh=live_arm(after_prepare);match_arm(fresh,after_prepare,'grip pose unchanged')
                upper_pose=fresh['pose'][:];upper_pose[2]+=.180
                upper=move('Movel_Cmd',upper_pose,fresh,drive,grip,'lifting_live_grip180mm')
                report['qualified_loaded_upper']=copy.deepcopy(upper)
                deliver(upper,drive,grip)

        save()
    except HandHealthPending as exc:
        report.setdefault('pauses',[]).append(dict(reason=str(exc),phase=report.get('phase')))
        report.update(phase='paused',outcome='hand_protection_wait_paused_without_motion',
            physical_object_success=False,automatic_continuation_disabled=True)
        save()
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc(),
            phase=report.get('phase')))
        report.update(failed_phase=report.get('phase'), phase='fault',
            outcome='worker_not_completed_no_automatic_retry', physical_object_success=False)
        if active_motion and robot is not None and not stop_attempted:
            stop_attempted = True
            report['arm_stop_writes'] += 1
            try:
                report['fault_stop_raw'] = call('Move_Stop_Cmd', block=False)
            except BaseException as stop_exc:
                report['fault_stop_error'] = repr(stop_exc)
        save()
    finally:
        if bus is not None:
            try:
                report['hand_close'] = bus.close()
            except BaseException as exc:
                report['cleanup_errors'].append(dict(method='manual_hand_close', error=repr(exc)))
        if robot is not None:
            for name in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try:
                    raw = encode(getattr(robot, name)())
                    report.setdefault('sdk_cleanup_results', []).append(dict(method=name, raw=raw))
                    if name == 'RM_API_UnInit' and (type(raw) is not int or raw != 0):
                        raise RuntimeError('API uninitialization returned nonzero/unknown status')
                except BaseException as exc:
                    report['cleanup_errors'].append(dict(method=name, error=repr(exc)))
            report['SDK_cleanup_finished'] = not report['cleanup_errors']
        if run is not None and (robot is None or report['SDK_cleanup_finished']):
            try:
                with output.with_name(output.stem+'_configuration.jsonl').open(
                        'a', encoding='utf8', newline='\n') as configuration_wire:
                    require_current_configuration(run, configuration_wire)
                protection('protection_after')
            except BaseException as exc:
                report['cleanup_errors'].append(dict(method='final_readonly_configuration', error=repr(exc)))
        if report['cleanup_errors'] and report.get('phase') == 'completed':
            report.update(failed_phase='final_cleanup_or_configuration', phase='fault',
                outcome='needs_onsite_recovery', physical_object_success=False)
            report.pop('qualified_upper', None)
            report.pop('qualified_upper_hand', None)
            report.pop('qualified_return', None)
            report.pop('qualified_return_hand', None)
        if lock is not None:
            try:lock.close()
            except BaseException as exc:report['cleanup_errors'].append(dict(method='lock.close',error=repr(exc)))
        report['closed_timestamp_utc'] = datetime.now(timezone.utc).isoformat()
        save()
    print('SAVED:', output, 'OUTCOME:', report['outcome'], flush=True)
    # A pause returns control to the parent UI, but its phase/outcome never
    # counts as a completed pickup or authorizes a later motion.
    return 0 if report['phase'] in ('completed','paused') and not report['exceptions'] and not report['cleanup_errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

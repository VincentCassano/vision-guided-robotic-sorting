#!/usr/bin/env python3
"""One explicitly confirmed empty return after a recorded supported opening."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import traceback
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SUCCESS = 'completed_empty_return_after_supported_release'
WAIT_PHASE = 'await_empty_return_after_release'


def installed():
    hashes = {}
    for line in (ROOT / 'deployment/vision-grasp-cycle-v20-files.sha256').read_text(encoding='ascii').splitlines():
        digest, name = line.split('  ', 1)
        path = (ROOT / name).resolve()
        if (ROOT.resolve() not in path.parents or name in hashes
                or hashlib.sha256(path.read_bytes()).hexdigest() != digest):
            raise ValueError('Installed payload differs: ' + name + '; no connection')
        hashes[name] = digest
    if not hashes:
        raise ValueError('Empty installed manifest; no connection')
    return hashes


def permission(folder, answer):
    path = folder / 'permissions/1.json'
    if path.exists():
        raise ValueError('Empty-return permission already used; no repeat')
    row = dict(index=1, phase=WAIT_PHASE, value=answer,
        shell_on_board_hand_completely_empty_and_original_corridor_clear=answer == 'u',
        scene_fixed_and_onsite_stop_ready=answer == 'u',
        onsite_actual_arm_stop_confirmed=answer == 'u',
        timestamp_utc=datetime.now(timezone.utc).isoformat())
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(row, ensure_ascii=False) + '\n', encoding='utf8')
    temporary.replace(path)
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grasp-report', type=Path, required=True)
    parser.add_argument('--release-report', type=Path, required=True)
    parser.add_argument('--resume-return-report', type=Path)
    for flag in ('execute', 'confirm-empty-clear-exclusive', 'confirm-onsite-stop', 'confirm-fixed-scene'):
        parser.add_argument('--' + flag, action='store_true')
    parser.add_argument('--sdk-dir', type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--output-parent', type=Path, default=ROOT / 'captures')
    args = parser.parse_args(argv)
    if not all((args.execute, args.confirm_empty_clear_exclusive,
                args.confirm_onsite_stop, args.confirm_fixed_scene)):
        parser.error('Empty clear hand, exclusive control, fixed scene and onsite stop required; no connection')
    if platform.system() != 'Linux' or platform.machine() != 'aarch64':
        parser.error('Reviewed onsite Linux aarch64 only; no connection')
    try:
        hashes = installed()
        source_bytes = dict(grasp=args.grasp_report.read_bytes(), release=args.release_report.read_bytes())
        if args.resume_return_report is not None:
            source_bytes['resume_return'] = args.resume_return_report.read_bytes()
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    folder = args.output_parent / ('empty_return_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    (folder / 'permissions').mkdir()
    (folder / 'sources').mkdir()
    sources = {}
    for role, raw in source_bytes.items():
        path = folder / ('sources/' + role + '_worker_result.json')
        path.write_bytes(raw)
        sources[role] = dict(original_path=str(getattr(args, role + '_report')),
            captured_path=str(path.relative_to(folder)), byte_count=len(raw),
            sha256=hashlib.sha256(raw).hexdigest())
    report = dict(schema_version=1, scope='explicit_empty_return_after_supported_release',
        installed_file_sha256=hashes, sources=sources, sdk_in_parent=False,
        offline_tests_executed=False, exceptions=[], worker_cleanup_timeout=False,
        hand_register_writes_allowed=False, controller_configuration_writes_allowed=False,
        load_lift_or_transfer_allowed=False, automatic_retry=False)
    report['explicit_resume_return_requested'] = args.resume_return_report is not None

    def save():
        temporary = folder / 'parent_result.tmp'
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')
        temporary.replace(folder / 'parent_result.json')

    def abort():
        path = folder / 'abort.json'
        if not path.exists():
            path.write_text('{"abort":true}\n', encoding='ascii')

    process = None
    native_hung = False
    prompted = False
    last_phase = None
    phase_began = time.monotonic()
    save()
    try:
        print('本次仅空手沿原World+Z通道返回上方点；不重新下降、不闭指、不写手/供电/保护。', flush=True)
        print('电池须留在板或固定支撑上，手已完全脱离且空手，退出其他控制，现场守急停。', flush=True)
        if args.resume_return_report is not None:
            print('显式续返：重新读实际当前点和静止组，须仍在原World-Z通道内；旧运动最后一帧不作当前点。', flush=True)
            print('旧停止返回0只作ACK，须现场确认实际停稳；不自动对齐旧点、不自动重试。', flush=True)
        else:
            print('当前点若与刚才松指终点不同，会零运动拒绝；不自动对齐旧点。', flush=True)
        command = [sys.executable, str(ROOT / 'deployment/return_after_release_worker.py'),
            '--folder', str(folder), '--sdk-dir', str(args.sdk_dir), '--execute']
        if args.resume_return_report is not None:
            command.append('--resume-return-report')
        report['worker_process'] = dict(command=command, stdin='DEVNULL', close_fds=True)
        save()
        with (folder / 'process_stdout.log').open('wb') as out, (folder / 'process_stderr.log').open('wb') as err:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=out, stderr=err, close_fds=True)
            while process.poll() is None:
                path = folder / 'worker_result.json'
                if path.exists():
                    state = json.loads(path.read_text(encoding='utf8'))
                    phase = state.get('phase')
                    if phase != last_phase:
                        last_phase = phase
                        phase_began = time.monotonic()
                        print('PHASE:', phase, flush=True)
                    limit = {'preflight': 90, 'fresh_before_empty_return': 90,
                        'preflight_returning_empty': 120, 'returning_empty': 90,
                        'reading_current_quiet_groups': 90,
                        'completed': 15, 'fault': 15, 'cancelled': 15}.get(phase)
                    if limit is not None and time.monotonic() - phase_began > limit:
                        raise TimeoutError('Empty-return worker phase deadline: ' + str(phase))
                    if phase == WAIT_PHASE and not prompted:
                        if state.get('permission_index') != 1:
                            raise RuntimeError('Empty-return worker permission mismatch')
                        prompted = True
                        print('本次完整前检通过；输入等待期间不查询SDK、不保持手TCP连接。', flush=True)
                        print('不是提起电池：只有确认空手后才输入u；不满足就q保持退出。', flush=True)
                        while True:
                            answer = input('现场确认臂已实际停稳，电池留在板上、手已完全脱离且空手、原上升通道无遮挡、场景未变、守急停。u=空手返回；q=退出：').strip().lower()
                            if answer in ('u', 'q'):
                                break
                            print('只输入u或q。', flush=True)
                        current = json.loads(path.read_text(encoding='utf8'))
                        if (process.poll() is not None or current.get('phase') != WAIT_PHASE
                                or current.get('permission_index') != 1):
                            raise RuntimeError('Worker changed during input; stale permission refused')
                        report['operator_input'] = permission(folder, answer)
                        save()
                elif time.monotonic() - phase_began > 30:
                    raise TimeoutError('Empty-return worker did not produce startup report')
                time.sleep(.2)
    except (Exception, KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
        abort()
        print('已禁止后续命令；接触或继续运动立即现场硬件停止。程序退出不表示停止或断扭矩。', flush=True)
        if process is not None and process.poll() is None:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    native_hung = True
                    report['worker_cleanup_timeout'] = True
                    print('SDKworker未确认退出；现场硬件停止，不能把杀进程当停机。', flush=True)
    finally:
        if process is not None:
            report['worker_return_code'] = process.poll()
        path = folder / 'worker_result.json'
        if path.exists():
            report['worker_result'] = json.loads(path.read_text(encoding='utf8'))
        outcome = report.get('worker_result', {}).get('outcome', 'worker_result_missing')
        if native_hung:
            outcome = 'SDK_worker_unresponsive_hardware_stop_required'
        report['outcome'] = outcome
        save()
        bundle = folder.with_suffix('.zip')
        with zipfile.ZipFile(bundle, 'x', zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(folder.rglob('*')):
                if path.is_file():
                    archive.write(path, str(Path(folder.name) / path.relative_to(folder)))
        print('SAVED:', bundle, 'OUTCOME:', outcome, flush=True)
        errors = report.get('worker_result', {}).get('exceptions', [])
        if errors:
            print('BLOCK:', errors[-1]['error'], flush=True)
        if outcome == SUCCESS:
            print('已记录一次空手World+Z返回反馈；未重新抓取或提起电池，未宣称完整视觉闭环成功。', flush=True)
        elif outcome == 'cancelled_no_empty_return':
            print('保持当前点退出，没有返回命令。', flush=True)
        else:
            print('未完成空手返回；完整参数、原报告、原始通信和异常已统一保存。', flush=True)
    return 0 if report.get('outcome') == SUCCESS and not report['exceptions'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

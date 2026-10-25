#!/usr/bin/env python3
"""Explicit empty-hand withdrawal to four-channel PREPARE; no arm motion."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import signal
import shlex
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SUCCESS = 'completed_empty_hand_position_release_to_PREPARE'
OPENING_SUCCESS = 'completed_empty_hand_four_channel_prepare_feedback_recorded'
WAIT_PHASE = 'await_supported_position_release'


def installed():
    hashes = {}
    manifest = ROOT / 'deployment/grasp-rebuild-open-v1-files.sha256'
    for line in manifest.read_text(encoding='ascii').splitlines():
        digest, name = line.split('  ', 1)
        path = (ROOT / name).resolve()
        if (ROOT.resolve() not in path.parents or name in hashes
                or hashlib.sha256(path.read_bytes()).hexdigest() != digest):
            raise ValueError('Installed payload differs: ' + name + '; no connection')
        hashes[name] = digest
    if not hashes:
        raise ValueError('Empty installed manifest; no connection')
    return hashes


def write_permission(folder, answer):
    target = folder / 'permissions/1.json'
    if target.exists():
        raise ValueError('Release permission already used; no repeat')
    row = dict(index=1, phase=WAIT_PHASE, value=answer,
        empty_hand_and_opening_clear=answer == 'r',
        timestamp_utc=datetime.now(timezone.utc).isoformat())
    temp = target.with_suffix('.tmp')
    temp.write_text(json.dumps(row, ensure_ascii=False) + '\n', encoding='utf8')
    temp.replace(target)
    return row


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ('execute', 'confirm-empty-clear-exclusive', 'confirm-onsite-stop',
                 'confirm-exclusive-control', 'allow-protected-opening'):
        parser.add_argument('--' + flag, action='store_true')
    parser.add_argument('--sdk-dir', type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--output-parent', type=Path, default=ROOT / 'captures')
    args = parser.parse_args(argv)
    if not all((args.execute, args.confirm_empty_clear_exclusive,
                args.confirm_onsite_stop, args.confirm_exclusive_control)):
        parser.error('Empty hand, clear withdrawal sweep, onsite stop and exclusive control required; no connection')
    if platform.system() != 'Linux' or platform.machine() != 'aarch64':
        parser.error('Reviewed onsite Linux aarch64 only; no connection')
    try:
        hashes = installed()
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    if not args.allow_protected_opening:
        parser.error('This recovery entry requires --allow-protected-opening; four flex channels only')
    log = ROOT / 'docs/执行命令日志.txt'
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('a', encoding='utf8') as stream:
        stream.write(datetime.now(timezone.utc).isoformat() + ' ' + shlex.join([sys.executable, str(Path(__file__).resolve())] + (list(argv) if argv is not None else sys.argv[1:])) + '\n')
    folder = args.output_parent / ('empty_prepare_recovery_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    (folder / 'permissions').mkdir()
    report = dict(schema_version=1, scope='empty_hand_four_channel_prepare_only',
        installed_file_sha256=hashes, sdk_in_parent=False, offline_tests_executed=False,
        exceptions=[], worker_cleanup_timeout=False, arm_motion_allowed=False,
        force_zero_allowed=False, automatic_retry=False,
        allow_protected_opening=args.allow_protected_opening,
        protection_cleared_claimed=False)

    def save():
        temp = folder / 'parent_result.tmp'
        temp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')
        temp.replace(folder / 'parent_result.json')

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
        print('若仍有喀喀声、持续转动或升温，立即现场硬件停止，不等待本程序。', flush=True)
        print('重建抓取姿势第一步：只将前四通道退到8000，机械臂不动。小拇指/对掌保持；这不代表整手完全张开。', flush=True)
        print('确认空壳已经取出，手内没有物件，向张开运动范围无遮挡；退出其他控制，现场守急停。', flush=True)
        if args.allow_protected_opening:
            print('本次显式选择仅张开恢复：仅前四通道各退到8000；小拇指/旋转不写。', flush=True)
            print('原前四路状态3只用于空手向张开恢复记录，不清保护；状态4、实际限流/身份/通信异常仍阻断。', flush=True)
        else:
            print('读取完整当前参数后才提示r。状态5单独记录；状态3/4、限流/身份/通信异常仍禁止松指命令。', flush=True)
        command = [sys.executable, str(ROOT / 'deployment/empty_prepare_recovery_worker.py'),
            '--folder', str(folder), '--sdk-dir', str(args.sdk_dir), '--execute']
        if args.allow_protected_opening:
            command.append('--allow-protected-opening')
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
                    limit = {'preflight': 90, 'fresh_before_release': 90,
                             'releasing_position': 90, 'completed': 15, 'blocked': 15,
                             'cancelled': 15}.get(phase)
                    if limit is not None and time.monotonic() - phase_began > limit:
                        raise TimeoutError('Release worker phase deadline: ' + str(phase))
                    if phase == WAIT_PHASE and not prompted:
                        if state.get('permission_index') != 1:
                            raise RuntimeError('Release worker permission mismatch')
                        prompted = True
                        print('新鲜前检通过；等待输入期间程序不查询SDK，也不发送手部命令。', flush=True)
                        if args.allow_protected_opening:
                            print('两组完整当前读数已记录。r仅写拇指弯曲/食指/中指/无名指8000，小拇指/旋转保持现有目标；不移动臂。', flush=True)
                        else:
                            print('r仅将手松至PREPARE，不全伸指，不旋回拇指，不提起或返回机械臂。', flush=True)
                        while True:
                            answer = input('确认手内无物、向张开范围无遮挡，退出示教/其他控制，双手退出、现场守急停。r=四路退到8000；q=保持退出：').strip().lower()
                            if answer in ('r', 'q'):
                                break
                            print('只输入r或q。', flush=True)
                        current = json.loads(path.read_text(encoding='utf8'))
                        if (process.poll() is not None or current.get('phase') != WAIT_PHASE
                                or current.get('permission_index') != 1):
                            raise RuntimeError('Worker changed during input; stale release permission refused')
                        report['operator_input'] = write_permission(folder, answer)
                        save()
                elif time.monotonic() - phase_began > 30:
                    raise TimeoutError('Release worker did not produce startup report')
                time.sleep(.2)
    except (Exception, KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc)))
        abort()
        print('已禁止后续命令；接触、异响或继续运动立即现场硬件停止。程序退出不表示断扭矩。', flush=True)
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
                    print('SDKworker未确认退出；现场硬件停止，不能把杀进程当成停机。', flush=True)
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
        if outcome == OPENING_SUCCESS:
            print('已记录前四通道退向8000且另外两通道保持本次读数；臂未移动。', flush=True)
            print('本次是空手PREPARE，不是全手伸直；未清保护，也未证明断扭矩。', flush=True)
        elif outcome == SUCCESS:
            print('已读到空手PREPARE；臂未移动；未证明扭矩释放。', flush=True)
        elif outcome == 'cancelled_no_position_write':
            print('已保持现存目标退出，没有松指命令。', flush=True)
        else:
            print('未确认退回PREPARE，不自动重试或运动；接触、异响或继续运动现场急停，完整读数已保存。', flush=True)
    return 0 if report.get('outcome') in (SUCCESS, OPENING_SUCCESS) and not report['exceptions'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

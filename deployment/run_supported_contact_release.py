#!/usr/bin/env python3
"""Explicit recovery of the audited154756 force-mode abort; no grasp/arm writes.

An inert shell must be supported independently throughout opening. This is a
new operator-authorized recovery session, never an automatic fault response.
Force-zero exits force control; it neither proves torque release nor guarantees
return to retained position targets. Position opening is gated by actual return.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import logging
from pathlib import Path
import socket
import time
import traceback
import zipfile

import run_supported_battery_contact as prior
from supported_contact_policy import ContactHand, PREPARE, FORCE_TARGETS, ZERO_FORCE, ACTIVE_CHANNELS
from supported_contact_transport import SupportedContactTransport, PhaseAuthority
from continuous_hand_cycle import RELEASE_PLAN
from run_school_battery_hand import TOL, TARGET_TOL

ROOT = Path(__file__).resolve().parents[1]
LOADED_NAME = 'supported_contact_loaded_154756.json'
LOADED_SHA = '09386a5a2ebfa3fdcca0428bd667c9f22dbdfac3dbb3c78a61aba4aa0a9edf28'
LATEST_RELEASE_NAME = 'supported_contact_release_162058.json'
LATEST_RELEASE_SHA = '4a1dbcfd5fbf9f590d5761db2fe25af1fd02f81044c778b932c0b28d91c4b25d'


class ReleaseTransport(SupportedContactTransport):
    """Keep strict framing; this entry can enter RELEASE only, exactly once."""
    def set_phase(self, phase, authority):
        if phase not in ('release','readonly'):
            self.latch_fault('Recovery cannot prepare, reset or enter contact')
            raise RuntimeError('Recovery permits release only')
        return super().set_phase(phase, authority)


class ReleaseHand(ContactHand):
    def _force_withdrawal_context(self, pending):
        authority = self.sequence.authority
        zero = (1115,ZERO_FORCE)
        return bool(self.report.get('force_exit_under_active_stall_authorized') is True
            and self.report.get('independent_support_confirmed_by_operator') is True
            and authority is getattr(self.io,'_phase_authority',None)
            and not authority._terminal_latched and not authority._completed
            and ((pending and self.io.phase == 'release'
                  and zero in authority._acknowledged_writes)
                 or (not pending and self.io.phase == 'readonly'
                     and zero not in authority._attempted_writes)))

    def recovery_snapshot(self, expected_force, anchor=None, withdrawal=False, exit_pending=False):
        """Record the complete raw-derived group BEFORE application rejection.

        Initial active positions are unknown/current, not a new approved grasp
        range. Post-zero observations must stay within that actual start and
        PREPARE. No cap is raised for a force or closing command.
        """
        began = self.clock()
        try:
            self._ready()
            if withdrawal or exit_pending:
                if (withdrawal and exit_pending or not self._force_withdrawal_context(exit_pending)
                        or withdrawal and (anchor is not None or tuple(expected_force) != FORCE_TARGETS)
                        or exit_pending and (anchor is None or tuple(expected_force) != ZERO_FORCE)):
                    raise RuntimeError('Force-withdrawal exception requires fresh entry-owned release authority')
            identity = self.read('recovery_identity',1000,8)
            status = self.read('recovery_status',1085,6)
            current = self.read('recovery_current_ma',1105,5)
            positions = self.read('recovery_positions',1145,6)
            rotation_current = self.io.single(1110)
            self.event('read',name='recovery_rotation_current_ma',address=1110,values=[rotation_current])
            targets = self.read('recovery_position_targets',1135,6)
            forces = self.read('recovery_force_targets',1115,5)
            feedback = self.read('recovery_force_feedback',1175,5)
            row = dict(identity=identity,status=status,current=current+[rotation_current],
                positions=positions,targets=targets,force_targets=forces,forces=feedback,
                started_monotonic=began,finished_monotonic=self.clock(),
                reads_are_sequential=True,physical_stop_verified=False,torque_release_verified=False)
            self.report['last_recovery_snapshot'] = copy.deepcopy(row)
            self.event('recovery_feedback_group',snapshot=row)
            self.save()
            self._identity_checked(identity)
            for values,length,label in ((status,6,'status'),(row['current'],6,'current'),
                    (positions,6,'position'),(targets,6,'position target'),
                    (forces,5,'force target'),(feedback,5,'force feedback')):
                self._u16(values,length,label)
            self.guard()
            if any(v not in range(6) or v == 3
                   or v == 5 and not (ch in ACTIVE_CHANNELS and (withdrawal or exit_pending))
                   for ch,v in enumerate(status)):
                raise RuntimeError('Protection/invalid recovery status; hardware stop, no write')
            if any(v == 4 and ch not in ACTIVE_CHANNELS for ch,v in enumerate(status)):
                raise RuntimeError('Unexpected force-control channel')
            if any(c > limit for c,limit in zip(row['current'],self.report['configuration']['current_limit_ma'])):
                raise RuntimeError('Controller current limit exceeded')
            if forces != list(expected_force):
                raise RuntimeError('Live force mode changed since audited abort; no write')
            if any(abs(a-b) > TARGET_TOL for a,b in zip(targets,PREPARE)):
                raise RuntimeError('Retained position target changed; no write')
            if any(abs(positions[ch]-PREPARE[ch]) > TOL for ch in range(2,6)):
                raise RuntimeError('Inactive finger moved; no write')
            if any(positions[ch] < PREPARE[ch]-TOL for ch in ACTIVE_CHANNELS):
                raise RuntimeError('Active finger below retained prepare target; no write')
            if anchor is not None and any(not max(0,min(a,b)-TOL) <= p <= min(65535,max(a,b)+TOL)
                    for p,a,b in zip(positions,anchor,PREPARE)):
                raise RuntimeError('Recovery moved outside actual-start to retained-target bounds')
            if 5 in status:
                self.report.setdefault('force_exit_stall_observations',[]).append(dict(
                    status=status[:],positions=positions[:],force_targets=forces[:],
                    scope='pending_readonly_exit' if exit_pending else 'one_zero_force_withdrawal_only',
                    status5_is_ignored_for_position_writes=False,physical_stop_verified=False))
                self.save()
            return row
        except BaseException:
            self._disable()
            raise

    def observe_force_exit(self, anchor, timeout=30):
        began, stable, since = self.clock(), 0, None
        while self.clock()-began < timeout:
            row = self.recovery_snapshot(ZERO_FORCE,anchor,exit_pending=True)
            reached = (not any(v in (3,4,5) for v in row['status'])
                       and all(abs(a-b) <= TOL for a,b in zip(row['positions'],PREPARE)))
            now = row['finished_monotonic']
            if reached:
                stable += 1
                if since is None: since = now
            else:
                stable, since = 0, None
            if reached and stable >= 3 and now-since >= 1 and now-began <= timeout:
                self._normal_release_ready, self._release_index = True, 0
                self.report['force_exit_return_observed'] = dict(final=row,groups=stable,
                    physical_stop_verified=False,torque_release_verified=False)
                self.save()
                self.report['force_exit_under_active_stall_authorized'] = False
                return row
        self._disable()
        raise RuntimeError('Force-zero ACK is not return; no automatic position opening after timeout')

    def write_pose(self, step, previous):
        if step not in RELEASE_PLAN:
            self._disable()
            raise RuntimeError('Recovery cannot send a grasp or prepare position')
        return super().write_pose(step,previous)


class ReleaseSequence(prior.SupportedSequence):
    def run(self, execute):
        if execute is not True:
            raise RuntimeError('Explicit supported release required')
        self.report.update(execution_scope='explicit_supported_force_exit_and_open_only',
            contact_recipe_withdrawn=True,automatic_fault_open=False)
        # Input gate BEFORE opening the hand TCP prevents stale broadcasts.
        prompt = ('先用固定支撑承托空壳全部重量，确认张开范围无遮挡、人员双手退出、现场急停就绪。'
                  'r=明确退出现存力控并张开；q=不发命令退出：')
        began = self.clock()
        answer = self.confirm(prompt).strip().lower()
        self.event('operator_input',prompt=prompt,value=answer,wait_seconds=self.clock()-began,monitored=False)
        if answer == 'q':
            self.report['outcome'] = 'cancelled_no_writes'
            self.save()
            return
        if answer != 'r': raise RuntimeError('Only explicit r authorizes recovery')
        self.report['independent_support_confirmed_by_operator'] = True
        # Permit WITHDRAWING the exact existing force target, not moving through
        # protection or writing any new position while protection persists.
        self.report['force_exit_under_active_stall_authorized'] = True
        self.fresh_arm(expected_role='pick_hover')
        hand = self.checked_hand()
        try:
            row = hand.recovery_snapshot(FORCE_TARGETS,withdrawal=True)
            self.report['recovery_start_actual_positions'] = row['positions'][:]
            self.report['initial_positions_are_not_approved_grasp_caps'] = True
            self.save()
            self.permit('release')
            self.io.write_group(1115,list(ZERO_FORCE))
            self.report['force_exit_command_acknowledged'] = True
            self.save()
            hand.observe_force_exit(row['positions'])
            hand.release_stages(list(PREPARE))
            final = hand.preflight(expected=self.profile['poses']['open'])
            self.report.update(outcome='completed_explicit_supported_open',final_hand=final,
                hand_open_feedback_pass=True,physical_stop_verified=False,torque_release_verified=False)
            self.io.set_phase('readonly',self.authority)
            self.hand_motion_enabled = False
            self.save()
        finally:
            self.report['force_exit_under_active_stall_authorized'] = False
            self.save()
            self.close_hand()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-supported-release',action='store_true')
    parser.add_argument('--confirm-inert-independent-support',action='store_true')
    parser.add_argument('--confirm-exclusive-control',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args = parser.parse_args(argv)
    if not all((args.execute_supported_release,args.confirm_inert_independent_support,
                args.confirm_exclusive_control,args.confirm_onsite_stop)):
        parser.error('Independent support + onsite stop + exclusive control required; no connection')
    directory = Path(__file__).resolve().parent
    def pinned(name,digest):
        raw = (directory/name).read_bytes()
        if hashlib.sha256(raw).hexdigest() != digest:
            raise ValueError('Pinned evidence mismatch: '+name)
        return json.loads(raw)
    try:
        document = pinned(LOADED_NAME,LOADED_SHA)
        latest_release = pinned(LATEST_RELEASE_NAME,LATEST_RELEASE_SHA)
        references = pinned('grasp_references_accepted_20261009.json',prior.accepted.baseline.REFERENCE_SHA)
        profile = pinned('battery_hand_verified_profile.json',references['profile_sha256'])
        profile['sha256'] = references['profile_sha256']
        previous = pinned(prior.accepted.measured.LATEST_NAME,prior.accepted.measured.LATEST_SHA)
        completed = pinned(prior.accepted.LATEST_NAME,prior.accepted.LATEST_SHA)
        latest = pinned(prior.LATEST_NAME,prior.LATEST_SHA)
        safe = prior.accepted.baseline.accepted_reference(references,profile)
        observation = prior.accepted.measured.accepted_latest_reference(previous,safe,profile)
        hover = prior.accepted.measured.accepted_pick_reference(references,profile,observation)
        prior.accepted.accepted_completion(completed,profile,hover)
        prior.accepted_continuous_completion(latest,profile,hover)
        if (document['outcome'] != 'aborted_no_further_writes' or document['configuration'] != profile['observed_configuration']
                or document['arm_command_attempted'] is not False or document['empty_prepare_feedback_pass'] is not True):
            raise ValueError('Unaccepted recovery provenance')
        if (latest_release.get('outcome') != 'aborted_no_further_writes'
                or latest_release.get('recovery_source_sha256') != LOADED_SHA
                or latest_release.get('arm_command_attempted') is not False
                or latest_release.get('moves') != []
                or latest_release.get('force_exit_command_acknowledged') is not None
                or latest_release.get('configuration') != profile['observed_configuration']
                or latest_release['force_pid_on_device'] != document['force_pid_on_device']
                or latest_release['last_recovery_snapshot']['status'] != [5,5,2,2,2,2]
                or latest_release['last_recovery_snapshot']['force_targets'] != list(FORCE_TARGETS)):
            raise ValueError('Latest stall-recovery provenance mismatch')
    except (OSError,ValueError,KeyError,TypeError,RuntimeError) as exc:
        parser.error(str(exc)+'; no connection')
    folder = ROOT/'captures'/('supported_contact_release_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    report = dict(schema_version=1,outcome='starting',moves=[],exceptions=[],cleanup_errors=[],
        recovery_source_sha256=LOADED_SHA,profile_sha256=profile['sha256'],
        latest_release_source_sha256=LATEST_RELEASE_SHA,
        arm_command_attempted=False,physical_stop_verified=False,torque_release_verified=False,
        full_pick_path_ready=False,hand_exchange_count=0,hand_exchange_elapsed_seconds=0.,
        source_sha256={name:hashlib.sha256((directory/name).read_bytes()).hexdigest()
                      for name in list(prior.SOURCE_NAMES)+['run_supported_contact_release.py',LOADED_NAME,LATEST_RELEASE_NAME]},
        timestamp_utc=datetime.now(timezone.utc).isoformat())
    def save():
        temp = folder/'result.tmp'
        temp.write_text(json.dumps(prior._json_safe(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        temp.replace(folder/'result.json')
    save()
    robot = sequence = None
    started = time.monotonic()
    logger = logging.getLogger('battery_hand_readonly_sdk.robotic_arm')
    level = logger.level
    try:
        import fcntl
        with (folder/'wire.jsonl').open('w',encoding='utf-8') as wire, \
             (folder/'samples.jsonl').open('w',encoding='utf-8') as samples, \
             (folder/'arm_protection_wire.jsonl').open('w',encoding='utf-8') as protection_wire, \
             (ROOT/'captures/hand_stage.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            robot = prior.connect(args.sdk_dir)
            logger.setLevel(logging.WARNING)
            def protection_log(row):
                protection_wire.write(json.dumps(row)+'\n'); protection_wire.flush()
            authority = PhaseAuthority()
            def factory(seq,guarded):
                if not guarded or seq.io is not None: raise RuntimeError('Single guarded hand session required')
                def log(row):
                    wire.write(json.dumps(row)+'\n'); wire.flush()
                    if row.get('event') == 'tx': report['hand_exchange_count'] += 1
                    if row.get('event') == 'response': report['hand_exchange_elapsed_seconds'] += row.get('elapsed_s',0.)
                seq.io = ReleaseTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),log,authority)
                return ReleaseHand(seq,seq.io,report,save,samples)
            sequence = ReleaseSequence(robot,observation,profile,report,save,samples,factory,
                target_reference=hover,latest_document=latest,authority=authority,
                protection_client=prior.ProtectionClient(protection_log))
            sequence.force_pid = copy.deepcopy(document['force_pid_on_device'])
            try: sequence.run(True)
            except (Exception,KeyboardInterrupt):
                sequence.fault()
                raise
            finally: sequence.close_hand()
    except (Exception,KeyboardInterrupt) as exc:
        report['outcome'] = 'aborted_no_further_writes'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        print('ABORT:',exc,'现场硬件停止并保持支撑；不自动重试、张开或移动臂。',flush=True)
    finally:
        if robot is not None:
            for method in ('Arm_Socket_Close','RM_API_UnInit'):
                try: getattr(robot,method)()
                except Exception as exc: report['cleanup_errors'].append(dict(method=method,error=repr(exc)))
        logger.setLevel(level)
        if report['cleanup_errors']: report['outcome']='failed_cleanup'
        report['run_elapsed_seconds']=time.monotonic()-started
        save()
        with zipfile.ZipFile(folder.with_suffix('.zip'),'x',zipfile.ZIP_DEFLATED) as archive:
            for item in sorted(folder.iterdir()):
                if item.is_file(): archive.write(item,folder.name+'/'+item.name)
        print('SAVED:',folder.with_suffix('.zip'),'OUTCOME:',report['outcome'],flush=True)
    return 0 if report['outcome'] == 'completed_explicit_supported_open' else 1


if __name__ == '__main__':
    raise SystemExit(main())

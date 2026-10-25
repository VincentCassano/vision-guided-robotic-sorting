#!/usr/bin/env python3
"""Supervised EMPTY-hand school battery pose demonstration; arm stays stationary."""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import time
import traceback
from datetime import datetime

from school_battery_transport import Transport, ALLOWED_WRITES
from run_school_battery import compile_plan

ROOT = Path(__file__).resolve().parents[1]
IDENTITY = [512,769,151,8193,290,2]
TOL = 150
TARGET_TOL = 2  # Observed register quantization; separate from position tolerance.
ADAPTED_PLAN = [
    dict(phase='prepare_batch',address=1135,count=6,values=[8000,8000,8000,8000,65535,65535]),
    dict(phase='adapted_battery_pose',address=1135,count=6,values=[15000,30000,35000,40000,65535,65535]),
    dict(phase='school_final_index',address=1136,count=1,values=[40000]),
]


class ProtectionFault(RuntimeError):
    pass


class Hand:
    def __init__(self, io, report, save, samples, clock=time.monotonic, confirm=input):
        self.io,self.report,self.save,self.samples = io,report,save,samples
        self.clock,self.confirm = clock,confirm

    def event(self, event, **fields):
        row = dict(event=event,monotonic=self.clock(),**fields)
        self.samples.write(json.dumps(row)+'\n'); self.samples.flush()

    def read(self,name,address,count):
        started=self.clock(); values=self.io.read(address,count)
        self.event('read',name=name,address=address,values=values,started_monotonic=started)
        return values

    def snapshot(self):
        status=self.read('status',1085,6)
        if any(v not in (0,1,2) for v in status):
            self.report['protection_latched']=True
            self.io.disable_motion(); self.save()
            raise ProtectionFault('Protection/invalid status: '+str(status))
        current=self.read('current_ma_0_to_4',1105,5)
        rotation_current=self.io.single(1110)
        self.event('read',name='rotation_current_ma',address=1110,values=[rotation_current])
        positions=self.read('positions',1145,6)
        self.report['last_snapshot']=dict(status=status,current=current+[rotation_current],positions=positions)
        if any(c>limit for c,limit in zip(current+[rotation_current],self.report['configuration']['current_limit_ma'])):
            self.report['protection_latched']=True
            self.io.disable_motion(); self.save()
            raise ProtectionFault('Measured current exceeds live controller current limit')
        self.save()
        return self.report['last_snapshot']

    def preflight(self, expected=None, start_open=False):
        identity=self.read('identity',1000,8)
        if identity[:6]!=IDENTITY or identity[6]!=0:
            self.io.disable_motion()
            raise RuntimeError('Unexpected identity or nonzero sub-exception: '+str(identity))
        snap=self.snapshot()
        targets=self.read('targets',1135,6)
        speeds=self.read('speeds',1125,6)
        forces=self.read('force_targets',1115,5)
        values=dict(identity=identity,**snap,targets=targets,speeds=speeds,force_targets=forces)
        self.report['last_preflight']=values; self.save()
        if snap['status']!=[2]*6 or any(forces):
            raise RuntimeError('Require reached state 2 and zero force-control targets')
        if any(abs(a-b)>TOL for a,b in zip(targets,snap['positions'])):
            raise RuntimeError('Targets do not match positions within feedback tolerance')
        if expected is not None and any(abs(a-b)>TARGET_TOL for a,b in zip(targets,expected)):
            raise RuntimeError('Target changed outside this script')
        if expected is not None and speeds!=self.report.get('required_speeds',[65535]*6):
            raise RuntimeError('Recorded speed profile changed; no next pose')
        if start_open and (any(v>TOL for v in targets[:5]+snap['positions'][:5])
                           or not 0<=targets[5]<=2000 or not 0<=snap['positions'][5]<=2150):
            raise RuntimeError('Start requires open flex fingers and thumb at side-palm')
        return values

    def configuration(self):
        blocks=(('identity',1000,8),('calibration_end_absolute',1020,6),
                ('calibration_start_absolute',1030,6),('thumb_presets_absolute',1040,3),
                ('pid_p',1045,6),('pid_i',1055,6),('pid_d',1065,6),('pid_g',1075,6),
                ('current_limit_ma',1095,6),('stall_speed',1185,6),('stall_current_ma',1195,6),
                ('stall_stop_ms',1205,6),('stall_retry_ms',1215,6))
        self.report['configuration']={}
        for name,addr,count in blocks:
            self.report['configuration'][name]=self.read(name,addr,count);self.save()
        c=self.report['configuration']
        if c['identity'][:6]!=IDENTITY or c['identity'][6]!=0:
            raise RuntimeError('Wrong device identity or sub-exception')
        if any(a==b for a,b in zip(c['calibration_start_absolute'],c['calibration_end_absolute'])):
            raise RuntimeError('Degenerate factory calibration range; no motion')
        if c['current_limit_ma']!=[1299]*6 or c['stall_speed']!=[16]*6 or c['stall_current_ma']!=[200]*6 or c['stall_stop_ms']!=[300]*6 or c['stall_retry_ms']!=[500]*6:
            raise RuntimeError('Protection configuration differs from this device baseline')

    def wait(self, expected, active_channels, previous, min_seconds=1.0):
        begin=self.clock(); idle_since=None; stable=0
        speeds=self.report.get('required_speeds',[65535]*6)
        travel=max((abs(expected[c]-previous[c])/speeds[c] for c in active_channels),default=0)
        deadline=begin+travel+30
        while self.clock()<deadline:
            snap=self.snapshot(); pos=snap['positions']; status=snap['status']
            # State and position reads are sequential. Firmware may briefly
            # correct an idle finger after a load changes; status0/1 alone is
            # not proof that another controller issued a new target.
            targets=self.read('targets_during_motion',1135,6)
            if any(abs(a-b)>TARGET_TOL for a,b in zip(targets,expected)):
                self.event('target_changed_during_motion',expected=expected,actual=targets)
                raise RuntimeError('Target changed outside this script: '+str(targets))
            for ch in range(6):
                if ch not in active_channels:
                    if abs(pos[ch]-previous[ch])>TOL:
                        self.event('inactive_position_drift',channel=ch,expected=previous[ch],
                                   actual=pos[ch],status=status[ch],targets=targets)
                        raise RuntimeError('Inactive finger position drift exceeds tolerance: '+str(ch))
                    if status[ch]!=2:
                        self.event('inactive_finger_settling',channel=ch,expected=previous[ch],
                                   actual=pos[ch],status=status[ch],targets=targets)
                if not max(0,min(previous[ch],expected[ch])-TOL)<=pos[ch]<=min(65535,max(previous[ch],expected[ch])+TOL):
                    raise RuntimeError('Position outside commanded travel: '+str(pos))
            reached=status==[2]*6 and all(abs(a-b)<=TOL for a,b in zip(pos,expected))
            if reached:
                stable+=1
                if idle_since is None:idle_since=self.clock()
            else:
                stable=0;idle_since=None
            if reached and stable>=2 and self.clock()-idle_since>=5 and self.clock()-begin>=min_seconds:
                self.event('pose_reached_and_observed',expected=expected,positions=pos,
                           elapsed_seconds=self.clock()-begin,idle_seconds=self.clock()-idle_since)
                return snap
        raise RuntimeError('Pose timeout; no automatic return')

    def ask(self,text):
        began=self.clock();cmd=self.confirm(text).strip().lower()
        self.event('operator_input',command=cmd,wait_seconds=self.clock()-began)
        return cmd

    def write_pose(self,step,previous):
        self.preflight(expected=previous)
        expected=previous[:];ch=step['address']-1135
        expected[ch:ch+step['count']]=step['values']
        row=dict(step=step,expected=expected,outcome='pending')
        self.report['moves'].append(row);self.save()
        self.event('pose_write_before',address=step['address'],values=step['values'])
        self.io.write_group(step['address'],step['values'])
        row['snapshot']=self.wait(expected,tuple(range(ch,ch+step['count'])),previous)
        # Validate actual target quantization as well as position feedback.
        self.preflight(expected=expected)
        row['outcome']='reached_and_observed';self.save()
        return expected

    def run(self,plan):
        self.configuration()
        first=self.preflight(start_open=True)
        self.report['initial']=first;self.save()
        if self.ask('Empty hand, clear swept volume, onsite stop ready. Enter n to configure factory speeds, q to leave: ')!='n':
            self.report['outcome']='operator_exit_no_writes';return
        initial=self.preflight(start_open=True)
        self.report['after_confirmation']=initial;self.save()
        self.io.enable_motion()
        self.io.write_group(1125,[65535]*6)
        self.report['required_speeds']=[65535]*6
        previous=initial['targets'][:]
        self.preflight(expected=previous)
        index=0
        while True:
            text=('n = school stage {}/{} {}; r = open and finish; q = leave pose: '.format(index+1,len(plan),plan[index]['phase'])
                  if index<len(plan) else 'Battery pose reached. r = open and finish; q = leave pose: ')
            cmd=self.ask(text)
            if cmd=='q':
                self.report.update(outcome='operator_exit_pose_held',last_expected=previous);return
            if cmd=='r':
                # This unit's measured side-palm origin is 728 (about 1 degree).
                opening=dict(phase='operator_open',address=1135,count=6,values=[0,0,0,0,0,728])
                previous=self.write_pose(opening,previous)
                self.report.update(outcome='completed_open',final_expected=previous);return
            if cmd!='n' or index>=len(plan):continue
            previous=self.write_pose(plan[index],previous);index+=1
            self.report['completed_school_stages']=index;self.save()
            print('REACHED stage',index,'targets',previous,flush=True)

    def observe_pose(self,phase,expected):
        before=self.preflight(expected=expected)
        angles=self.read('angle_feedback_raw_u16',1165,6)
        forces=self.read('force_feedback_raw',1175,5)
        after=self.preflight(expected=expected)
        row=dict(phase=phase,expected=expected[:],before=before,angle_raw_u16=angles,
                 force_feedback_raw=forces,after=after,reads_are_sequential=True)
        self.report.setdefault('pose_observations',[]).append(row);self.save()
        return row

    def run_adapted(self):
        """New empty-hand candidate: school finger order, thumb flex capped at15000.

        This is an evidence-based posture choice, not a certified joint limit.
        It cannot authorize contact with a battery or waive protection.
        """
        self.configuration()
        first=self.preflight(start_open=True)
        self.report.update(initial=first,adaptation=dict(
            source='school_battery_pose.json',thumb_flex_candidate=15000,
            thumb_limit_certified=False,combined_pose_verified=False,
            school_index40000_verified=False),real_battery_grasp_verified=False,
            torque_release_verified=False)
        self.save()
        if self.ask('EMPTY HAND: n = configure factory speeds; q = leave: ')!='n':
            self.report['outcome']='operator_exit_no_writes';return
        initial=self.preflight(start_open=True)
        self.report['after_confirmation']=initial;self.save()
        self.io.enable_motion()
        self.io.write_group(1125,[65535]*6)
        self.report['required_speeds']=[65535]*6
        previous=initial['targets'][:]
        self.observe_pose('initial_open',previous)
        index=0
        while True:
            text=('n = adapted stage {}/{} {}; r = phased open; q = leave pose: '.format(
                  index+1,len(ADAPTED_PLAN),ADAPTED_PLAN[index]['phase'])
                  if index<len(ADAPTED_PLAN) else 'ADAPTED EMPTY-HAND pose reached. r = phased open; q = leave pose: ')
            cmd=self.ask(text)
            if cmd=='q':
                self.report.update(outcome='operator_exit_pose_held',last_expected=previous);return
            if cmd=='r':
                previous=self.release_stages(previous)
                self.report.update(outcome='completed_adapted_open',final=self.preflight(expected=previous),
                                   final_expected=previous,restored_open_position=True)
                self.save();return
            if cmd!='n' or index>=len(ADAPTED_PLAN):continue
            step=ADAPTED_PLAN[index]
            previous=self.write_pose(step,previous)
            self.observe_pose(step['phase'],previous)
            index+=1
            self.report['completed_adapted_stages']=index;self.save()
            print('REACHED adapted stage',index,'targets',previous,flush=True)

    def release_stages(self,previous):
        stages=(dict(phase='release_thumb_flex',address=1135,count=1,values=[0]),
                dict(phase='release_four_fingers',address=1136,count=4,values=[0,0,0,0]),
                dict(phase='return_thumb_side_palm',address=1140,count=1,values=[728]))
        for step in stages:
            ch=step['address']-1135
            desired=previous[:];desired[ch:ch+step['count']]=step['values']
            if desired!=previous:
                previous=self.write_pose(step,previous)
            else:
                self.event('already_at_release_target',phase=step['phase'],expected=desired)
            print('OPEN stage:',step['phase'],'targets:',previous,flush=True)
        return previous

    def open_only(self):
        """Independent supervised release path; never resumes the grasp sequence."""
        self.configuration()
        initial=self.preflight()
        self.report['initial']=initial;self.save()
        if any(v<300 for v in initial['speeds']):
            raise RuntimeError('Opening speed below previously used profiles; no write')
        self.report['required_speeds']=initial['speeds'][:]
        previous=initial['targets'][:]
        if self.ask('OPEN ONLY: empty hand, clear opening path, onsite stop ready. r = open; q = leave: ')!='r':
            self.report['outcome']='operator_exit_no_writes';return
        # Recheck after human confirmation. No speed/force/protection writes.
        self.preflight(expected=previous)
        self.report['release_authorized']=True;self.save()
        self.io.enable_motion()
        previous=self.release_stages(previous)
        final=self.preflight(expected=previous)
        self.report.update(outcome='completed_open_only',final=final,final_expected=previous,
                           restored_open_position=True,torque_release_verified=False)
        self.save()

    def fault_details(self):
        self.io.disable_motion()
        if self.io.broken:return
        # Read-only feedback. Later state 2 never clears the write latch.
        try:
            for _ in range(3):
                row={}
                for name,addr,n in (('status',1085,6),('current',1105,5),('positions',1145,6),('targets',1135,6)):
                    row[name]=self.read('post_fault_'+name,addr,n)
                row['rotation_current']=self.io.single(1110)
                self.report.setdefault('post_fault',[]).append(row);self.save()
        except (Exception,KeyboardInterrupt) as exc:
            self.report['post_fault_read_error']=repr(exc)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-empty-hand',action='store_true')
    parser.add_argument('--adapted-empty-hand',action='store_true')
    parser.add_argument('--open-only',action='store_true')
    parser.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--confirm-factory-speed',action='store_true')
    parser.add_argument('--plan-only',action='store_true')
    parser.add_argument('--plan-adapted-only',action='store_true')
    args=parser.parse_args()
    if sum((args.plan_only,args.plan_adapted_only,args.execute_empty_hand,args.adapted_empty_hand,args.open_only))>1:
        parser.error('Plan, adapted plan, demonstrations and open-only are mutually exclusive')
    if (args.execute_empty_hand or args.adapted_empty_hand) and not(args.confirm_empty_clear_exclusive and args.confirm_onsite_stop and args.confirm_factory_speed):
        parser.error('Execution requires empty/clear/exclusive, onsite stop, and factory-speed confirmations')
    if args.open_only and not(args.confirm_empty_clear_exclusive and args.confirm_onsite_stop):
        parser.error('Open-only requires empty/clear/exclusive and onsite hardware stop confirmations')
    recipe_path=Path(__file__).with_name('school_battery_pose.json')
    plan=compile_plan(json.loads(recipe_path.read_text(encoding='utf-8')))
    if args.adapted_empty_hand or args.plan_adapted_only:
        plan=ADAPTED_PLAN
    if any((s['address'],tuple(s['values'])) not in ALLOWED_WRITES for s in plan):
        parser.error('Recipe differs from the audited school poses')
    if args.plan_only or args.plan_adapted_only:
        print(json.dumps(plan,indent=2));return 0
    import fcntl
    folder=ROOT/'captures'/('school_battery_hand_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir(parents=True)
    names=('run_school_battery_hand.py','school_battery_transport.py','run_school_battery.py','school_battery_pose.json')
    report=dict(mode='open_only' if args.open_only else ('adapted_empty_hand' if args.adapted_empty_hand else ('execute_empty_hand' if args.execute_empty_hand else 'read_only')),outcome='starting',
                moves=[],plan=plan,school_runtime_speed='unknown',
                selected_speed_profile='preserve_current_for_opening' if args.open_only else 'manufacturer_factory_65535',
                real_battery_grasp_verified=False,source_sha256={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in names})
    def save():
        tmp=folder/'result.tmp';tmp.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');tmp.replace(folder/'result.json')
    io=None;hand=None
    try:
        with (ROOT/'captures'/'hand_stage.lock').open('a') as lock,(folder/'wire.jsonl').open('w',encoding='utf-8') as wire,(folder/'samples.jsonl').open('w',encoding='utf-8') as samples:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            def log(row):wire.write(json.dumps(row)+'\n');wire.flush()
            io=Transport(socket.create_connection(('169.254.128.18',8080),timeout=3),log)
            hand=Hand(io,report,save,samples)
            print('EMPTY HAND ONLY. Hardware stop for contact. Opening preserves current speeds; grasp selects factory speed. Fault/quit does not release torque.',flush=True)
            try:
                if args.open_only:hand.open_only()
                elif args.adapted_empty_hand:hand.run_adapted()
                elif args.execute_empty_hand:hand.run(plan)
                else:
                    hand.configuration();report['initial']=hand.preflight();report['outcome']='read_only_complete'
            except (Exception,KeyboardInterrupt):
                # Keep the raw log streams open during post-fault reads.
                hand.fault_details()
                raise
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        report.update(outcome='aborted_no_further_writes',error=repr(exc),traceback=traceback.format_exc())
        print('ABORT:',exc,'Onsite hardware stop if contact or continuing motion.',flush=True)
        return 1
    finally:
        if io is not None:
            io.disable_motion()
            try:io.sock.close()
            except Exception as exc:report['close_error']=repr(exc)
        save();print('SAVED:',folder,'OUTCOME:',report['outcome'],flush=True)


if __name__=='__main__':raise SystemExit(main())

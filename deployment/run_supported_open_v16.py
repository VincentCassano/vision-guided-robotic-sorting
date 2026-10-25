"""One explicitly supported five-flex opening; no arm SDK, reset or retry."""
import argparse
import fcntl
import json
from pathlib import Path
import time
import traceback

from known_cylinder_records import dump
from visual_pick_sort_v15_hand import GroupHandBus
from visual_pick_sort_v16_source import ROOT,verify_delivery,accepted_success


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--confirm-supported-stationary-clear-exclusive',action='store_true',required=True)
    args=parser.parse_args();output=args.output.resolve()
    output.parent.mkdir(parents=True,exist_ok=True)
    report=dict(scope='supported_open_five_only',phase='preflight',exceptions=[],
        arm_SDK_loaded=False,arm_queries=0,arm_actuator_writes=0,hand_register_writes=0,
        configuration_writes=0,reset_writes=0,automatic_retry=False,
        support_and_stationary_arm_and_clear_opening_confirmed=args.confirm_supported_stationary_clear_exclusive,
        physical_release_verified=False,protection_cleared_claimed=False)
    bus=lock=None
    def record(row):
        with output.with_name('hand_wire.jsonl').open('a',encoding='utf8',newline='\n') as stream:
            stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    try:
        report['checked_files']=len(verify_delivery());success=accepted_success()
        lock=(ROOT/'captures/hand_stage.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        bus=GroupHandBus(record,success['prepare_targets'],success['grip_targets'])
        current=bus.connect_read();report['fresh_supported_hand']=current;dump(output,report)
        if current.get('ok') is not True or current.get('snapshot_stale') or current.get('blockers'):
            raise RuntimeError('Fresh hand connection/configuration/current unavailable; no opening')
        state=current['snapshot'];desired=[0,0,0,0,0,state['targets'][5]]
        if any(0>min(state['targets'][i],state['positions'][i]) for i in range(5)):
            raise RuntimeError('Opening must not add flexion to a protected channel')
        report['requested_targets']=desired;report['protected_channels_before']=state['protected_channels']
        if any(state['targets'][i]!=0 for i in range(5)):
            io=bus._io
            # Exact withdrawal capability for this supported release, added
            # only to this connection. The opposition channel is untouched.
            io.groups['supported_open_five']=(1135,(0,0,0,0,0))
            bus._serial+=1;io.grant_group('supported_open_five',bus._serial)
            report.update(phase='supported_open_write',hand_register_writes=1);dump(output,report)
            io.write_group(1135,[0,0,0,0,0]);report['opening_ACK']=True
            readback=io.read(1135,6);report['target_readback']=readback
            if any(abs(a-b)>2 for a,b in zip(readback,desired)):
                raise RuntimeError('Supported opening target ACK/readback differs; no retry')
        else:report['opening_skipped_targets_already_zero']=True
        after=bus.refresh();report['after_open_read']=after
        if after.get('ok') is not True or after.get('snapshot_stale') or after.get('blockers'):
            raise RuntimeError('Opening was attempted but fresh read is unavailable; do not infer actual release')
        report.update(phase='completed',outcome='supported_open_targets_recorded_check_actual_release')
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report.update(phase='fault',outcome='supported_open_not_completed_no_automatic_retry')
    finally:
        if bus is not None:report['hand_close']=bus.close()
        if lock is not None:lock.close()
        report['finished_monotonic']=time.monotonic();dump(output,report)
    print('SUPPORTED OPEN:',report['outcome'],flush=True)
    print('No arm motion/reset. Confirm actual release and empty hand before e.',flush=True)
    return 0 if report['phase']=='completed' else 1


if __name__=='__main__':raise SystemExit(main())

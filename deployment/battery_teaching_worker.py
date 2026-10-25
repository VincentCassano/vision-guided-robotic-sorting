#!/usr/bin/env python3
"""One noninteractive READ-ONLY SDK lifecycle; cannot access parent stdin."""
import argparse
import copy
import json
from pathlib import Path
import traceback

import capture_battery_teaching as entry


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True)
    args=parser.parse_args(argv)
    folder=args.request.resolve().parent
    request=json.loads(args.request.read_text(encoding='utf-8'))
    entry.verified_installation()
    provenance=entry.load_provenance()
    original=(Path(__file__).parent/'localized_pregrasp_193957.json').read_bytes()
    resume=Path(request['resume_report']).read_bytes()
    source,target,upper,accepted_points=entry.reviewed_teaching_resume(resume,original,provenance[2],provenance[4])
    report=copy.deepcopy(request['report'])
    if (request.get('schema_version')!=1 or report.get('target')!=target or report.get('upper_reference')!=upper
            or report.get('points',[])[:len(accepted_points)]!=accepted_points
            or report.get('execution_ready') is not False or report.get('hardware_arm_writes')!=0
            or report.get('hand_writes_allowed') is not False or report.get('sdk_in_parent') is not False):
        parser.error('Invalid parent scope/provenance; no connection')
    for point in report['points']:entry.check_taught_point(point)
    report['worker_capture_pass']=False
    def save():
        tmp=folder/'worker_result.tmp'
        tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        tmp.replace(folder/'worker_result.json')
    save()
    sdk_args=argparse.Namespace(sdk_dir=Path(request['sdk_dir']))
    try:
        with (folder/'wire.jsonl').open('w',encoding='utf-8') as wire,\
                (folder/'samples.jsonl').open('w',encoding='utf-8') as samples,\
                (folder/'arm_protection_wire.jsonl').open('w',encoding='utf-8') as arm_wire:
            entry.capture_one(sdk_args,request['role'],request['clearance_mm'],provenance,source,
                report,save,samples,wire,arm_wire)
        if report['cleanup_errors']:raise RuntimeError('Worker cleanup failed')
        report['worker_capture_pass']=True
    except (Exception,KeyboardInterrupt) as exc:
        report['worker_capture_pass']=False
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc(),scope='readonly_sdk_worker'))
    finally:save()
    return 0 if report['worker_capture_pass'] else 1


if __name__=='__main__':raise SystemExit(main())

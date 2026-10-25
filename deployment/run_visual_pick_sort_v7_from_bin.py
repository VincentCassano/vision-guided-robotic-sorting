"""Stdlib coordinator: empty return, fresh visual pick/sort, one result bundle."""
import argparse
from datetime import datetime
import hashlib
import io
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--completed-v6',type=Path,required=True)
    for key in ('hand-empty-exclusive','onsite-stop','fixed-setup','inert-shell'):
        parser.add_argument('--confirm-'+key,action='store_true',required=True)
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual robot host only')
    from visual_pick_sort_v7_source import verify_delivery,qualified_v6_bin
    verify_delivery();qualified_v6_bin(args.completed_v6)
    captures=ROOT/'captures';before=set(captures.glob('*.zip'))
    report=dict(scope='empty_return_then_fresh_visual_pick_sort_v7',exceptions=[],steps=[],
        SDK_in_coordinator=False,actual_pickup_or_sort_not_assumed=True)
    name=datetime.now().strftime('visual_pick_sort_v7_bundle_%Y%m%d_%H%M%S_%f')
    def run(extra):
        cmd=[sys.executable,str(ROOT/'deployment/run_visual_pick_sort_v7.py'),
            '--source-pregrasp',str(captures/'visual_pregrasp_stage_v8_20261021_174304_223863.zip'),
            '--color','black','--execute','--confirm-inert-shell-exclusive',
            '--confirm-onsite-stop','--confirm-fixed-setup']+extra
        previous=set(captures.glob('visual_pick_sort_v7_*.zip'))
        rc=subprocess.run(cmd,cwd=str(ROOT),check=False).returncode
        made=set(captures.glob('visual_pick_sort_v7_*.zip'))-previous
        if len(made)!=1:raise RuntimeError('Expected exactly one current run archive')
        path=made.pop()
        with zipfile.ZipFile(path) as z:
            prefix=z.namelist()[0].split('/')[0]+'/'
            result=json.loads(z.read(prefix+'result.json'))
        report['steps'].append(dict(command=cmd,returncode=rc,archive=path.name,
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),outcome=result['outcome']))
        return rc,result
    try:
        print('第一步先核对本次投放位与闭合空手；u前确认手已空、五指张开范围和整条空返通道清空。u后先张开五指再空返。',flush=True)
        rc,returned=run(['--empty-return-from-v6',str(args.completed_v6.resolve())])
        if (rc or returned['outcome']!='completed_empty_return_from_qualified_v6_bin'
                or not returned.get('worker',{}).get('empty_return_completed')
                or returned.get('worker',{}).get('cleanup_errors')):
            raise RuntimeError('Empty return not qualified; no fresh visual approach or grip')
        print('已空返。下面采当前空壳：S/Q保存，m高位对准，d下降，p确认无接触；保持空壳不动，随后g。',flush=True)
        rc,result=run(['--capture-current-target'])
        report['outcome']=result['outcome'];report['last_returncode']=rc
        report['full_cycle_accepted']=bool(result.get('new_cycle_real_device_accepted'))
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report['outcome']='coordinator_stopped_no_retry'
        print('STOP:',repr(exc),flush=True)
    finally:
        created=set(captures.glob('*.zip'))-before
        runs=sorted(p for p in created if p.name.startswith('visual_pick_sort_v7_'))
        represented=set()
        for path in runs:
            try:
                with zipfile.ZipFile(path) as z:
                    for item in z.namelist():
                        if item.endswith('/input_pregrasp.zip'):
                            raw=z.read(item)
                            with zipfile.ZipFile(io.BytesIO(raw)) as inner:
                                if inner.testzip():raise ValueError('Nested source CRC failed')
                                represented.add(hashlib.sha256(raw).hexdigest())
                                for nested in inner.namelist():
                                    if nested.endswith('/input_visual_target.zip'):
                                        represented.add(hashlib.sha256(inner.read(nested)).hexdigest())
            except (OSError,ValueError,zipfile.BadZipFile) as exc:
                report.setdefault('packaging_warnings',[]).append(dict(archive=path.name,error=repr(exc)))
        extras=[p for p in created if p not in runs and p.name.startswith((
            'visual_pregrasp_stage_v7_cycle_','visual_task_binding_'))
            and hashlib.sha256(p.read_bytes()).hexdigest() not in represented]
        members=runs+sorted(extras)
        report['archive_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in members}
        report['nested_source_archive_hashes']=sorted(represented)
        target=captures/(name+'.zip')
        with zipfile.ZipFile(target,'x',zipfile.ZIP_DEFLATED) as z:
            z.writestr('bundle.json',json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            for path in members:z.write(path,path.name)
        print('统一回传这一个ZIP：',target,flush=True)
        print('SHA256:',hashlib.sha256(target.read_bytes()).hexdigest(),flush=True)
    return 0 if report.get('full_cycle_accepted') and not report['exceptions'] else 1


if __name__=='__main__':raise SystemExit(main())

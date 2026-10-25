#!/usr/bin/env python3
"""One guided physical measurement ticket; isolated read-only robot snapshots."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.pregrasp_measurements import FIELDS,ZERO_ALLOWED,parse_length,validate_worksheet,summarize_measurements


def verify_installation():
    rows={}
    for line in (ROOT/'deployment/pregrasp-measurement-files.sha256').read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if name in rows or ROOT.resolve() not in path.parents:raise ValueError('Invalid measurement install path')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise ValueError('Installed file differs: '+name)
        rows[name]=digest
    if not rows:raise ValueError('Empty measurement manifest')
    return rows


def run_worker(args,folder,runner=subprocess.run):
    folder.mkdir(parents=True,exist_ok=False)
    command=[sys.executable,str(ROOT/'deployment/pregrasp_measurement_worker.py'),'--output',str(folder),
             '--sdk-dir',str(args.sdk_dir),'--role','current_static']
    status=dict(command=command,stdin='DEVNULL',close_fds=True,timeout_seconds=120,sdk_in_parent=False,
                timeout_is_hardware_stop=False)
    with (folder/'stdout.log').open('wb') as out,(folder/'stderr.log').open('wb') as err:
        try:
            p=runner(command,stdin=subprocess.DEVNULL,stdout=out,stderr=err,close_fds=True,timeout=120,check=False)
            status['return_code']=p.returncode
        except subprocess.TimeoutExpired as exc:status.update(timed_out=True,error=repr(exc))
        except OSError as exc:status.update(start_failed=True,error=repr(exc))
    (folder/'process_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf-8')
    path=folder/'result.json'
    if not path.is_file():return dict(static_readout_valid=False,geometry_reference_eligible=False,execution_ready=False,
        worker_error='No worker result; see process/stdout/stderr',process=status)
    result=json.loads(path.read_text(encoding='utf-8'))
    if (result.get('execution_ready') is not False or result.get('hardware_writes')!=0
        or result.get('arm_actuator_writes')!=0 or result.get('hand_register_writes')!=0):
        raise ValueError('Worker result violates no-write scope')
    if status.get('return_code')!=0:
        result['static_readout_valid']=False;result['geometry_reference_eligible']=False
    result['process']=status
    return result


def collect_values(ticket,save,reader=input,hand_measurements_allowed=True):
    def ask(prompt):
        answer=reader(prompt).strip()
        ticket.setdefault('input_events',[]).append(dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),prompt=prompt,answer=answer))
        save()
        if answer.lower()=='q':raise KeyboardInterrupt()
        return answer
    def length(prompt,key=None):
        while True:
            value=ask(prompt)
            try:return parse_length(value,allow_zero=key in ZERO_ALLOWED)
            except ValueError as exc:print(str(exc),flush=True)
    def yes(prompt):
        while True:
            value=ask(prompt).lower()
            if value in ('y','yes','是'):return True
            if value in ('n','no','否'):return False
            if value in ('u','未知',''):return None
            print('输入y/n/u；q保存退出。',flush=True)
    w=ticket['worksheet'];o=w['observations']
    while True:
        value=ask('当前手：e=空手，o=有物件，u=不确定（不自动张开）：').lower()
        if value in ('e','o','u',''):
            o['holding_state']={'e':'empty','o':'object','u':'unknown','':'unknown'}[value];break
    hand_measurements_allowed=hand_measurements_allowed and o['holding_state']=='empty'
    if hand_measurements_allowed:
        o['flange_rim_point_identified']=yes('能明确辨认腕末端圆形安装面、并选定其外缘同一测点P吗？y/n/u（遮住就u，不拆卸）：')
        o['reference_description']=ask('描述P在哪个可见安装面/外缘位置；不清楚填u：')
        if o['reference_description'].lower() in ('u','未知',''):o['reference_description']=None
    else:
        o['flange_rim_point_identified']=None;o['reference_description']=None
        print('静止读数无效或未报告空手：本次跳过法兰/手部/线缆近身量尺，仅采可安全获得的板和空壳尺寸。',flush=True)
    print('所有长度单位mm，可输入小数。u=不知道/无法安全量；q=保存现有数据退出。不要输入估计值凑齐。',flush=True)
    print('法兰/P相关项只能用刚才同一实体安装面；手型仍是当前读到的形状，不要求回旧点。',flush=True)
    for key,label,help_text in FIELDS:
        if not hand_measurements_allowed and key not in tuple(row[0] for row in FIELDS[:8])+('measurement_allowance_mm',):
            w['lengths'][key]=None;save();continue
        print('\n'+label+'：'+help_text,flush=True)
        w['lengths'][key]=length(label+' mm（u跳过）：',key);save()
        if key.startswith('current_hand_to_') and w['lengths'][key]==0:
            print('已记录接触。立即现场硬件停止，保留现场状态；程序保存退出，不自动张开或回臂。',flush=True)
            raise KeyboardInterrupt()
    print('\n线缆：每条外露活动线单独记录。A为随末端总成移动的固定接头，B为下一处固定卡扣。',flush=True)
    print('P到A走直线；A到B沿线缆实际走向量自由段全长，不强拉、不只量两端直线距离。无法安全量就u。',flush=True)
    while hand_measurements_allowed and yes('还有一段外露活动线缆要记录吗？y/n/u：') is True:
        row=dict(id='cable_'+str(len(w['cables'])+1))
        row['anchor_A_follows_end_assembly']=yes('A是否固定在手/腕部相机总成上、随其一起动？y/n/u：')
        row['rim_P_to_anchor_A_mm']=length('同一P到接头A直线距离mm（u跳过）：')
        row['free_length_A_to_B_mm']=length('A到B沿线缆自由段的全长mm（u跳过）：')
        row['note']=ask('A/B在哪、这段线连什么？不清楚u：')
        w['cables'].append(row);save()
    o['all_visible_moving_cables_listed']=yes('可见的所有外露活动线段是否都记录了？y/n/u：') if hand_measurements_allowed else None
    o['arm_hand_unchanged_during_measurement']=yes('整个测量期间，臂/手均未改变姿态或目标，尺具和双手已撤开吗？y/n/u：')
    save()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--collect',action='store_true',help='Fresh stationary snapshots plus guided physical measurements; no motion')
    mode.add_argument('--worksheet-only',action='store_true',help='No SDK/device; write a blank worksheet')
    mode.add_argument('--from-json',type=Path,help='Offline validate a populated worksheet; no SDK/device')
    parser.add_argument('--confirm-stationary-exclusive',action='store_true')
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    args=parser.parse_args(argv)
    if args.collect and not args.confirm_stationary_exclusive:
        parser.error('--collect requires --confirm-stationary-exclusive; stop other control/teaching scripts and keep arm/hand stationary')
    if args.collect and (platform.system()!='Linux' or platform.machine()!='aarch64'):
        parser.error('Device reads permitted only on reviewed onsite Linux aarch64; no connection')
    try:installed=verify_installation()
    except (ValueError,OSError) as exc:parser.error(str(exc)+'; no connection')
    folder=args.output_parent/('pregrasp_measurements_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    ticket=dict(schema_version=1,timestamp_utc=datetime.now(timezone.utc).isoformat(),execution_ready=False,hardware_writes=0,
        sdk_in_parent=False,arm_motion_allowed=False,hand_motion_allowed=False,
        current_state_not_historical=True,source='guided_operator_lengths_and_isolated_readonly_brackets',
        installed_file_sha256=installed,exceptions=[],input_events=[],worksheet=dict(schema_version=1,lengths={},observations={},cables=[]))
    def save():
        tmp=folder/'result.tmp';tmp.write_text(json.dumps(ticket,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        tmp.replace(folder/'result.json')
        (folder/'measurements.json').write_text(json.dumps(ticket['worksheet'],ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    save()
    try:
        if args.worksheet_only:ticket['outcome']='blank_measurement_worksheet_no_device'
        elif args.from_json:
            ticket['worksheet']=validate_worksheet(json.loads(args.from_json.read_text(encoding='utf-8')))
            ticket['analysis']=summarize_measurements(ticket['worksheet']);ticket['outcome']='offline_worksheet_review_no_device'
        else:
            print('本次只记录当前姿态和集中量尺，不定位/下降/闭指/归位。接触或意外继续运动立即现场硬件停止。',flush=True)
            print('无需回旧测量位。测不到就u，q保存退出。采完一个ZIP统一回传，不逐项传结果。',flush=True)
            print('先读当前设备（只读子进程；通常几十秒；不在等待输入期间保持SDK连接）…',flush=True)
            ticket['before']=run_worker(args,folder/'before');save()
            print('当前手型：',ticket['before'].get('analysis',{}).get('hand_pose_classification','未知'),
                '静止读数有效：',ticket['before'].get('static_readout_valid'),flush=True)
            if not ticket['before'].get('static_readout_valid'):
                print('当前设备读取未通过；完整报错已保存。仅量可安全取得的实体尺寸，不据此使用当前姿态。',flush=True)
            collect_values(ticket,save,hand_measurements_allowed=ticket['before'].get('static_readout_valid') is True)
            print('量尺结束，双手/尺具撤开后复读同一静止姿态，不发运动命令…',flush=True)
            ticket['after']=run_worker(args,folder/'after');save()
            ticket['analysis']=summarize_measurements(ticket['worksheet'],ticket.get('before'),ticket.get('after'))
            ticket['outcome']='physical_measurements_collected_descent_pending'
    except KeyboardInterrupt:
        ticket['outcome']='partial_measurements_saved_no_actuator_writes'
        ticket['analysis']=summarize_measurements(ticket['worksheet'],ticket.get('before'),ticket.get('after'))
    except Exception as exc:
        ticket['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        ticket['outcome']='measurement_collection_failed_no_actuator_writes'
    finally:
        ticket['execution_ready']=False;save()
        with zipfile.ZipFile(folder.with_suffix('.zip'),'x',zipfile.ZIP_DEFLATED) as z:
            for path in sorted(folder.rglob('*')):
                if path.is_file():z.write(path,str(Path(folder.name)/path.relative_to(folder)))
        print('SAVED:',folder.with_suffix('.zip'),'OUTCOME:',ticket['outcome'],'EXECUTION_READY: False',flush=True)
        print('本程序没有自动下降入口；量尺/回读成功不认证物理路径或停止。',flush=True)
    return 1 if ticket['exceptions'] else 0


if __name__=='__main__':raise SystemExit(main())

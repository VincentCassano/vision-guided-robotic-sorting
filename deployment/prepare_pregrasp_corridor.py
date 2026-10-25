#!/usr/bin/env python3
"""Consume accepted onsite path results; offline geometry preparation, no SDK."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.vertical_corridor import analyse_vertical_corridor


def verify_installation():
    rows={};path=ROOT/'deployment/pregrasp-corridor-files.sha256'
    for line in path.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);target=(ROOT/name).resolve()
        if name in rows or ROOT.resolve() not in target.parents:raise ValueError('Invalid corridor installation path')
        if hashlib.sha256(target.read_bytes()).hexdigest()!=digest:raise ValueError('Corridor installed file differs: '+name)
        rows[name]=digest
    if not rows:raise ValueError('Empty corridor manifest')
    return rows


def export_corridor(folder,result):
    folder.mkdir(parents=True,exist_ok=True)
    (folder/'corridor.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    with (folder/'vertical_reference_gap.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['index','X_mm','Y_mm','Z_mm','ideal_reference_gap_mm',
            'relative_FK_reference_gap_mm','FK_rotation_from_anchor_deg','budgeted_plane_lower_bound_mm','executable'])
        for row in result['rows']:
            writer.writerow([row['index'],*row['desired_world_mm'],row['ideal_reference_plane_gap_mm'],
                row['relative_FK_z_reference_gap_mm'],row['rotation_from_anchor_degrees'],row['budgeted_plane_lower_bound_mm'],False])
    with (folder/'arm_axis_motion.csv').open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['path_index','joint_axis','X_mm','Y_mm','Z_mm','body_collision_envelope_known'])
        for row in result['axis_samples']:writer.writerow([row['path_index'],row['joint_axis'],*row['world_mm'],False])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-profile',type=Path,default=ROOT/'deployment/pregrasp_corridor_scene.json')
    parser.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    args=parser.parse_args(argv)
    try:
        installed=verify_installation()
        raw=(ROOT/'deployment/taught_pregrasp_200357.json').read_bytes()
        profile=json.loads(args.scene_profile.read_text(encoding='utf-8'))
        result=analyse_vertical_corridor(raw,profile)
    except (ValueError,OSError,KeyError,TypeError) as exc:parser.error(str(exc)+'; no SDK or device connection')
    result['installed_file_sha256']=installed
    folder=args.output_parent/('pregrasp_corridor_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    export_corridor(folder,result)
    with zipfile.ZipFile(folder.with_suffix('.zip'),'x',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(folder.iterdir()):z.write(path,str(Path(folder.name)/path.name))
    print('200357原主机数值阶段已验收；不重算模型、不重采示教或手型。')
    print('固定PREPARE/固定腕姿/垂直平移条件下，参考平面理想间隙：',
        result['maximum_ideal_reference_plane_gap_mm'],'->',result['minimum_ideal_reference_plane_gap_mm'],'mm')
    print('这只覆盖同一参考平面的条件计算；不包含电池、板边、臂身或线缆避碰。')
    print('SAVED:',folder.with_suffix('.zip'),'EXECUTION_READY: False')
    return 0


if __name__=='__main__':raise SystemExit(main())

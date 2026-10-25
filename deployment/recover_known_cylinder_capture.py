"""Recover already saved A/B records after v3 JSON failure; files only.

No camera, inference, SDK, device queries, model fitting, or new acquisition.
Original files are snapshotted without modification. A missing original B image
or completion timestamp is reported explicitly, never recreated or assumed.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

from known_cylinder_records import archive_folder, dump, encode

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--recover-latest', action='store_true')
    source.add_argument('--source-session', type=Path)
    parser.add_argument('--reference-spacing-mm', type=float, required=True)
    args = parser.parse_args()
    if not 0 < args.reference_spacing_mm < 1000:
        parser.error('Reference spacing must be finite and between 0 and 1000 mm')
    if args.recover_latest:
        choices = sorted(p for p in (ROOT/'captures').glob('known_cylinder_20*') if p.is_dir())
        if not choices:
            parser.error('No original known_cylinder session directory; original files are required')
        original = choices[-1]
    else:
        original = args.source_session.resolve()
    if not original.is_dir():
        parser.error('Source session is not a directory')
    if (original/'result.json').is_file():
        try:
            read(original/'result.json')
        except (ValueError, OSError):
            pass
        else:
            parser.error('Latest session already has readable result.json; do not silently recover an older session. Use the exact crashed directory with --source-session')
    folder = ROOT/'captures'/('known_cylinder_recovered_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(scope='recovery_arithmetic_on_original_saved_records_only',
        source_session=str(original),reference_spacing_mm=args.reference_spacing_mm,
        camera_opened=False,inference_started=False,robot_SDK_loaded=False,
        arm_connections=0,arm_writes=0,hand_writes=0,new_images_acquired=0,
        model_refitted=False,original_files_modified=False,execution_ready=False,
        original_runtime_metadata_available=False,original_B_raw_image_available=False,
        B_completion_wall_time_known=False)
    try:
        snapshot=[]
        for path in sorted(original.iterdir()):
            if not path.is_file():
                continue
            data=path.read_bytes()
            # Flat snapshots keep all original observations, including failed A.
            target=folder/('original_'+path.name)
            target.write_bytes(data)
            snapshot.append(dict(name=path.name,sha256=hashlib.sha256(data).hexdigest(),bytes=len(data)))
        dump(folder/'original_inventory.json',snapshot)
        print('SOURCE:',original,flush=True)
        observations=[]
        for path in sorted(original.glob('observation_*.json')):
            row=read(folder/('original_'+path.name))
            if row.get('burst_summary',{}).get('burst_qualified') is True:
                observations.append((path.name,row))
        if len(observations)!=1:
            raise ValueError('Expected exactly one saved qualified A; found %d. All original files retained.'%len(observations))
        name,a=observations[0]
        if a.get('delivery_version')!='known-cylinder-vision-v3-20261020':
            raise ValueError('Recovery is scoped to this v3 failure; unexpected A version')
        interrupted=read(folder/'original_interrupted_burst.json')
        frames=interrupted['frames']
        if not frames or any(f.get('delivery_version')!=a['delivery_version'] for f in frames):
            raise ValueError('Missing B records or mixed delivery versions')
        if any(f['timestamp_ms']<=a['timestamp_ms'] for f in frames):
            raise ValueError('B frames are not newer than the saved A observation')
        import known_cylinder_capture as burst
        b=burst.summarize(frames)
        # Do not reissue a timed pass: v3 lost its completion wall time. Only
        # the persisted frame sequence can be checked and summarized here.
        report['A_source_observation']=name
        report['A_summary']=a['burst_summary']
        report['B_recovered_summary']=b
        dump(folder/'recovered_B.json',dict(summary=b,frames=frames,
            raw_B_image_available=False,original_timed_burst_pass_reissued=False,execution_ready=False))
        if not b['frame_quality_pass']:
            raise ValueError('Saved B frame quality rejected: '+repr(b['reject_reasons']))
        pair=burst.relative_pair(a['burst_summary'],b,args.reference_spacing_mm)
        report.update(relative_pair_result=pair,
            outcome='recorded_pair_recovered_absolute_accuracy_pending',
            limitations=['Original B raw image was not saved before the v3 failure',
                'Original B completion monotonic time and final runtime metadata were not saved',
                'Frame statistics and relative distance do not establish absolute accuracy or expand FIT support'])
        dump(folder/'relative_pair_recovered.json',pair)
        print('RECOVERED A:',name,'B FRAMES:',len(frames),flush=True)
        print('RELATIVE PAIR:',encode(pair),flush=True)
        return 0
    except Exception as exc:
        report.update(error=repr(exc),outcome='recovery_incomplete_original_files_retained')
        print('RECOVERY INCOMPLETE:',exc,flush=True)
        return 1
    finally:
        dump(folder/'result.json',report)
        archive=archive_folder(folder)
        print('SAVED:',archive,'NO NEW ACQUISITION / NO ROBOT COMMANDS',flush=True)


if __name__=='__main__':
    raise SystemExit(main())

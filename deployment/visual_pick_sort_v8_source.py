"""File provenance for one supported ring withdrawal, then a NEW healthy cycle."""
import hashlib
import io
import json
from pathlib import Path
import zipfile
from visual_pick_sort_v7_source import (
    qualified_v6_bin,back40_offset,grip_recipe_proof as previous_proof,
    verify_delivery as frozen_v7_delivery,accepted_pregrasp as frozen_v7_pregrasp)

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v8-files.sha256'
PREPARE=[8000,8000,8000,8000,65535,64556]
RECIPE=[27500,31500,27000,20000,65535,64556]
FAULT_SHA='2eed977c200d1403e7edd6cbc6196717f2f51f55a46721977f4241f29d8a1c80'
VISUAL_SHA='95986ac0c5abcda8479d47e35ccf434c28242075152264c989996baf1bae8644'


def verify_delivery():
    checked={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if (ROOT not in path.parents or name in checked or len(digest)!=64
                or hashlib.sha256(path.read_bytes()).hexdigest()!=digest):
            raise ValueError('v8 delivery file differs: '+name)
        checked[name]=digest
    if not checked:raise ValueError('Empty v8 manifest')
    return checked


def accepted_pregrasp(path,expected_sha=None):
    # v8 leaves the successful v7 visual producer/validator completely frozen.
    return frozen_v7_pregrasp(path,expected_sha)


def grip_recipe_proof():
    old=previous_proof()
    return dict(previous=old,recipe=RECIPE[:],ring_only_change=[20500,20000],
        raw_source_sha256=FAULT_SHA,ring_fault_actual=20283,
        healthy_grip_or_actual_pickup_accepted=False,
        direct_lift_under_status3_permitted=False,automatic_added_closure_retry=False)


def qualified_supported_v7(path,pregrasp_path):
    raw=Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=FAULT_SHA:raise ValueError('Exact235346 v7 fault archive required')
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        names=z.namelist();root=names[0].split('/')[0]+'/'
        get=lambda n:json.loads(z.read(root+n))
        hashes=get('source_hashes.json');parent=get('result.json');worker=get('worker_result.json')
        if (z.testzip() or len(names)!=len(set(names))
                or set(hashes)!={n[len(root):] for n in names if n!=root+'source_hashes.json'}
                or any(hashlib.sha256(z.read(root+n)).hexdigest()!=h for n,h in hashes.items())
                or parent.get('worker')!=worker or parent.get('delivery_sha256')!=frozen_v7_delivery()
                or worker['cleanup_errors'] or worker['arm_actuator_writes']!=0
                or worker['hand_register_writes']!=1 or worker['controller_configuration_writes']!=0
                or worker.get('arm_segments') or worker.get('connected_transfer')
                or worker.get('grasp_feedback') or worker.get('grasp_qualified_groups')
                or [e['error'] for e in worker['exceptions']]!=[
                    "ProtectionFault('Current protection or unexpected force-control status: [2, 2, 2, 3, 2, 2]')"]):
            raise ValueError('235346 raw fault/command provenance differs')
        if (Path(pregrasp_path).read_bytes()!=z.read(root+'input_pregrasp.zip')
                or hashlib.sha256(Path(pregrasp_path).read_bytes()).hexdigest()!=VISUAL_SHA):
            raise ValueError('Completed healthy visual source must be the nested original')
        source=accepted_pregrasp(pregrasp_path,VISUAL_SHA)
        last=worker['last_snapshot']
        if (last['status']!=[2,2,2,3,2,2]
                or any(abs(a-b)>2 for a,b in zip(last['targets'],[27500,31500,27000,20500,65535,64556]))
                or last['positions'][3]!=20283 or last['current'][3]!=1086):
            raise ValueError('Ring-only protection provenance differs')
        return dict(source_sha256=FAULT_SHA,arm=source['arm'],configuration=source['configuration'],
            visual_source_sha256=VISUAL_SHA,old_fault_arm_state_used=False,
            startup_reference_role='completed_healthy_visual_endpoint_not_fault_last_arm_state',
            allowed_existing_status3_channels=[3],original_fault_feedback=last,
            no_lift_or_transfer_command_sent=True,support_must_be_freshly_confirmed=True)

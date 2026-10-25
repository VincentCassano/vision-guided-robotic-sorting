"""Live onsite grip tuning; previous fault archives are not startup authorities."""
import hashlib
import json
from pathlib import Path
import zipfile
from visual_pick_sort_v7_source import accepted_pregrasp,qualified_v6_bin,back40_offset

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pick-sort-v10-files.sha256'
PREPARE=[8000,8000,8000,8000,65535,64556]
RECIPE=[27500,31500,27000,19000,65535,64556]
CAPS=[30000,31500,29500,20250]
COMMANDS={'t+':(0,250),'t-':(0,-250),'i+':(1,250),'i-':(1,-250),
    'm+':(2,250),'m-':(2,-250),'r+':(3,250),'r-':(3,-250)}


def verify_delivery():
    out={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if (ROOT not in path.parents or name in out or len(digest)!=64
                or hashlib.sha256(path.read_bytes()).hexdigest()!=digest):
            raise ValueError('Live tune delivery differs: '+name)
        out[name]=digest
    if not out:raise ValueError('Empty manifest')
    return out


def validate_recipe(values):
    if (not isinstance(values,list) or len(values)!=6 or any(type(v) is not int for v in values)
            or values[4:]!=[65535,64556]
            or any(not 8000<=v<=cap for v,cap in zip(values[:4],CAPS))):
        raise ValueError('Recipe outside reviewed live-tuning bounds')
    return values[:]


def grip_recipe_proof():
    return dict(recipe=RECIPE[:],scope='initial_contact_candidate_not_holding_proof',
        caps=CAPS[:],step=250,seed_real_grip_verified=False,
        ring_cap_below_observed20500_protection_command=True,
        caps_are_supervised_candidates_not_manufacturer_safe_force_limits=True,
        all_status3_4_and_actual_current_guards_retained=True)


def accepted_recipe(path):
    """Derive targets from a locally saved, actually observed successful cycle."""
    record=json.loads(Path(path).read_bytes());archive=Path(record['archive'])
    raw=archive.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=record['archive_sha256']:raise ValueError('Accepted recipe source archive differs')
    with zipfile.ZipFile(archive) as z:
        names=z.namelist();root=names[0].split('/')[0]+'/'
        get=lambda n:json.loads(z.read(root+n))
        hashes=get('source_hashes.json');parent=get('result.json');worker=get('worker_result.json')
        if (z.testzip() or len(names)!=len(set(names))
                or set(hashes)!={n[len(root):] for n in names if n!=root+'source_hashes.json'}
                or any(hashlib.sha256(z.read(root+n)).hexdigest()!=h for n,h in hashes.items())
                or parent.get('worker')!=worker or parent.get('delivery_sha256')!=verify_delivery()
                or parent.get('exceptions') or not parent.get('new_cycle_real_device_accepted')
                or worker.get('outcome')!='completed_school_bin_drop_feedback_and_operator_empty_confirmation'
                or worker.get('exceptions') or worker.get('cleanup_errors')
                or worker.get('shell_in_selected_bin_observed_by_operator') is not True):
            raise ValueError('Only a complete actually observed cycle can supply a saved recipe')
        return dict(recipe=validate_recipe(worker['new_grip_receipt']['recipe']),
            configuration=worker['observed_run_configuration'],source_archive_sha256=record['archive_sha256'],
            actual_success_scope='onsite_operator_observed_single_black_shell_cycle_not_force_sensor')

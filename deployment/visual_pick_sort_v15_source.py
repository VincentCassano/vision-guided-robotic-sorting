"""Keep the accepted v14 visual binding; use a separate v15 delivery inventory."""
import hashlib

from visual_pick_sort_v14_source import (ROOT,SUCCESS_SHA,accepted_success,accepted_anchor,
    make_anchor,target_candidate,accepted_request,bin_rows)

MANIFEST=ROOT/'deployment/visual-pick-sort-v15-files.sha256'


def verify_delivery():
    inventory={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT not in path.parents or name in inventory or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Delivery file differs: '+name)
        inventory[name]=digest
    if not inventory:raise ValueError('Empty delivery manifest')
    return inventory

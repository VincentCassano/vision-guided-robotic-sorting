"""Freeze observed configuration for one supervised run; never set payloads."""
import copy
import hashlib
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'deployment/visual-pregrasp-motion-v3-files.sha256'


def verify_delivery():
    inventory = {}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        expected, name = line.split('  ', 1)
        path = (ROOT / name).resolve()
        if (ROOT not in path.parents or name in inventory or len(expected) != 64
                or hashlib.sha256(path.read_bytes()).hexdigest() != expected):
            raise ValueError('Deployment changed: ' + name)
        inventory[name] = expected
    if not inventory:
        raise ValueError('Empty deployment inventory')
    return inventory


def _get(role, wire):
    from read_visual_collision_snapshot import query, QUERIES
    row = query(next(spec for spec in QUERIES if spec[0] == role), wire)
    if row.get('error') or row.get('close_error') or row.get('socket_closed') is not True:
        raise ValueError('Current configuration GET failed: ' + role)
    return row['response']


def observe_configuration(wire):
    rows = {role: _get(role, wire) for role in
            ('identity', 'active_tool_payload_raw', 'active_work_frame_raw', 'installation')}
    tool = rows['active_tool_payload_raw']
    if (set(tool) != {'state', 'tool_name', 'pose', 'payload', 'position'}
            or tool['state'] != 'current_tool_frame'
            or not isinstance(tool['tool_name'], str) or not tool['tool_name']
            or tool['pose'] != [0] * 6 or any(type(v) is not int for v in tool['pose'])
            or type(tool['payload']) is not int or not 0 <= tool['payload'] <= 5000
            or not isinstance(tool['position'], list) or len(tool['position']) != 3
            or any(type(v) is not int for v in tool['position'])):
        raise ValueError('Only current zero-offset flange geometry and valid raw payload fields supported')
    work = rows['active_work_frame_raw']
    if (work.get('frame_name') != 'World' or work.get('pose') != [0] * 6
            or any(type(v) is not int for v in work['pose'])):
        raise ValueError('Current World geometry differs from visual reference')
    identity = rows['identity']
    if (identity.get('Product_version') != 'RM65-BI'
            or identity.get('ctrl_info', {}).get('version') != 'V1.6.5'
            or identity.get('plan_info', {}).get('version') != 'V1.6.5'
            or identity.get('algorithm_info', {}).get('version') != '1.4.10'
            or identity.get('dynamic_info', {}).get('model_version') != '2'):
        raise ValueError('Actual reviewed RM65-BI firmware identity required')
    return dict(raw=rows, sdk_frames=None,
                configuration_policy='freeze_existing_observed_configuration_no_setters',
                payload_physical_parameters_verified=False,
                payload_identification_required=False,
                sdk_COM_unit_crosscheck_performed=False)


def require_reference_geometry(run, reference):
    # Names/dynamics can differ; they do not redefine a zero-offset flange TCP.
    for kind in ('work', 'tool'):
        frame = reference[kind]
        if any(v != 0 for group in ('position', 'euler') for v in frame[group].values()):
            raise ValueError('Historical reference is not zero-offset geometry')
    if reference['work']['name'] != 'World':
        raise ValueError('Historical World differs')


def bind_or_require_frames(run, frames):
    if run['sdk_frames'] is not None:
        if frames != run['sdk_frames']:
            raise ValueError('Current run work/tool configuration changed')
        return
    raw_tool = run['raw']['active_tool_payload_raw']
    if set(frames) != {'work', 'tool'}:
        raise ValueError('Incomplete SDK work/tool snapshot')
    for kind, frame in frames.items():
        if set(frame) != {'name', 'position', 'euler', 'payload'}:
            raise ValueError('Incomplete SDK frame')
        for group, axes in (('position', {'x', 'y', 'z'}), ('euler', {'rx', 'ry', 'rz'})):
            if set(frame[group]) != axes or any(type(v) not in (int, float)
                    or not math.isfinite(v) or v != 0 for v in frame[group].values()):
                raise ValueError('SDK frame is not zero-offset flange/World')
        if set(frame['payload']) != {'payload', 'x', 'y', 'z'} or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in frame['payload'].values()):
            raise ValueError('Incomplete finite SDK dynamics')
    if (frames['work']['name'] != 'World' or frames['tool']['name'] != raw_tool['tool_name']
            or not math.isclose(frames['tool']['payload']['payload'], raw_tool['payload'] / 1000,
                                rel_tol=0, abs_tol=1e-6)):
        raise ValueError('Raw/SDK active tool identity or mass disagree')
    run['sdk_frames'] = copy.deepcopy(frames)


def require_current_configuration(run, wire):
    # Called with the hand TCP closed; preserve the original lane separation.
    for role in ('active_tool_payload_raw', 'active_work_frame_raw', 'installation'):
        if _get(role, wire) != run['raw'][role]:
            raise ValueError('Observed configuration changed during this run: ' + role)
    return copy.deepcopy(run['raw']['active_tool_payload_raw'])

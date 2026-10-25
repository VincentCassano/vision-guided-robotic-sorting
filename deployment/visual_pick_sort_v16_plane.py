"""Select the original recess top plane or its measured board-top height.

Only the plane changes. project_observation re-intersects every live camera
ray, so the resulting XYZ is not an old point with a Z-only correction.
"""
import copy
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SUPPORT_FILE=ROOT/'deployment/visual_pick_sort_v16_support.json'


def support_model():
    raw=SUPPORT_FILE.read_bytes();value=json.loads(raw)
    height=value.get('board_top_above_recess_bottom_mm')
    if (value.get('reference_support')!='recess_bottom' or value.get('second_support')!='board_top'
            or value.get('measurement_source')!='operator_vernier_caliper'
            or value.get('translation_axis')!='World_Z_positive'
            or type(height) not in (int,float) or not math.isfinite(height) or height<=0):
        raise ValueError('Measured board-top/recess height source required')
    return value,hashlib.sha256(raw).hexdigest()


def support_plane(base,surface):
    if surface not in ('recess','board_top'):
        raise ValueError('Select recess or measured board top; no arbitrary-height assumption')
    value,digest=support_model();plane=copy.deepcopy(base)
    rise=value['board_top_above_recess_bottom_mm'] if surface=='board_top' else 0.
    if surface=='board_top':
        plane['plane_point_world_m'][2]+=rise/1000.
        plane['same_upright_shell_equal_seated_top_plane_confirmed']=False
        plane['measured_support_height_shift']=dict(surface=surface,rise_mm=rise,
            source_sha256=digest,translation_world_m=[0.,0.,rise/1000.],
            height_detected_automatically=False,base_equal_height_plane=copy.deepcopy(base))
    return plane,rise,digest

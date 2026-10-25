"""Bind all original burst class observations to the matching school bin."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
CLASSES={0:('black_cyl_battery','black'),1:('red_cyl_battery','red'),2:('white_cyl_battery','white')}


def specification():
    value=json.loads((ROOT/'deployment/cylinder_color_spec_v23.json').read_bytes())
    if (value.get('same_dimensions_confirmed_by_user') is not True
            or value.get('inert_test_objects') is not True
            or value.get('diameter_mm')!=45.8 or value.get('total_height_mm')!=98.):
        raise ValueError('Confirm red/white shell dimensions before using the black-shell geometry')
    return value


def classify_frames(frames):
    spec=specification();observations=[]
    for row in frames:
        detection=row.get('selected_detection',{})
        cid=detection.get('class_id')
        if type(cid) is not int or cid not in CLASSES:
            raise ValueError('Original frame lacks a supported cylinder class')
        name,color=CLASSES[cid]
        confidence=detection.get('confidence')
        if (detection.get('class_name')!=name or type(confidence) not in (int,float)
                or not spec['class_confidences'][cid]<=confidence<=1.):
            raise ValueError('Original class name/confidence differs from the model policy')
        observations.append(dict(frame_number=row['frame_number'],timestamp_ms=row['timestamp_ms'],
            class_id=cid,class_name=name,color=color,confidence=confidence,
            RGB_file=row.get('burst_RGB_file'),RGB_sha256=row.get('burst_RGB_sha256')))
    if len(observations)<8 or len({r['class_id'] for r in observations})!=1:
        raise ValueError('All original burst frames must agree on the cylinder class')
    if any(b['frame_number']<=a['frame_number'] or b['timestamp_ms']<=a['timestamp_ms']
            for a,b in zip(observations,observations[1:])):
        raise ValueError('Class observations must come from strictly fresh frames')
    if any(not isinstance(r['RGB_sha256'],str) or len(r['RGB_sha256'])!=64 for r in observations):
        raise ValueError('Original RGB class evidence hashes are missing')
    first=observations[0]
    return dict(class_id=first['class_id'],class_name=first['class_name'],color=first['color'],
        logical_bin='black_bin' if first['class_id']==0 else 'red_white_bin',
        classification_basis='same_YOLO_class_in_every_original_burst_frame',
        frame_count=len(observations),observations=observations,
        hand_pose_changed_by_color=False,
        dimensions=dict(diameter_mm=spec['diameter_mm'],total_height_mm=spec['total_height_mm']))


def accepted_classification(point):
    rows=point.get('classification_frames')
    if not isinstance(rows,list):raise ValueError('Original class-frame evidence is missing')
    expected=classify_frames(rows)
    if expected!=point.get('target_classification'):
        raise ValueError('Target color differs from the original burst classifications')
    return expected

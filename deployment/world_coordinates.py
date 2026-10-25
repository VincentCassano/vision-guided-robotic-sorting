"""Read-only surface coordinate conversion, never robot target generation."""
import hashlib
import json
import numpy as np


class WorldCoordinates:
    def __init__(self,path):
        raw=path.read_bytes(); self.sha256=hashlib.sha256(raw).hexdigest()
        self.data=json.loads(raw); self.matrix=np.array(self.data['world_from_camera'],dtype=float)
        a=self.matrix
        if a.shape!=(4,4) or not np.isfinite(a).all() or not np.allclose(a[3],[0,0,0,1]):
            raise ValueError('Invalid world transform')
        r=a[:3,:3]
        if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1,atol=1e-6):
            raise ValueError('World transform is not a proper rigid transform')
        if self.data['context']['frames']['work']['frame_name']['name']!='World':
            raise ValueError('Expected World frame')

    def check_camera(self,serial,intr):
        context=self.data['context']
        k=[[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]]
        if serial!=context['serial'] or not np.allclose(k,context['K'],rtol=0,atol=1e-4):
            raise ValueError('Camera identity or intrinsics differ from calibration')
        if str(intr.model)!=context['distortion_model'] or not np.allclose(intr.coeffs,context['distortion'],rtol=0,atol=1e-8):
            raise ValueError('Camera distortion differs from calibration')

    def convert(self,detection):
        detection.update(position_world_m=None,position_world_mm=None,execution_ready=False,
                         world_point_kind='visible_surface_not_tcp_target')
        if not detection.get('depth_valid'): return
        point=np.asarray(detection['position_optical_m'],dtype=float)
        if point.shape!=(3,) or not np.isfinite(point).all(): raise ValueError('Invalid optical point')
        result=self.matrix[:3,:3]@point+self.matrix[:3,3]
        detection.update(position_world_m=result.tolist(),position_world_mm=(result*1000).tolist())


def draw_world_panel(view,detections):
    import cv2
    height=max(view.shape[0],120+len(detections)*54)
    panel=np.full((height,view.shape[1]+460,3),25,np.uint8)
    panel[:view.shape[0],:view.shape[1]]=view
    x=view.shape[1]+12
    def line(text,y,color=(230,230,230)):
        cv2.putText(panel,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1,cv2.LINE_AA)
    line('World XYZ (mm) | READ ONLY',25,(0,255,255))
    line('Visible surface points, NOT grasp targets',48)
    line('Head/base/lift unchanged. No robot commands.',70)
    for i,d in enumerate(detections):
        y=103+i*54
        state='OK' if d.get('visual_confirmed') else 'WAIT'
        line('#%s %s %s'%(d.get('track_id','?'),d['class_name'],state),y)
        point=d.get('position_world_mm')
        line(('X %+.1f  Y %+.1f  Z %+.1f'%tuple(point)) if point else 'DEPTH INVALID',y+21,(0,255,255))
    return panel

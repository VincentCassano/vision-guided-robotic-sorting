"""Synthetic RGB-D generation and image-based classification for stage 4."""
import cv2
import numpy as np

from .sorting_core import CLASSES, UnsafePlan, inside, slot

COLORS = {'NORMAL_BLACK':(25,25,25), 'DEFECT_RED':(25,25,230), 'DEFECT_WHITE':(245,245,245)}


def project_pixel(detection, depth, intrinsics, min_depth, max_depth, radius, scale=1.0):
    fx, fy, cx, cy = intrinsics
    if not np.isfinite(intrinsics).all() or fx <= 0 or fy <= 0:
        raise UnsafePlan('Invalid camera intrinsics')
    if depth.ndim != 2 or not np.isfinite([min_depth, max_depth, scale]).all() or not 0 < min_depth < max_depth or scale <= 0:
        raise UnsafePlan('Invalid depth configuration')
    if type(radius) is not int or not 0 <= radius <= 20:
        raise UnsafePlan('Invalid depth window')
    if not np.isfinite([detection['u'], detection['v']]).all():
        raise UnsafePlan('Nonfinite pixel')
    u, v = int(round(detection['u'])), int(round(detection['v']))
    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
        raise UnsafePlan('Pixel outside depth image')
    roi = depth[max(0, v-radius):v+radius+1, max(0, u-radius):u+radius+1].astype(np.float64) * scale
    valid = roi[np.isfinite(roi) & (roi >= min_depth) & (roi <= max_depth)]
    if not valid.size:
        raise UnsafePlan('No valid depth')
    z = float(np.median(valid))
    return np.array([(detection['u']-cx)*z/fx, (detection['v']-cy)*z/fy, z])


def render_scene(cfg, completed=()):
    camera = cfg['camera']
    rgb = np.full((480,640,3), 145, dtype=np.uint8)
    depth = np.zeros((480,640),dtype=np.float32)
    for row,v in enumerate(camera['row_pixels']):
        for col,u in enumerate(camera['column_pixels']):
            identity = 'r{}c{}'.format(row,col)
            if identity in completed:
                continue
            category = cfg['scene_categories'][row][col]
            rgb[v-24:v+25,u-15:u+16] = COLORS[category]
            depth[v-24:v+25,u-15:u+16] = camera['depth']
    return rgb,depth


def detect_batteries(rgb):
    hsv = cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
    masks = [cv2.inRange(hsv,(0,0,0),(179,255,60)),
             cv2.bitwise_or(cv2.inRange(hsv,(0,120,80),(10,255,255)),cv2.inRange(hsv,(170,120,80),(179,255,255))),
             cv2.inRange(hsv,(0,0,210),(179,55,255))]
    detections = []
    for category,mask in zip(CLASSES,masks):
        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 100:
                continue
            x,y,w,h = cv2.boundingRect(contour)
            detections.append({'category':category,'u':x+(w-1)/2.0,'v':y+(h-1)/2.0,'bbox':[x,y,w,h]})
    return sorted(detections,key=lambda d:(d['u'],d['v']))


def localize(detections, depth, intrinsics, cfg):
    fx,fy,cx,cy = intrinsics
    if not np.isfinite(intrinsics).all() or fx <= 0 or fy <= 0:
        raise UnsafePlan('Invalid camera intrinsics')
    targets,rejected = [],[]
    for detection in detections:
        try:
            optical = project_pixel(detection, depth, intrinsics, cfg['min_depth'], cfg['max_depth'], cfg['depth_window'])
            position = (np.array(cfg['camera']['rotation']) @ optical + np.array(cfg['camera']['translation'])).tolist()
            if not inside(position,cfg['workspace']):
                raise UnsafePlan('Target outside base_link workspace')
            row,col,identity = slot(position,cfg)
            targets.append({'id':identity,'row':row,'column':col,'category':detection['category'],'position':position})
        except (UnsafePlan,KeyError,ValueError,TypeError) as exc:
            rejected.append(str(exc))
    return targets,rejected

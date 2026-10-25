"""Photo-cutout simulation fixture; not new validation data or measured depth."""
from pathlib import Path
import cv2
import numpy as np
from .sorting_core import CLASSES, require


class PhotoScene:
    def __init__(self, directory):
        directory = Path(directory)
        image = cv2.imread(str(directory/'images/IMG_7865.JPG'))
        require(image is not None, 'Missing photo fixture IMG_7865.JPG')
        height,width = image.shape[:2]
        self.patches = {}
        for line in (directory/'labels/IMG_7865.txt').read_text().splitlines():
            cid,x,y,w,h = map(float,line.split())
            cid = int(cid)
            if cid not in (0,1,2) or CLASSES[cid] in self.patches:
                continue
            x1,y1 = max(0,int((x-w/2)*width)),max(0,int((y-h/2)*height))
            x2,y2 = min(width,int((x+w/2)*width)+1),min(height,int((y+h/2)*height)+1)
            patch = image[y1:y2,x1:x2]
            patch_width = max(1,int(round(80*patch.shape[1]/patch.shape[0])))
            self.patches[CLASSES[cid]] = cv2.resize(patch,(patch_width,80),interpolation=cv2.INTER_AREA)
        require(len(self.patches)==3,'Photo fixture must contain all three supported classes')

    def render(self,cfg,completed=()):
        rgb = np.full((480,640,3),145,np.uint8)
        depth = np.zeros((480,640),np.float32)
        for row,v in enumerate(cfg['camera']['row_pixels']):
            for col,u in enumerate(cfg['camera']['column_pixels']):
                if 'r{}c{}'.format(row,col) in completed:
                    continue
                patch = self.patches[cfg['scene_categories'][row][col]]
                h,w = patch.shape[:2]
                x,y = u-w//2,v-h//2
                rgb[y:y+h,x:x+w] = patch
                depth[y:y+h,x:x+w] = cfg['camera']['depth']
        return rgb,depth

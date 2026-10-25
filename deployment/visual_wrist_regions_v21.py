"""Mark four slot centers on the fresh binding photograph; no robot/camera SDK."""
import argparse
import json
from pathlib import Path
from known_cylinder_records import dump
from visual_pick_sort_v21_source import grid_matrix,NEAR_CELLS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--context',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stop-file',type=Path)
    args=p.parse_args()
    import cv2
    import numpy as np
    context=json.loads(args.context.read_bytes());raw=cv2.imread(str(args.image))
    if raw is None or [raw.shape[1],raw.shape[0]]!=context['image_size']:
        raise ValueError('Fresh binding image/context required')
    points=[];window='WRIST REGIONS | near-left -> near-right -> far-right -> far-left | S save R redraw Q cancel'
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and 0<=x<raw.shape[1] and 0<=y<raw.shape[0] and len(points)<4:
            points.append([float(x),float(y)])
    cv2.namedWindow(window,cv2.WINDOW_AUTOSIZE);cv2.setMouseCallback(window,click)
    print('只标一次四个角槽中心：近排左 → 近排右 → 远排右 → 远排左；点槽心，不点电池顶盖。S保存，R重画。绿色五槽用新腕姿。',flush=True)
    try:
        while True:
            if args.stop_file is not None and args.stop_file.exists():return 2
            image=raw.copy();record=dict(schema_version=1,role='wrist_selection_only',polygon_px=points,
                camera_context=context,near_cells=NEAR_CELLS,
                corner_order=['near_left','near_right','far_right','far_left'])
            ready=False
            if len(points)==4:
                try:
                    h=grid_matrix(record,context);inverse=np.linalg.inv(h)
                    overlay=image.copy()
                    for row,col in NEAR_CELLS:
                        vertices=[]
                        for u,v in [(col*.5-.25,row*.5-.25),(col*.5+.25,row*.5-.25),
                                    (col*.5+.25,row*.5+.25),(col*.5-.25,row*.5+.25)]:
                            q=inverse@np.array([u,v,1.]);vertices.append((q[:2]/q[2]).astype(int))
                        cv2.fillPoly(overlay,[np.array(vertices,np.int32)],(0,210,40))
                    image=cv2.addWeighted(overlay,.30,image,.70,0);ready=True
                except Exception as exc:cv2.putText(image,str(exc)[:90],(12,65),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),2)
            for i,(x,y) in enumerate(points):
                cv2.circle(image,(int(x),int(y)),7,(0,0,255),2)
                cv2.putText(image,str(i+1),(int(x)+9,int(y)-8),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
            cv2.putText(image,'CORNERS %d/4 | GREEN = NEW NEAR WRIST | S SAVE / R REDRAW / Q CANCEL'%len(points),
                (12,28),cv2.FONT_HERSHEY_SIMPLEX,.60,(0,255,255),2)
            cv2.imshow(window,image);key=cv2.waitKey(30)&0xff
            if key in (ord('q'),ord('Q'),27):return 2
            if key in (ord('r'),ord('R')):points.clear()
            if key in (ord('s'),ord('S')) and ready:
                dump(args.output,record)
                cv2.imwrite(str(args.output.with_suffix('.png')),image)
                print('已保存：最近排三个及中排右两个使用新腕姿，其余使用原腕姿。',flush=True)
                return 0
    finally:cv2.destroyWindow(window)


if __name__=='__main__':raise SystemExit(main())

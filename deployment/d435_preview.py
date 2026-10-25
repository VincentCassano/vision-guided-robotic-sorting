"""Read-only D435 RGB-D and optional World surface coordinates; no robot control."""
import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
import yaml
from camera_preview import ROOT, draw_frame
from battery_grasp.inference_client import InferenceClient
from depth_preview import measure
from battery_grasp.temporal_confirmation import TemporalConfirmation
from world_coordinates import WorldCoordinates, draw_world_panel


def main():
    parser=argparse.ArgumentParser(description='Head D435 RGB-D / World surface preview; no robot control')
    parser.add_argument('--inference-python',required=True)
    parser.add_argument('--serial',default='344322075008')
    parser.add_argument('--width',type=int,default=1280)
    parser.add_argument('--height',type=int,default=720)
    parser.add_argument('--fps',type=int,default=30)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--depth',action='store_true',help='Diagnostic aligned depth and surface XYZ; no robot control')
    parser.add_argument('--world-calibration',type=Path,help='Read-only candidate World transform')
    parser.add_argument('--confirm-fixed-camera',action='store_true',help='Confirm head, base and lift unchanged since calibration')
    parser.add_argument('--weights',default=str(ROOT/'src/battery_grasp/models/battery_yolov5s_v3_last.pt'))
    parser.add_argument('--self-test',action='store_true',help='Test model on the supplied photo without camera access')
    parser.add_argument('--check',action='store_true',help='List SDK devices only; do not start streaming')
    args=parser.parse_args()
    world=None
    if args.world_calibration:
        if not args.depth or not args.confirm_fixed_camera:
            raise ValueError('World preview requires --depth --confirm-fixed-camera; head/base/lift must be unchanged')
        world=WorldCoordinates(args.world_calibration)
    (ROOT/'docs').mkdir(exist_ok=True)
    def log(command):
        with (ROOT/'docs/执行命令日志.txt').open('a',encoding='utf-8') as out:
            out.write(subprocess.list2cmdline(command)+'\n')
    log([sys.executable,*sys.argv])
    if args.self_test:
        client=InferenceClient(args.inference_python,str(ROOT/'src/battery_grasp/scripts/yolo_worker.py'),str(ROOT/'deployment/yolov5-runtime'),args.weights,device=args.device)
        log(client.process.args)
        try:
            image=ROOT/'src/battery_grasp/test_data/battery_v1/images/IMG_7862.JPG'
            detections=client.detect(cv2.imread(str(image)))
            expected=[int(line.split()[0]) for line in (image.parent.parent/'labels/IMG_7862.txt').read_text().splitlines() if line.strip()]
            assert sorted(d['class_id'] for d in detections)==sorted(expected),detections
            print('PASS: model self-test, objects:',len(detections),'camera not opened')
        finally: client.close()
        return
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError('This camera Python has no pyrealsense2. Set CAMERA_PYTHON to the interpreter used by the robot camera SDK. Do not change ROS/system Python.') from exc
    context=rs.context()
    devices=[(dev.get_info(rs.camera_info.serial_number),dev.get_info(rs.camera_info.name)) for dev in context.query_devices()]
    print('RealSense devices:',devices,flush=True)
    if args.serial not in [serial for serial,_ in devices]:
        raise RuntimeError('Head D435 '+args.serial+' not found; no other camera selected.')
    if args.check:
        print('PASS: head D435 visible to SDK. Streaming/model not tested.')
        return
    values=yaml.safe_load((ROOT/'src/battery_grasp/config/yolo_v3_calibrated.yaml').read_text())['battery_detector_yolo']['class_confidences']
    calibrated=True
    client=None;pipeline=None;started=False
    try:
        client=InferenceClient(args.inference_python,str(ROOT/'src/battery_grasp/scripts/yolo_worker.py'),
            str(ROOT/'deployment/yolov5-runtime'),args.weights,
            device=args.device)
        log(client.process.args)
        pipeline=rs.pipeline(context);config=rs.config();config.enable_device(args.serial)
        config.enable_stream(rs.stream.color,args.width,args.height,rs.format.bgr8,args.fps)
        if args.depth: config.enable_stream(rs.stream.depth,640,480,rs.format.z16,args.fps)
        profile=pipeline.start(config);started=True
        align=rs.align(rs.stream.color) if args.depth else None
        depth_scale=profile.get_device().first_depth_sensor().get_depth_scale() if args.depth else None
        actual=profile.get_stream(rs.stream.color).as_video_stream_profile()
        if world: world.check_camera(args.serial,actual.get_intrinsics())
        print('Actual RGB:',actual.width(),actual.height(),actual.fps(),'serial:',args.serial,flush=True)
        title='Head D435 - battery v3';cv2.namedWindow(title,cv2.WINDOW_NORMAL)
        measured=0
        confirmation=TemporalConfirmation()
        timestamp_domain=None
        while True:
            begin=time.perf_counter()
            frames=pipeline.wait_for_frames(5000)
            if align is not None: frames=align.process(frames)
            color=frames.get_color_frame()
            if not color:
                confirmation.reset()
                continue
            frame=np.asanyarray(color.get_data())
            depth_frame=frames.get_depth_frame() if args.depth else None
            if args.depth and not depth_frame:
                confirmation.reset()
                continue
            depth=np.asanyarray(depth_frame.get_data()) if args.depth else None
            intr=depth_frame.profile.as_video_stream_profile().intrinsics if args.depth else None
            detections=client.detect(frame)
            if calibrated: detections=[d for d in detections if d['confidence']>=values[d['class_id']]]
            domain=str(color.get_frame_timestamp_domain())
            if domain!=timestamp_domain: confirmation.reset()
            timestamp_domain=domain
            detections=confirmation.update(detections,color.get_timestamp()/1000.0)
            annotated=frame.copy()
            metadata=dict(execution_ready=False,confirmation_seconds=1.0,
                          confirmation_scope='consecutive_processed_frames',max_observation_gap_seconds=.5)
            if args.depth:
                if depth.shape != frame.shape[:2]: raise RuntimeError('Aligned depth size does not match RGB')
                metadata.update(schema_version=1,depth_source='d435_aligned',execution_ready=False,
                    coordinate_frame='color_optical',axes='x right, y down, z forward; metres',
                    point_kind='bbox_center_surface_not_grasp_pose',depth_scale_m=depth_scale,
                    color_timestamp_ms=color.get_timestamp(),depth_timestamp_ms=depth_frame.get_timestamp(),
                    timestamp_domain=str(color.get_frame_timestamp_domain()),
                    intrinsics=dict(width=intr.width,height=intr.height,fx=intr.fx,fy=intr.fy,ppx=intr.ppx,ppy=intr.ppy,model=str(intr.model),coeffs=list(intr.coeffs)))
                for d in detections:
                    d.update(measure(depth,depth_scale,d['u'],d['v'],intr,rs.rs2_deproject_pixel_to_point))
                    if world: world.convert(d)
                    xyz=d['position_optical_m']
                    label=('#%s'%d['track_id']) if world else (('CAM %.3f %.3f %.3f m'%tuple(xyz)) if xyz else 'DEPTH INVALID')
                    location=(max(0,int(d['bbox'][0])),min(frame.shape[0]-8,int(d['bbox'][1]+d['bbox'][3])+18))
                    cv2.putText(annotated,label,location,cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),2)
            rate=1/max(time.perf_counter()-begin,1e-6)
            measured=rate if not measured else .8*measured+.2*rate
            metadata['vision_candidates']=confirmation.candidates(detections)
            view=draw_frame(annotated,detections,measured,'D435 RGB-D' if args.depth else 'D435',calibrated)
            if world:
                metadata.update(world_frame='World',world_transform_sha256=world.sha256,
                                world_from_camera=world.matrix.tolist(),world_units='m and mm in explicitly named fields',
                                camera_mount_unchanged_user_confirmed=True,world_point_kind='visible_surface_not_tcp_target',
                                calibration_diagnostic_pass=False,execution_ready=False)
                view=draw_world_panel(view,detections)
            cv2.imshow(title,view)
            key=cv2.waitKey(1)&255
            if key in (27,ord('q'),ord('Q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1: break
            if key in (ord('t'),ord('T')):
                calibrated=not calibrated
                confirmation.reset()
            if key in (ord('s'),ord('S')):
                folder=ROOT/'captures'/datetime.now().strftime('%Y%m%d_%H%M%S_%f');folder.mkdir(parents=True)
                if not cv2.imwrite(str(folder/'original.png'),frame) or not cv2.imwrite(str(folder/'detections.png'),view):
                    raise RuntimeError('Failed to save captured images')
                if args.depth and not cv2.imwrite(str(folder/'aligned_depth_u16.png'),depth):
                    raise RuntimeError('Failed to save aligned depth')
                (folder/'detections.json').write_text(json.dumps(dict(serial=args.serial,width=frame.shape[1],height=frame.shape[0],
                    thresholds=values if calibrated else [.25]*4,detections=detections,**metadata),indent=2),encoding='utf-8')
                print('Saved:',folder,flush=True)
    finally:
        try:
            if started: pipeline.stop()
        finally:
            if client is not None: client.close()
            cv2.destroyAllWindows()


if __name__=='__main__':
    try: main()
    except KeyboardInterrupt: pass
    except Exception as exc:
        print('ERROR:',exc,file=sys.stderr)
        sys.exit(1)

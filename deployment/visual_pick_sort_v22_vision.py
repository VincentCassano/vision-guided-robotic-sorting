"""Capture a visual point for a new wrist reference; no old TCP mapping or robot access."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback
from known_cylinder_records import archive_folder, dump

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def bbox_iou(a,b):
    ax,ay,aw,ah=a;bx,by,bw,bh=b
    intersection=max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
    return intersection/max(aw*ah+bw*bh-intersection,1e-9)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role',choices=('reference','target'),required=True)
    parser.add_argument('--position-session',type=Path,required=True)
    parser.add_argument('--confirm-fixed-camera',action='store_true',required=True)
    parser.add_argument('--confirm-measured-shell-only',action='store_true',required=True)
    parser.add_argument('--inference-python',default='/usr/bin/python3')
    parser.add_argument('--device',default='auto')
    parser.add_argument('--confirm-unchanged-setup',action='store_true',required=True)
    parser.add_argument('--anchor',type=Path,required=True,help='Reviewed historical visual evidence for equal-height plane geometry only')
    parser.add_argument('--grasp-source',type=Path,default=ROOT/'deployment/visual_binding_grasp_source_123051.json')
    parser.add_argument('--pregrasp-source',type=Path,default=ROOT/'deployment/visual_binding_pregrasp_source_024412.json')
    parser.add_argument('--pregrasp-crosscheck',type=Path,default=ROOT/'deployment/visual_binding_pregrasp_crosscheck_081023.json')
    parser.add_argument('--confirm-upright-shell-on-this-board',action='store_true',required=True)
    parser.add_argument('--finish-after-first-reference',action='store_true')
    parser.add_argument('--automatic',action='store_true')
    parser.add_argument('--define-workspace',action='store_true')
    parser.add_argument('--observe-only',action='store_true')
    parser.add_argument('--workspace-config',type=Path)
    parser.add_argument('--completed-ledger',type=Path)
    parser.add_argument('--scan-response',type=Path)
    parser.add_argument('--stop-file',type=Path)
    args=parser.parse_args()
    folder=ROOT/'captures'/('visual_pick_point_v22_'+args.role+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    pipeline=None;client=None;started=False;saved=0;capture=None;points=[];binding=None;target_count=0
    last_capture_message=None;last_capture_pass=None;plane=None
    workspace_points=[];workspace_clicks=[];ledger=[];scan_rows=[]
    auto_stable_since=None;auto_stable_frames=0;auto_previous_bbox=None
    inventory_since=None;inventory_frames=0;inventory_kind=None
    scan_started=time.monotonic()
    report=dict(scope='new_wrist_reference_visual_top_point_only',
            delivery_version='visual-pick-point-v22-fix2-20261024',capture_role=args.role,old_grasp_tcp_mapping_used=False,
        old_2p8_plus_4p5_offsets_used=False,new_wrist_pose_read_by_this_process=False,
        robot_SDK_loaded=False,arm_connections=0,arm_writes=0,hand_writes=0,
        controller_configuration_writes=0,camera_option_writes=0,execution_ready=False,
        full6D_handeye_pass=False,RGBD_position_validation_pass=False,
        object_geometry_independently_accepted=False,source_session=str(args.position_session))
    try:
        from visual_task_anchor import accepted_grasp,validate_saved_anchor
        from visual_equal_height_plane import make_plane
        from visual_pick_sort_v22_auto_height import automatic_projection,observed_rise,upright_measurement
        from visual_batch_workspace_v20 import polygon,contains,foot,same_detection,front_first
        from visual_auto_recovery_v22 import VisualRecovery
        recovery=VisualRecovery()
        grasp=accepted_grasp(args.grasp_source.read_bytes(),args.pregrasp_source.read_bytes(),
            args.pregrasp_crosscheck.read_bytes())
        report['historical_grasp_metadata_role']='only_validate_original_visual_plane_provenance_not_new_wrist_or_TCP'
        report['historical_grasp_metadata']=grasp
        candidate_path=ROOT/'deployment/camera_position_candidate_225543.json'
        candidate=json.loads(candidate_path.read_text(encoding='utf8'));candidate_sha=sha(candidate_path)
        qualification=json.loads((ROOT/'deployment/marker_pnp_position_005012_evidence.json').read_text(encoding='utf8'))
        session=args.position_session.resolve()
        validation=json.loads((session/'position_validation_result.json').read_text(encoding='utf8'))
        if (candidate_sha!=qualification['candidate_sha256'] or validation['candidate_sha256']!=candidate_sha
                or qualification['marker_pnp_position_consistency_pass'] is not True
                or candidate['fit_count']!=16 or candidate['fit_position_pass'] is not True
                or candidate['excluded_samples'] or candidate['validation_used_for_fit']):
            raise ValueError('Frozen position candidate and reviewed marker evidence required')
        rows=validation['validation']
        if (len(rows)!=6 or {r['id'] for r in rows}!={'001','002','003','004','005','006'}
                or any(not 0<=r['position_error_pnp_mm']<=5 for r in rows)
                or validation.get('source_candidate_changed') or validation.get('validation_used_for_fit')
                or validation.get('error')):
            raise ValueError('All six original marker-PnP positions required; no promotion of RGBD gate')
        for name,digest in qualification['validation_source_hashes'].items():
            target=(session/name).resolve()
            if target.parent!=session or sha(target)!=digest:raise ValueError('Reviewed evidence changed: '+name)
        model=json.loads((ROOT/'deployment/black_cylinder_measured_20261020.json').read_text(encoding='utf8'))
        weights=ROOT/'src/battery_grasp/models/battery_v3_camera.torchscript'
        if not weights.is_file():raise FileNotFoundError('Keep the existing accepted TorchScript model: '+str(weights))
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        import yaml
        from battery_grasp.inference_client import InferenceClient
        from known_cylinder_geometry_axis import locate
        import known_cylinder_capture as burst
        thresholds=yaml.safe_load((ROOT/'src/battery_grasp/config/yolo_v3_calibrated.yaml').read_text())['battery_detector_yolo']['class_confidences']
        report.update(candidate_sha256=candidate_sha,marker_position_qualification=qualification,
            measured_shell=model,weights_sha256=sha(weights),class_confidences=thresholds,
            new_reference_place_confirmation_required_in_parent=True,historical_plane_geometry_reused=True,
            new_wrist_reference_point_must_be_captured_this_run=True,
            runtime_source_sha256={name:sha(ROOT/'deployment'/name) for name in
                ('visual_pick_sort_v22_vision.py','visual_task_anchor.py','known_cylinder_geometry_axis.py',
                 'known_cylinder_capture.py','known_cylinder_records.py','visual_equal_height_plane.py',
                 'visual_pick_sort_v16_plane.py','visual_pick_sort_v18_auto_height.py','visual_pick_sort_v22_auto_height.py',
                 'visual_batch_workspace_v20.py','visual_auto_recovery_v22.py')})
        client=InferenceClient(args.inference_python,str(ROOT/'src/battery_grasp/scripts/yolo_worker.py'),
            str(ROOT/'deployment/yolov5-runtime'),str(weights),device=args.device,class_confidences=thresholds)
        pipeline=rs.pipeline();config=rs.config();config.enable_device('344322075008')
        config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
        profile=pipeline.start(config);started=True
        intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        k=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]],float)
        if (k.tolist()!=candidate['source_context']['K'] or list(intr.coeffs)!=candidate['source_context']['distortion']
                or str(intr.model)!=candidate['source_context']['distortion_model'] or any(intr.coeffs)):
            raise ValueError('Camera intrinsics changed; no assumed compensation')
        transform=np.asarray(candidate['world_from_camera'],float)
        context=dict(camera_serial='344322075008',image_size=[1280,720],K=k.tolist(),
            distortion=list(intr.coeffs),distortion_model=str(intr.model),candidate_sha256=candidate_sha,
            weights_sha256=sha(weights),shell_model_sha256=sha(ROOT/'deployment/black_cylinder_measured_20261020.json'),
            source_grasp_sha256=grasp['source_worker_sha256'],camera_fixed_confirmed=args.confirm_fixed_camera,
            source_pregrasp_sha256=grasp['pregrasp_source_sha256'],
            source_pregrasp_crosscheck_sha256=grasp['pregrasp_crosscheck_sha256'],
            measured_shell=dict(diameter_mm=model['diameter_mm'],total_height_mm=model['total_height_mm']))
        context['geometry_module_sha256']=sha(ROOT/'deployment/known_cylinder_geometry_axis.py')
        report['binding_camera_context']=context
        if args.automatic and args.define_workspace:raise ValueError('Choose workspace definition or automatic capture')
        if args.automatic:
            if args.workspace_config is None:raise ValueError('Define the image workspace first')
            workspace=json.loads(args.workspace_config.read_bytes())
            workspace_points=polygon(workspace,context)
            report['workspace']=workspace
            if args.completed_ledger is not None:
                ledger=json.loads(args.completed_ledger.read_bytes())['completed']
            report.update(automatic_target_selection=True,object_coordinates_from_ROI=False,
                image_priority='fewest_failed_observations_then_frontmost_image_base_then_leftmost_no_slot_lookup',
                completed_ledger=ledger,scan_frames=scan_rows)
        report['support_inference_method']='every_original_rim_two_measured_support_plane_hypotheses_v22_fix1'
        report['cap_measurement_method']='body_axis_aligned_upper_silhouette_fallback; original_closed_rim_first'
        report['capture_input_policy']='freeze_first_selected_detector_bbox_for_entire_burst; preserve_raw_detector_bbox_and_every_RGB_frame'
        report['per_frame_coordinates_smoothed']=False
        report['original_per_frame_image_geometry_gates_retained']=True
        report['original_free3D_burst_verdict_separately_recorded']=True
        report['new_equal_height_plane_method_not_free3D_acceptance']=True
        report['plane_burst_repeatability_mm']=3.0
        if args.anchor is not None:
            anchor_path=args.anchor.resolve()
            binding=validate_saved_anchor(json.loads(anchor_path.read_bytes()),context,grasp)
            for stem in ('reference_observation','reference_image'):
                name=binding[stem+'_file'];source_path=(anchor_path.parent/name).resolve()
                if (source_path.parent!=anchor_path.parent or source_path.name!=name
                        or sha(source_path)!=binding[stem+'_sha256']):
                    raise ValueError('Saved reference evidence differs: '+name)
                copied_name='geometry_source_'+stem+source_path.suffix
                (folder/copied_name).write_bytes(source_path.read_bytes())
                report.setdefault('historical_plane_geometry_evidence',{})[stem]=dict(
                    file=copied_name,sha256=sha(source_path),original_name=name)
                if stem=='reference_observation':geometry_reference_observation=json.loads(source_path.read_bytes())
            dump(folder/'geometry_source_anchor.json',binding)
            report.update(historical_geometry_source_validated=True,geometry_source_anchor=str(anchor_path),
                geometry_source_anchor_sha256=sha(anchor_path),old_grasp_tcp_mapping_used=False)
        if binding is None:raise ValueError('Reviewed visual plane geometry source required')
        # Reconstruct the unchanged historical recess-reference geometry;
        # the current selected support is applied separately below.
        plane=make_plane(binding,geometry_reference_observation,transform,True)
        plane['method_source_sha256']=sha(ROOT/'deployment/visual_equal_height_plane.py')
        plane['original_task_anchor_sha256']=sha(folder/'geometry_source_anchor.json')
        plane['old_grasp_tcp_mapping_used']=False
        base_plane=plane
        report.update(base_equal_height_plane=base_plane,automatic_support_selection=True,
            operator_height_selection=False,arbitrary_unknown_height_supported=False,
            every_live_camera_ray_reintersected=True)
        dump(folder/'base_equal_height_plane.json',base_plane)
        dump(folder/'equal_height_plane.json',plane)
        report['equal_height_plane']=plane
        report['equal_height_plane_sha256']=sha(folder/'equal_height_plane.json')
        title=('WORKSPACE SETUP | four corners then S' if args.define_workspace else
            'AUTO TARGET | Q stop' if args.automatic else 'SELECT LIVE PICK TARGET | S save Q exit')+' | NO MOTION'
        cv2.namedWindow(title,cv2.WINDOW_NORMAL)
        selection=dict(bbox=None,click=None)
        def mouse(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN and y>=100:
                if args.define_workspace:
                    if len(workspace_clicks)==4:workspace_clicks.clear()
                    workspace_clicks.append([float(x),float(y-100)])
                elif not args.automatic:selection['click']=(x,y-100)
        cv2.setMouseCallback(title,mouse)
        print('首次圈工作区：沿抓取板区域依次点四角（不含投放盒），S保存；R重画，Q退出。' if args.define_workspace
            else '自动识别工作区目标，无需点击或按S；Q停止采图。' if args.automatic
            else '绑定原位置：把同一空壳放回044001成功抓取前的原位，点选后按S。' if args.role=='reference'
            else '当前抓取目标：点选这次要抓的空壳，按S；保存后不要再移动它。',flush=True)
        print('自动选择并采集；未合格时自动重新观测、轮换目标后回查，无需点击或按S。' if args.automatic
            else '点当前空壳，S采样；圆柱方向固定为本板直立方向，再由原始轮廓估测位置。',flush=True)
        print('当前窗口只采图、不连接臂手SDK；采集期间空壳/头部/工作板不能移动。',flush=True)
        print('自动判断本板两层；不人工选高度。原图、原自由三维、新测高结果均保留，Q退出。',flush=True)
        scan_started=time.monotonic()
        while True:
            if args.stop_file is not None and args.stop_file.exists():
                report['auto_scan_outcome']='operator_stopped';break
            observation_budget=30. if args.observe_only else recovery.budget_seconds
            if args.automatic and time.monotonic()-scan_started>observation_budget:
                report['auto_scan_outcome']='automatic_visual_recovery_exhausted'
                report['visual_recovery']=recovery.summary();break
            frames=pipeline.wait_for_frames(5000);color=frames.get_color_frame()
            if not color:continue
            image=np.asanyarray(color.get_data()).copy()
            detections=client.detect(image)
            black=[d for d in detections if d['class_id']==0 and d['confidence']>=thresholds[0]]
            result=dict(delivery_version=report['delivery_version'],
                frame_number=int(color.get_frame_number()),timestamp_ms=float(color.get_timestamp()),
                detections=detections,geometry_candidate_available=False,execution_ready=False,
                selection_scope='same_seated_shell_visual_point_for_new_wrist_reference',
                old_grasp_tcp_mapping_used=False,old_2p8_plus_4p5_offsets_used=False,motion_authorized=False)
            view=image.copy()
            for d in detections:
                x,y,w,h=[int(v) for v in d['bbox']]
                cv2.rectangle(view,(x,y),(x+w,y+h),(140,140,140),1)
                cv2.putText(view,'%s %.2f'%(d['class_name'],d['confidence']),(max(0,x),max(15,y-5)),0,.45,(180,180,180),1)
            marker_polygons=[]
            try:
                dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                if hasattr(cv2.aruco,'ArucoDetector'):
                    corners,ids,_=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters()).detectMarkers(image)
                else:corners,ids,_=cv2.aruco.detectMarkers(image,dictionary)
                marker_polygons=[c.reshape(4,2).astype(np.float32) for c in corners]
                result['visible_aruco_ids']=[] if ids is None else [int(v) for v in ids.flatten()]
            except (cv2.error,AttributeError) as exc:result['aruco_exclusion_error']=str(exc)
            black.sort(key=lambda d:(d['bbox'][0],d['bbox'][1]))
            workspace_black=([d for d in black if contains(foot(d['bbox']),workspace_points)
                and not any(cv2.pointPolygonTest(p,(d['bbox'][0]+d['bbox'][2]/2,d['bbox'][1]+d['bbox'][3]/2),False)>=0
                    for p in marker_polygons)]
                if args.automatic else black)
            completed_visible=([d for d in workspace_black if any(
                same_detection(d['bbox'],old['measurement_bbox_px']) for old in ledger)] if args.automatic else [])
            if args.automatic:
                recovery.observe(workspace_black)
                report['final_workspace_detections']=workspace_black
                if len(workspace_black)>9:
                    report['auto_scan_outcome']='more_than_nine_workspace_black_detections';break
                kind=('processed_target_still_visible' if completed_visible else
                    'workspace_no_black_observed' if not workspace_black else 'workspace_observed')
                if kind!=inventory_kind:
                    inventory_since=time.monotonic();inventory_frames=0;inventory_kind=kind
                inventory_frames+=1
                inventory_ready=inventory_frames>=8 and time.monotonic()-inventory_since>=2.
                result['automatic_inventory_kind']=kind
                result['processed_target_visible']=bool(completed_visible)
                if completed_visible:report['processed_targets_absent_in_fresh_scan']=False
                elif inventory_ready:report['processed_targets_absent_in_fresh_scan']=True
                scan=dict(frame_number=result['frame_number'],timestamp_ms=result['timestamp_ms'],
                    detections=workspace_black,completed_visible=completed_visible,classification=kind)
                if capture is None:
                    filename='scan_%04d.png'%(len(scan_rows)+1)
                    if not cv2.imwrite(str(folder/filename),image):raise RuntimeError('Inventory image save failed')
                    scan.update(RGB_file=filename,RGB_sha256=sha(folder/filename))
                scan_rows.append(scan)
                if inventory_ready and (kind!='workspace_observed' or args.observe_only):
                    report['auto_scan_outcome']=kind;report['final_workspace_detections']=workspace_black
                    report['processed_targets_absent_in_fresh_scan']=not completed_visible
                    break
                if selection['bbox'] is None and workspace_black and not completed_visible and not args.observe_only:
                    selection['bbox']=recovery.choose(time.monotonic())
                    report['visual_recovery']=recovery.summary()
                    if selection['bbox'] is None and recovery.exhausted():
                        report['auto_scan_outcome']='automatic_visual_recovery_exhausted';break
            candidate_rows=[]
            locked_match=None
            if capture is not None:
                matches=[d for d in black if bbox_iou(capture['measurement_bbox'],d['bbox'])>=.5]
                if matches:locked_match=max(matches,key=lambda d:bbox_iou(capture['measurement_bbox'],d['bbox']))
            for number,d in enumerate(black,1):
                x,y,w,h=[float(v) for v in d['bbox']]
                locked=capture is not None and d is locked_match
                measurement_bbox=(capture['measurement_bbox'][:] if locked else d['bbox'][:])
                item=dict(detection_id=number,detection=d,geometry_candidate_available=False,
                    measurement_bbox=measurement_bbox,measurement_bbox_locked=locked,
                    raw_detector_bbox=d['bbox'][:],
                    geometry_input_policy='fixed_burst_bbox' if locked else 'live_detector_bbox')
                if args.automatic and (not contains(foot(d['bbox']),workspace_points)
                        or selection['bbox'] is None or not same_detection(selection['bbox'],d['bbox'])):
                    item['geometry_error']='NOT CURRENT AUTO TARGET; DETECTION RETAINED'
                    candidate_rows.append(item);continue
                center=(x+w/2,y+h/2)
                if any(cv2.pointPolygonTest(p,center,False)>=0 for p in marker_polygons):
                    item['geometry_error']='ARUCO BOARD EXCLUDED'
                    candidate_rows.append(item)
                    continue
                try:
                    geometry=locate(image,measurement_bbox,k,model['diameter_mm'],model['total_height_mm'])
                    item['geometry']=geometry
                    if geometry['geometry_candidate_available']:
                        chosen=geometry['selected_candidate'];world={}
                        for field in ('top_center','geometric_body_center'):
                            p=np.asarray(chosen[field+'_camera_m'])
                            q=transform[:3,:3]@p+transform[:3,3]
                            inside=all(sum(e[j]*p[j] for j in range(3))+e[3]<=.005 for e in candidate['fit_camera_hull_equations'])
                            world[field+'_world_m']=q.tolist();world[field+'_inside_FIT_support']=inside
                            world[field+'_maximum_FIT_face_violation_mm']=float(max(
                                (np.dot(e[:3],p)+e[3])/np.linalg.norm(e[:3])*1000
                                for e in candidate['fit_camera_hull_equations']))
                        world['axis_from_top_into_body_world']=(transform[:3,:3]@np.asarray(chosen['axis_from_top_into_body_camera'])).tolist()
                        item.update(world,geometry_candidate_available=True)
                    else:item['geometry_error']=geometry['reject_reason']
                except (ValueError,RuntimeError,cv2.error,np.linalg.LinAlgError,FloatingPointError,OverflowError) as exc:
                    item['geometry_error']=str(exc)
                candidate_rows.append(item)
            click=selection['click'];selection['click']=None
            if click is not None and capture is None:
                choices=[r for r in candidate_rows if r.get('geometry_error')!='ARUCO BOARD EXCLUDED'
                    and r['detection']['bbox'][0]<=click[0]<=sum(r['detection']['bbox'][::2])
                    and r['detection']['bbox'][1]<=click[1]<=sum(r['detection']['bbox'][1::2])]
                if choices:selection['bbox']=min(choices,key=lambda r:r['detection']['bbox'][2]*r['detection']['bbox'][3])['detection']['bbox'][:]
            selected=None
            if selection['bbox'] is not None:
                choices=[r for r in candidate_rows if r.get('geometry_error')!='ARUCO BOARD EXCLUDED'
                    and bbox_iou(selection['bbox'],r['detection']['bbox'])>=.25]
                if choices:
                    selected=max(choices,key=lambda r:bbox_iou(selection['bbox'],r['detection']['bbox']))
                    selection['bbox']=selected['detection']['bbox'][:]
                else:result['geometry_error']='SELECTED TARGET LOST - CLICK AGAIN OR A FOR AUTO'
            else:
                qualified=[r for r in candidate_rows if r['geometry_candidate_available']]
                if len(qualified)==1:selected=qualified[0]
                elif len(qualified)>1:result['geometry_error']='MULTIPLE GEOMETRY CANDIDATES - CLICK MEASURED SHELL'
                else:
                    choices=[r for r in candidate_rows if r.get('geometry_error')!='ARUCO BOARD EXCLUDED']
                    if len(choices)==1:selected=choices[0]
                    else:result['geometry_error']='NO QUALIFIED GEOMETRY - CLICK MEASURED SHELL; S SAVES ALL REASONS'
            result['candidate_rows']=candidate_rows
            if selected is not None:
                result['selected_detection_id']=selected['detection_id']
                for name,value in selected.items():
                    if name not in ('detection','detection_id'):result[name]=value
            result['selection_mode']='operator_image_selection' if selection['bbox'] is not None else 'automatic_unique_geometry'
            if args.automatic:result['selection_mode']='automatic_quality_rotation_then_front_first'
            if capture is not None and not result.get('measurement_bbox_locked',False):
                result['geometry_candidate_available']=False
                result['geometry_error']='FIXED ROI TARGET LOST OR IDENTITY CHANGED; original0.5 overlap required'
            if capture is not None:
                result['burst_locked_measurement_bbox']=capture['measurement_bbox'][:]
            if result['geometry_candidate_available']:
                try:
                    result['original_free3D_support_height_mm']=observed_rise(result,base_plane)
                    result['upright_circle_height_measurement']=upright_measurement(result,base_plane)
                    result['auto_support_height_observation_mm']=result['upright_circle_height_measurement']['observed_rise_mm']
                except (ValueError,TypeError,KeyError,np.linalg.LinAlgError,FloatingPointError,OverflowError) as exc:
                    result['auto_support_height_error']=str(exc)
            if args.automatic and scan_rows:
                scan_rows[-1].update(selected_detection_id=result.get('selected_detection_id'),
                    geometry_candidate_available=result.get('geometry_candidate_available'),
                    geometry_error=result.get('geometry_error'),
                    auto_support_height_error=result.get('auto_support_height_error'),
                    height_observation_mm=result.get('auto_support_height_observation_mm'))
                report['last_automatic_geometry_reason']=(result.get('auto_support_height_error')
                    or result.get('geometry_error'))
            completed=None;raw_completed=None
            if capture is not None:
                image_name='burst_%03d_frame_%03d.png'%(saved+1,len(capture['frames'])+1)
                image_path=folder/image_name
                if not cv2.imwrite(str(image_path),image,[cv2.IMWRITE_PNG_COMPRESSION,1]):
                    raise RuntimeError('Per-frame RGB evidence save failed')
                result['burst_RGB_file']=image_name
                result['burst_RGB_sha256']=sha(image_path)
                raw_completed=burst.observe(capture,result,time.monotonic())
                if raw_completed is not None:
                    completed,selected_plane,support_rise,support_sha,decision,auto_evidence,method_frames=automatic_projection(
                        capture['frames'],base_plane,raw_completed['elapsed_seconds'])
                    if args.automatic and ledger and any(f.get('processed_target_visible') for f in capture['frames']):
                        completed.update(frame_quality_pass=False,burst_qualified=False)
                        completed['reject_reasons'].append('previous_processed_target_visible_in_current_burst')
                    if args.role=='reference' and decision.get('selected_surface')!='recess':
                        completed.update(frame_quality_pass=False,burst_qualified=False)
                        completed['reject_reasons'].append('reference_requires_original_successful_recess_place')
                    report['last_auto_support_decision']=decision
                    report['last_original_free3D_burst_summary']=raw_completed
                    report.setdefault('original_free3D_burst_results',[]).append(raw_completed)
                    saved+=1;stem='observation_%03d'%saved
                    payload=dict(result,burst_summary=completed,original_free3D_burst_summary=raw_completed,burst_frames=capture['frames'],
                        locked_measurement_bbox=capture['measurement_bbox'][:],
                        detector_bbox_is_not_3D_motion=True,per_frame_coordinates_smoothed=False,
                        auto_support_decision=decision,auto_support_height_evidence=auto_evidence,
                        auto_support_method_frames=method_frames)
                    last_capture_pass=completed['burst_qualified']
                    if last_capture_pass:
                        last_capture_message='AUTO '+decision['selected_surface'].upper()+' +%.1fmm; raw height %.2fmm'%(support_rise,decision['observed_rise_median_mm'])
                        report.update(automatically_selected_support=decision['selected_surface'],
                            equal_height_plane=selected_plane,measured_support_rise_mm=support_rise,
                            support_height_model_sha256=support_sha)
                        dump(folder/'equal_height_plane.json',selected_plane)
                    else:
                        top=completed.get('top_center_repeatability_max_mm')
                        body=completed.get('geometric_body_center_repeatability_max_mm')
                        if decision.get('qualified') is not True:
                            last_capture_message='AUTO HEIGHT UNKNOWN: '+','.join(decision['reject_reasons'])
                        elif top is not None and body is not None and (top>3 or body>3):
                            last_capture_message='PLANE REJECTED: jitter top %.3f / body %.3f mm; limit3mm'%(top,body)
                        else:last_capture_message='PLANE REJECTED: '+','.join(completed['reject_reasons'])
                    report['last_capture_message']=last_capture_message
                    report['last_capture_pass']=last_capture_pass
                    if completed['burst_qualified']:
                        points.append(dict(observation=stem,**completed))
                        report['qualified_point_captures']=points
                        target_count+=1
                        reference=dict(schema_version=1,scope='new_wrist_reference_visual_top_point_only',
                            top_center_world_m=completed['top_center_world_m'][:],
                            burst_summary=completed,original_free3D_burst_summary=raw_completed,
                            equal_height_plane=selected_plane,binding_camera_context=context,observation=stem,
                            support_surface=decision['selected_surface'],measured_support_rise_mm=support_rise,
                            support_height_model_sha256=support_sha,height_detected_automatically=True,
                            auto_support_decision=decision,auto_support_height_evidence=auto_evidence,
                            operator_height_selection=False,arbitrary_unknown_height_supported=False,
                            observation_file=stem+'.json',reference_index=target_count,
                            historical_geometry_anchor_sha256=sha(folder/'geometry_source_anchor.json'),
                            old_grasp_tcp_mapping_used=False,old_2p8_plus_4p5_offsets_used=False,
                            wrist_pose_not_inferred_from_image=True,motion_authorized=False,execution_ready=False)
                        if args.automatic:
                            reference.update(measurement_bbox_px=capture['measurement_bbox'][:],
                                automatic_workspace=report['workspace'],automatic_target_selection=True)
                        dump(folder/('reference_point_%03d.json'%target_count),reference)
                        dump(folder/'latest_reference_point.json',reference)
                        payload.update(binding_role='new_wrist_reference_visual_point',reference_point=reference)
                        report['latest_reference_point']=reference
                    dump(folder/(stem+'.json'),payload)
                    if not cv2.imwrite(str(folder/(stem+'.png')),image):raise RuntimeError('Raw image evidence save failed')
                    dump(folder/'capture_state.json',dict(report,saved_observations=saved,reference_points=target_count))
                    print('SAVED',stem,'PLANE_BURST_QUALIFIED:',completed['burst_qualified'],'ORIGINAL_FREE3D_BURST:',raw_completed['burst_qualified'],
                        'REASONS:',completed['reject_reasons'],flush=True)
                    if decision.get('qualified') is True:
                        print('自动识别：'+('板顶' if decision['selected_surface']=='board_top' else '槽内')+
                            '；原连续估高%.2fmm、波动%.2fmm仅作诊断，最终XYZ已通过3mm检查。'%(decision['observed_rise_median_mm'],decision['height_repeatability_max_mm']),flush=True)
                    elif not args.automatic:
                        print('本轮测高尚未确定。新方法各帧(mm)：',decision.get('observed_rise_by_frame_mm'),
                            '；旧自由测高(mm)：',decision.get('original_height_decision',{}).get('original_unconstrained_height_decision',{}).get('observed_rise_by_frame_mm'),flush=True)
                        print('轮廓拟合错误：',decision.get('hypothesis_errors',[]),flush=True)
                    if payload.get('reference_point') is not None:
                        print(('REFERENCE SAVED' if args.role=='reference' else 'LIVE TARGET SAVED')+'; shell top World(mm):',
                            [v*1000 for v in completed['top_center_world_m']],
                            'OLD TCP MAPPING USED: False; NO MOTION',flush=True)
                    capture=None
                    if args.automatic:
                        if completed['burst_qualified']:
                            report['auto_scan_outcome']='target_qualified'
                        else:
                            recovery.failed(completed['reject_reasons'],time.monotonic(),
                                observation=stem,image=stem+'.png')
                            report['visual_recovery']=recovery.summary()
                            selection['bbox']=None
                            auto_stable_since=None;auto_stable_frames=0;auto_previous_bbox=None
                            last_capture_message='AUTO REOBSERVING: '+str(last_capture_message)
                            last_capture_pass=None
                            print('本组未合格，自动重新观测/轮换目标；原图保留，不增加完成数。',flush=True)
            for item in candidate_rows:
                x,y,w,h=[int(v) for v in item['detection']['bbox']]
                active=selected is item
                color_box=(0,255,0) if active else (0,180,255)
                cv2.rectangle(view,(x,y),(x+w,y+h),color_box,2)
                label='#%d %s'%(item['detection_id'],'SELECTED' if active else (item.get('geometry_error') or 'GEOMETRY'))
                cv2.putText(view,label[:75],(max(0,x),max(15,y-20)),0,.45,color_box,1)
                if item.get('measurement_bbox_locked'):
                    mx,my,mw,mh=[int(v) for v in item['measurement_bbox']]
                    cv2.rectangle(view,(mx,my),(mx+mw,my+mh),(255,180,0),1)
                if item.get('geometry'):
                    ellipse=item['geometry']['ellipse']
                    cv2.ellipse(view,(tuple(ellipse['center_px']),tuple(ellipse['diameters_px']),ellipse['angle_deg']),(255,0,255),2)
            message=result.get('auto_support_height_error') or result.get('geometry_error') or 'PLANE CANDIDATE - FIELD ACCURACY UNACCEPTED'
            if result['geometry_candidate_available'] and not result.get('auto_support_height_error'):
                message='Raw free3D candidate; PLANE POINT REQUIRED'
            if result.get('auto_support_height_observation_mm') is not None:
                message=('VISUAL HEIGHT %.2fmm; automatic observation' if args.automatic else
                    'VISUAL HEIGHT %.2fmm; S collects all8 frames for automatic support')%result['auto_support_height_observation_mm']
            message_color=(0,255,0) if result['geometry_candidate_available'] else (0,140,255)
            if last_capture_message is not None and capture is None:
                message=last_capture_message
                message_color=(0,255,0) if last_capture_pass else ((0,180,255)
                    if args.automatic and last_capture_pass is None else (0,80,255))
            view=cv2.copyMakeBorder(view,100,0,0,0,cv2.BORDER_CONSTANT,value=(0,0,0))
            drawn=workspace_clicks if args.define_workspace else workspace_points
            if drawn:
                contour=np.asarray([[x,y+100] for x,y in drawn],np.int32)
                cv2.polylines(view,[contour],len(drawn)==4,(255,200,0),2)
            phase=('BIND ORIGINAL PLACE' if args.role=='reference' else 'LIVE PICK TARGET')+' | S SAVE | saved: %d'%target_count
            if capture is not None:phase='CAPTURING FIXED ROI - KEEP SHELL STILL (%d frames)'%len(capture['frames'])
            if args.define_workspace:
                phase='WORKSPACE: FOUR CORNERS (%d/4) | S SAVE | R REDRAW'%len(workspace_clicks)
                message='Draw the pick area only; exclude bins. Q exits. NO ROBOT MOTION'
            elif args.automatic and capture is None:phase='AUTO OBSERVING WORKSPACE | Q STOP'
            cv2.putText(view,phase+' | NO ROBOT COMMANDS',(12,28),0,.60,(0,255,255),2)
            cv2.putText(view,message[:120],(12,60),0,.55,message_color,1)
            outline=result.get('geometry',{}).get('contour_detail',{})
            mode='UPRIGHT CIRCLE HEIGHT | fixed common axis | all raw rim pixels'
            if result['geometry_candidate_available'] and not result.get('top_center_inside_FIT_support',False):
                mode+=' | OUTSIDE FIT'
            cv2.putText(view,mode+' | selected support; normal estimated; NO MOTION',(12,85),0,.5,(180,180,255),1)
            if completed is not None and not cv2.imwrite(str(folder/(stem+'_view.png')),view):
                raise RuntimeError('Display evidence save failed')
            cv2.imshow(title,view);key=cv2.waitKey(1)&255
            if args.define_workspace:
                if key in (ord('r'),ord('R')):workspace_clicks.clear()
                if key in (27,ord('q'),ord('Q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1:
                    report['auto_scan_outcome']='workspace_definition_cancelled';break
                if key in (ord('s'),ord('S')):
                    workspace=dict(schema_version=1,camera_context=context,polygon_px=workspace_clicks[:],
                        purpose='image_selection_region_only_not_grasp_coordinates',slot_lookup=False)
                    try:polygon(workspace,context)
                    except ValueError as exc:print(str(exc),flush=True);continue
                    if args.workspace_config is None:raise ValueError('Workspace save path missing')
                    args.workspace_config.parent.mkdir(parents=True,exist_ok=True)
                    dump(args.workspace_config,workspace);dump(folder/'workspace.json',workspace)
                    if not cv2.imwrite(str(folder/'workspace_view.png'),view):raise RuntimeError('Workspace view save failed')
                    report['auto_scan_outcome']='workspace_defined';break
                continue
            if key in (27,ord('q'),ord('Q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1:
                if args.automatic:report['auto_scan_outcome']='operator_stopped'
                break
            if ((args.finish_after_first_reference or args.automatic) and completed is not None
                    and completed['burst_qualified']):break
            if args.automatic and completed is not None:
                # A rejected perception burst has made no actuator request.
                # Keep this camera process and automatically select a fresh
                # group/another target; never reuse its rejected coordinates.
                continue
            if args.automatic and capture is None and recovery.preview_expired(time.monotonic()):
                recovery.failed([result.get('auto_support_height_error') or result.get('geometry_error')
                    or 'selected_target_not_stably_measurable'],time.monotonic(),
                    image=scan_rows[-1].get('RGB_file') if scan_rows else None)
                report['visual_recovery']=recovery.summary()
                selection['bbox']=None
                auto_stable_since=None;auto_stable_frames=0;auto_previous_bbox=None
                print('当前目标暂未能稳定测量，自动轮换并回查；未发运动。',flush=True)
                continue
            if args.automatic and capture is None and not args.observe_only:
                if (result['geometry_candidate_available'] and not result.get('auto_support_height_error')
                        and not completed_visible):
                    box=result.get('measurement_bbox')
                    if auto_previous_bbox is not None and same_detection(box,auto_previous_bbox):auto_stable_frames+=1
                    else:auto_stable_frames=1;auto_stable_since=time.monotonic()
                    auto_previous_bbox=box[:]
                    if (auto_stable_frames>=3 and time.monotonic()-auto_stable_since>=1.
                            and (not ledger or inventory_ready)):
                        key=ord('s')
                else:auto_stable_since=None;auto_stable_frames=0;auto_previous_bbox=None
            if not args.automatic and capture is None and key in (ord('a'),ord('A')):selection['bbox']=None
            if not args.automatic and capture is None and ord('1')<=key<=ord('9'):
                chosen=[r for r in candidate_rows if r['detection_id']==key-ord('0') and r.get('geometry_error')!='ARUCO BOARD EXCLUDED']
                if chosen:selection['bbox']=chosen[0]['detection']['bbox'][:]
            if key in (ord('s'),ord('S')):
                if capture is not None:print('正在采集，请保持空壳不动；Q可保存中断记录并退出。',flush=True)
                elif not result['geometry_candidate_available'] or result.get('auto_support_height_error') is not None:
                    saved+=1;stem='observation_%03d'%saved
                    dump(folder/(stem+'.json'),result)
                    if not cv2.imwrite(str(folder/(stem+'.png')),image) or not cv2.imwrite(str(folder/(stem+'_view.png')),view):
                        raise RuntimeError('Rejected image evidence save failed')
                    last_capture_pass=False
                    last_capture_message='PLANE REJECTED: '+str(result.get('auto_support_height_error') or result.get('geometry_error'))
                    report['last_capture_message']=last_capture_message;report['last_capture_pass']=False
                    print('SAVED REJECTED',stem,result.get('auto_support_height_error') or result.get('geometry_error'),flush=True)
                else:
                    selection['bbox']=next(r['detection']['bbox'][:] for r in candidate_rows if r['detection_id']==result['selected_detection_id'])
                    capture=burst.begin(time.monotonic())
                    capture['measurement_bbox']=selection['bbox'][:]
                    capture['measurement_bbox_origin']='selected detector bbox at this S; immutable until burst completes'
                    last_capture_message=None;last_capture_pass=None
                    report['last_locked_measurement_bbox']=capture['measurement_bbox'][:]
                    print('CAPTURING：保持空壳静止，至少8帧/约2秒；采样窗口上限15秒，不含单帧推理/取帧阻塞。',flush=True)
        report['outcome']=report.get('auto_scan_outcome') or ('new_wrist_reference_point_saved' if target_count else
            'no_qualified_new_wrist_reference_point')
        report['reference_points']=target_count
        report['six_dimensional_TCP_calibrated']=False
        report['motion_authorized']=False
        return 0
    except BaseException as exc:
        report.update(error=repr(exc),traceback=traceback.format_exc(),outcome='failed_no_robot_access')
        print('ERROR:',exc,flush=True)
        return 1
    finally:
        cleanup_errors=[]
        try:
            if started:pipeline.stop()
        except Exception as exc:cleanup_errors.append('pipeline.stop: '+repr(exc))
        try:
            if client is not None:client.close()
        except Exception as exc:cleanup_errors.append('client.close: '+repr(exc))
        report['saved_observations']=saved
        if capture is not None:
            try:
                dump(folder/'interrupted_burst.json',dict(capture,interrupted=True,execution_ready=False))
            except Exception as exc:cleanup_errors.append('interrupted record: '+repr(exc))
        report['cleanup_errors']=cleanup_errors
        try:
            dump(folder/'result.json',report)
        except Exception as exc:
            cleanup_errors.append('result record: '+repr(exc))
            try:
                dump(folder/'result_fallback.json',dict(outcome='record_write_failed',
                    original_error=str(report.get('error')),cleanup_errors=cleanup_errors,execution_ready=False))
            except Exception as nested:print('RECORD FAILED:',repr(nested),flush=True)
        # Packaging is attempted even when another record or cleanup operation failed.
        try:
            archive=archive_folder(folder)
            print('SAVED:',archive,'EXECUTION_READY: False',flush=True)
            if args.scan_response is not None:
                dump(args.scan_response,dict(outcome=report.get('outcome'),archive=str(archive),
                    error=report.get('error'),cleanup_errors=cleanup_errors,
                    processed_targets_absent=report.get('processed_targets_absent_in_fresh_scan'),
                    workspace_black_count=len(report.get('final_workspace_detections',[])),
                    visual_recovery=report.get('visual_recovery')))
        except Exception as exc:print('ARCHIVE FAILED:',repr(exc),'FILES RETAINED:',folder,flush=True)


if __name__=='__main__':sys.exit(main())

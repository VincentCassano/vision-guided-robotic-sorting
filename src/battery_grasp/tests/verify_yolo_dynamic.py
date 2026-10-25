#!/usr/bin/env python3
import argparse
import copy
import json
import sys
import time
from pathlib import Path
from verify_stage4 import CFG, PACKAGE
from battery_grasp.photo_scene import PhotoScene
from battery_grasp.inference_client import InferenceClient
from battery_grasp.yolo_inference import detection_message
from battery_grasp.rgbd import project_frame
from battery_grasp.observation_tracking import ObservationTracker
from battery_grasp.observation_adapter import ObservationAdapter
from battery_grasp.task_control import TaskController


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repository',required=True)
    parser.add_argument('--output',required=True)
    args = parser.parse_args()
    cfg = copy.deepcopy(CFG)
    cfg['max_age'] = 5.
    camera = cfg['camera']
    scene = PhotoScene(PACKAGE/'test_data/battery_v1')
    tracker = ObservationTracker(camera['rotation'],camera['translation'],camera['frame_id'],max_gap=3.)
    adapter = ObservationAdapter(cfg)
    control = TaskController(cfg,0.)
    control.confirm_scene_reset(control.planner.session,0.)
    client = InferenceClient(sys.executable,str(PACKAGE/'scripts/yolo_worker.py'),args.repository,
                             str(PACKAGE/'models/battery_yolov5s_v1.pt'),square_confidence=.60)
    removed,results,durations = set(),[],[]
    now = 0.
    fx,fy,cx,cy = camera['intrinsics']
    def frame():
        nonlocal now
        now += .5
        stamp = int(now*1e9)
        rgb,depth = scene.render(cfg,removed)
        start = time.monotonic()
        detections = client.detect(rgb)
        durations.append(time.monotonic()-start)
        detection = detection_message(detections,stamp,camera['frame_id'],640,480)
        optical = project_frame(detection,dict(stamp_ns=stamp,frame_id=camera['frame_id'],image=depth,encoding='32FC1'),
                                dict(stamp_ns=stamp,frame_id=camera['frame_id'],width=640,height=480,
                                     K=[fx,0,cx,0,fy,cy,0,0,1]))
        observation = tracker.process(optical,stamp)
        adapted = adapter.process(observation,stamp)
        control.observe(adapted)
        return adapted
    try:
        assert not frame()['valid']
        assert frame()['valid']
        assert len(control.latest['objects'])==9
        assert all(value is None for value in control.tick(now).values())
        control.command('MODE_MANUAL',now)
        for _ in range(2):
            assert control.command('START',now)[0]['accepted']
            for step in range(100):
                frame()
                output = control.tick(now)
                assert control.state!='ERROR',control.reason
                if output['result']:
                    assert output['result']['status']=='DONE'
                    results.append(output['result'])
                    removed.add(output['result']['target_id'])
                if control.state=='IDLE': break
            else: raise AssertionError('Manual timeout')
            for _ in range(2):
                assert frame()['valid']
                assert all(value is None for value in control.tick(now).values())
        assert len(results)==2
        control.command('MODE_AUTO',now)
        assert control.command('START',now)[0]['accepted']
        for _ in range(150):
            frame()
            output = control.tick(now)
            assert control.state!='ERROR',control.reason
            if output['result']:
                assert output['result']['status']=='DONE'
                results.append(output['result'])
                removed.add(output['result']['target_id'])
            if control.state=='DONE': break
        assert control.state=='DONE',control.reason
        assert len(results)==len(removed)==9
        assert control.latest['objects']==[]
        assert sum(r['bin']=='LEFT_UPPER_BIN' for r in results)==7
        assert [r['target_id'] for r in results]==['r{}c{}'.format(r,c) for c in range(3) for r in range(3)]
        report = dict(passed=True,ros_tested=False,simulation_only=True,actual_model=True,
                      scene='Photo cutouts from existing held-out fixture; not independent evaluation data',
                      manual_completed=2,auto_completed=7,final_state=control.state,
                      inference_frames=len(durations),max_inference_seconds=max(durations),results=results)
        Path(args.output).write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))
    finally:
        client.close()


if __name__=='__main__':
    main()

"""Selectable USB/webcam preview using the existing battery inference module."""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
(ROOT/'deployment/runtime-config').mkdir(parents=True, exist_ok=True)
os.environ.setdefault('YOLO_CONFIG_DIR', str(ROOT/'deployment/runtime-config'))
os.environ.setdefault('YOLOV5_CONFIG_DIR', str(ROOT/'deployment/runtime-config'))
sys.path.insert(0, str(ROOT / 'src/battery_grasp/src'))
import cv2
from battery_grasp.yolo_inference import BatteryDetector

COLORS = [(60, 220, 60), (60, 60, 255), (255, 220, 0), (0, 180, 255)]


def backend_id(name):
    if name == 'auto':
        return cv2.CAP_DSHOW if os.name == 'nt' else cv2.CAP_ANY
    return {'dshow': cv2.CAP_DSHOW, 'msmf': cv2.CAP_MSMF, 'v4l2': cv2.CAP_V4L2}[name]


def scan_cameras(limit, backend):
    available = []
    for index in range(limit):
        cap = cv2.VideoCapture(index, backend)
        try:
            ok, frame = cap.read() if cap.isOpened() else (False, None)
            if ok and frame is not None:
                available.append(index)
                print(f'  摄像头 {index}: {frame.shape[1]} x {frame.shape[0]}', flush=True)
        finally:
            cap.release()
    return available


def draw_frame(frame, detections, fps, camera, calibrated):
    scale = min(1.0, 1280 / frame.shape[1], 800 / frame.shape[0])
    view = cv2.resize(frame, (round(frame.shape[1]*scale), round(frame.shape[0]*scale)))
    for d in detections:
        x, y, w, h = d['bbox']
        x1, y1, x2, y2 = [round(v*scale) for v in (x, y, x+w, y+h)]
        color = COLORS[d['class_id']]
        label = f"{d['class_name']} {d['confidence']:.2f}"
        if 'visual_confirmed' in d:
            label += ' OK' if d['visual_confirmed'] else f" WAIT {d['stable_seconds']:.1f}s"
            if not d['visual_confirmed']: color=(150,150,150)
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)
        tx = max(0, min(x1, view.shape[1]-tw-4))
        ty = max(48, y1-6)
        cv2.rectangle(view, (tx, ty-th-4), (tx+tw+4, ty+baseline), (25, 25, 25), -1)
        cv2.putText(view, label, (tx+2, ty), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
    cv2.rectangle(view, (0, 0), (view.shape[1], 30), (25, 25, 25), -1)
    text = f'Camera {camera} | {fps:.1f} FPS | {len(detections)} objects | {"Calibrated" if calibrated else "Conf 0.25"} | T: threshold  S: save  Q/Esc: exit'
    cv2.putText(view, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1)
    return view


def main():
    parser = argparse.ArgumentParser(description='电池模型摄像头预览，未指定摄像头时扫描并选择')
    parser.add_argument('--camera', type=int, help='摄像头编号，例如 0 或 1；跳过扫描')
    parser.add_argument('--list-cameras', action='store_true', help='只列出可读取的摄像头')
    parser.add_argument('--scan-limit', type=int, default=6, help='扫描编号 0 至 N-1，默认 6')
    parser.add_argument('--backend', choices=['auto', 'dshow', 'msmf', 'v4l2'], default='auto')
    bundled_repository = ROOT/'deployment/yolov5-runtime'
    parser.add_argument('--repository', type=Path, default=bundled_repository if bundled_repository.is_dir() else ROOT.parent/'AIyolov5/yolov5')
    parser.add_argument('--self-test', action='store_true', help='使用包内测试照片检查依赖、模型和绘框，不打开摄像头')
    parser.add_argument('--weights', type=Path, default=ROOT/'src/battery_grasp/models/battery_yolov5s_v3_last.pt')
    parser.add_argument('--device', default='auto', help='auto、cpu 或 GPU 编号 0')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--raw-threshold', action='store_true', help='启动时四类均使用 0.25')
    args = parser.parse_args()
    if args.scan_limit < 1 or args.width < 1 or args.height < 1 or (args.camera is not None and args.camera < 0):
        parser.error('摄像头编号必须非负，扫描数量和分辨率必须为正数')
    (ROOT/'docs').mkdir(exist_ok=True)
    with (ROOT/'docs/执行命令日志.txt').open('a', encoding='utf-8') as log:
        log.write(subprocess.list2cmdline([sys.executable, *sys.argv])+'\n')
    if args.self_test:
        import torch
        import yaml
        import re
        gui = re.search(r'^\s*GUI:\s*(.*)$', cv2.getBuildInformation(), re.M)
        if not gui or gui.group(1).strip().upper() in ('NONE', 'NO'):
            raise RuntimeError('当前OpenCV不支持窗口，请使用opencv-python而非headless版本。')
        values = yaml.safe_load((ROOT/'src/battery_grasp/config/yolo_v3_calibrated.yaml').read_text())['battery_detector_yolo']['class_confidences']
        detector = BatteryDetector(args.repository, args.weights, device='cpu' if args.device == 'auto' else args.device, class_confidences=values)
        sample = ROOT/'src/battery_grasp/test_data/battery_v1/images/IMG_7862.JPG'
        frame = cv2.imread(str(sample))
        if frame is None:
            raise RuntimeError(f'缺少包内测试照片：{sample}')
        detections = detector.detect(frame)
        labels = sample.parent.parent/'labels'/(sample.stem+'.txt')
        truth = [line.split() for line in labels.read_text().splitlines() if line.strip()]
        if sorted(d['class_id'] for d in detections) != sorted(int(g[0]) for g in truth):
            raise RuntimeError('自检照片检测类别或数量与预期不符，请查看环境和模型版本。')
        output = ROOT/'docs/camera_self_test.png'
        if not cv2.imwrite(str(output), draw_frame(frame, detections, 0, 'test', True)):
            raise RuntimeError('无法写入自检截图')
        print(f'PASS: Python {sys.version.split()[0]}, torch {torch.__version__}, OpenCV {cv2.__version__}, GUI {gui.group(1).strip()}, objects {len(detections)}\n截图：{output}\n此自检未打开摄像头或窗口。')
        return
    backend = backend_id(args.backend)
    camera = args.camera
    if camera is None or args.list_cameras:
        print('正在扫描摄像头；其他程序占用的设备可能无法列出。', flush=True)
        available = scan_cameras(args.scan_limit, backend)
        if not available:
            raise RuntimeError('没有可读取的摄像头。检查连接/相机权限，关闭占用程序；Windows 可重试 --backend msmf。')
        if args.list_cameras:
            return
        while True:
            choice = input(f'选择摄像头编号 {available}（回车使用 {available[0]}）：').strip()
            if not choice:
                camera = available[0]
                break
            if choice.isdigit() and int(choice) in available:
                camera = int(choice)
                break
            print('请输入上面列出的编号。')
    import torch
    import yaml
    profile = yaml.safe_load((ROOT/'src/battery_grasp/config/yolo_v3_calibrated.yaml').read_text())
    thresholds = profile['battery_detector_yolo']['class_confidences']
    calibrated = not args.raw_threshold
    device = ('0' if torch.cuda.is_available() else 'cpu') if args.device == 'auto' else args.device
    print(f'加载模型：{args.weights}\n设备：{device}\n阈值（黑/红/白/方形）：{thresholds}\nT 切换阈值，S 保存当前帧，Q 或 Esc 退出。', flush=True)
    detector = BatteryDetector(args.repository, args.weights, device=device,
                               class_confidences=thresholds if calibrated else [.25]*4)
    cap = cv2.VideoCapture(camera, backend)
    title = 'Battery Camera Preview'
    try:
        if not cap.isOpened():
            raise RuntimeError(f'摄像头 {camera} 打开失败，可能被其他程序占用。')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        fps = 0.0
        while True:
            start = time.perf_counter()
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError('摄像头读取失败或已断开，请重新连接后启动。')
            detections = detector.detect(frame)
            measured = 1/max(time.perf_counter()-start, 1e-6)
            fps = measured if fps == 0 else .8*fps+.2*measured
            view = draw_frame(frame, detections, fps, camera, calibrated)
            cv2.imshow(title, view)
            key = cv2.waitKey(1) & 255
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord('t'), ord('T')):
                calibrated = not calibrated
                detector.class_confidences = thresholds if calibrated else [.25]*4
            if key in (ord('s'), ord('S')):
                target = ROOT/'captures'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                target.mkdir(parents=True)
                if not cv2.imwrite(str(target/'original.png'), frame) or not cv2.imwrite(str(target/'detections.png'), view):
                    raise RuntimeError(f'图片保存失败：{target}')
                (target/'detections.json').write_text(json.dumps(dict(camera=camera,weights=str(args.weights),
                    class_confidences=detector.class_confidences,detections=detections),indent=2),encoding='utf-8')
                print(f'已保存：{target}', flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'错误：{exc}', file=sys.stderr)
        sys.exit(1)

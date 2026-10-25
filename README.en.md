# Vision-Guided Robotic Grasping and Sorting System

[简体中文](README.md) | **English**

[HTML guide (download and open in a browser)](docs/guide.en.html) · [中文HTML说明](docs/guide.zh-CN.html)

## Project overview

This project connects visual detection, spatial localization, robotic grasping, and destination-based sorting into a continuous workflow. A head-mounted Intel RealSense D435 supplies live images. A YOLOv5 model identifies battery shapes and colors, the localization pipeline estimates the current target position, and a robotic arm with a dexterous hand approaches, grasps, lifts, deposits, and returns to its observation pose.

Target coordinates come from current visual observations. Near and far wrist profiles handle different parts of the workspace. The present task uses upright cylindrical test objects of the same dimensions: black objects go to the normal-product bin, while red and white objects share the reject bin. Square objects are detected but are not grasped.

The web dashboard combines continuous camera video, automatic and single-object operation, software stop/reset, stage feedback, and logs. Localization consumes original images rather than the browser video stream. A target must remain visible for at least **two seconds** before entering detection statistics; brief detections and repeated page updates do not increase the count.

- **Perception:** D435 RGB acquisition, three-color detection, and localization on two measured support levels.
- **Execution:** near/far wrist selection, preparation after descent, grasping, lifting, sorting, and return.
- **Interaction:** a dark web dashboard, LAN access, live task status, and an optional ROS 1 bridge.
- **Traceability:** original images, communication records, motion results, and file hashes. Motion-cycle counts remain separate from physical sorting outcomes.

The underlying pipeline has field records for a four-object black-shell batch and some three-color operations. **The current web integration is awaiting field acceptance.** A completed motion cycle does not prove retention, delivery into the correct bin, or an end-to-end success rate.

## Project structure

```text
vision-guided-robotic-sorting/
├── README.md / README.en.md          Chinese and English project guides
├── requirements-web.txt              Web, camera, and inference dependencies
├── deployment/
│   ├── run_visual_task_control_v24.py      Web and continuous-camera entry point
│   ├── visual_task_control_v24.py          HTTP commands, state, stop, and logs
│   ├── visual_task_dashboard_v24.html      Web dashboard
│   ├── visual_live_camera_v24.py           Single D435 owner; paired frames/results
│   ├── visual_detection_confirm_v24.py     Two-second target-presence confirmation
│   ├── visual_pick_sort_live_v24_vision.py  Target selection, support level, position
│   ├── run_visual_pick_sort_live_v24.py     Automatic/single-object batch scheduling
│   ├── visual_pick_sort_v23_worker.py      Original arm/hand motion subprocess
│   ├── task_control_ros_v24.py             Optional ROS task bridge
│   ├── CURRENT_RELEASE.json               Version, entry points, acceptance status
│   └── *-files.sha256                     Motion and web runtime file manifests
├── src/battery_grasp/
│   ├── scripts/yolo_worker.py         Separate YOLO/TorchScript inference process
│   ├── src/battery_grasp/             Inference, planning, and shared ROS modules
│   ├── config/                       Class thresholds and task configuration
│   ├── launch/                       ROS node launch files
│   └── models/                       Model location; only notes are published
├── docs/
│   ├── ASSETS.md                     Model, SDK, calibration, and binding setup
│   └── guide.*.html                  Bilingual HTML guides and selection-only motto
└── tools/release/
    ├── build_runtime_archive.py       Package manifest-listed runtime files
    ├── prepare_github.py              Export the curated public source tree
    └── render_project_guides.py       Generate both HTML guides
```

Control flow: **dashboard command → HTTP task control → batch scheduler → visual localization and the original SDK motion subprocess**. The camera service continues supplying live video during motion.

Device calibration/bindings belong in `calibration/`, original sessions in `captures/`, and local publication checkouts in `publish/`; these directories are excluded from the public source. Some earlier-named scripts and JSON files in `deployment/` remain part of the current verification/provenance chain. Use `CURRENT_RELEASE.json` to identify the active entry point.

## Environment configuration

### Hardware and operating scope

| Component | Current configuration |
|---|---|
| Robot | RealMan RM65BI, left-arm execution; controller V1.6.5, SDK API 4.3.1 |
| Dexterous hand | OYMotion ROH-AP001 over RS485 |
| Camera | Intel RealSense D435, RGB 1280 × 720 at 30 fps |
| Test objects | Unpowered cylinders/shells, 45.8 mm diameter and 98 mm height |
| Support levels | Measured recess bottom and acrylic-board top, separated by 9.6 mm |
| Installation | Device-specific calibration and near/far bindings; fixed head, board, and mounting relationship during operation |

### Software and device assets

The runtime uses Linux, Python 3.8 or later, NumPy, OpenCV with ArUco, PyTorch, PyYAML, and `pyrealsense2`. The web service uses the Python standard library; Node.js, a frontend build, and a cloud service are unnecessary. ROS 1 is optional and the web interface does not require a ROS Master.

Reuse the existing robot environment where possible. For a new environment, the [dependency list](requirements-web.txt) provides a starting point; select PyTorch/CUDA and RealSense drivers for the actual host. It is not a compatibility certification for every version combination.

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-web.txt
```

The GitHub source does not contain model weights, vendor SDK binaries, the on-site `calibration/` directory, or captured datasets. Before running the real robot, restore the device-specific assets listed in [Asset configuration](docs/ASSETS.md). Calibration and taught coordinates from another installation are not substitutes.

## Usage

### 1. Prepare the workspace

For a new source checkout:

```bash
git clone https://github.com/VincentCassano/vision-guided-robotic-sorting.git
cd vision-guided-robotic-sorting
```

An existing installation can keep using `~/test/battery-d435-preview`. Restore the required assets and exit other grasping or camera-preview programs to avoid competing access. SHA256 manifests require exact source bytes; regenerate the relevant release manifest after intentional source edits.

### 2. Start the web interface

```bash
/usr/bin/python3 deployment/run_visual_task_control_v24.py --host 0.0.0.0
```

Startup opens the camera, inference process, and dashboard. Robot actions require a task-button command. The launcher requests a normal 1440 × 940 browser window without fullscreen; browsers other than Chromium retain their window preferences.

- Local access: `http://127.0.0.1:18081`
- LAN access: `http://<Ubuntu-host-IP>:18081`
- Find the Ubuntu host address: `hostname -I`
- Add `--no-browser` to start the service without automatically opening a browser.

When using a virtual environment, replace `/usr/bin/python3` with its interpreter and pass the absolute model-interpreter path through `--inference-python /absolute/path/to/python`.

### 3. Operate a task

| Dashboard action | Behavior |
|---|---|
| Automatic operation | Select qualified targets and run complete pick/sort/return cycles for the batch |
| Single-object operation | Complete the full cycle for one target |
| Continue recognition | Reobserve a vision-pending scene; does not resume a held-object recovery |
| Stop | Latch a software stop request; do not automatically release the hand or return the arm |
| Reset | Reset software task state only; do not clear device faults or move the robot |

Yellow boxes mark candidates awaiting two-second confirmation. Green boxes mark confirmed targets. A blue box marks the capture position of the target locked for the current cycle. Video stays live during motion, while the commanded target coordinates stay locked. Detection counts represent currently visible, confirmed objects rather than a sum across frames; completed motion cycles are reported separately.

Current settings are 30% for positioning/transfer, 15% for descent/lifting, a 180 mm pickup lift, a 40 mm post-drop raise for red/white, and an 80 mm raise for black. These settings belong to this installation and workspace; they are not general clearance or speed recommendations.

Use unpowered test objects, clear the full hand and carried-object route, and keep an on-site operator at the hardware emergency stop. A web stop or process exit does not establish physical standstill. Closing the browser leaves the service running; press `Ctrl+C` in its terminal to request stopping and wait for cleanup.

### 4. Records and optional interfaces

Records are written under `captures/`. On service exit, `real_task_control_v24_<timestamp>.zip` combines dashboard logs and the original robot session. A cycle that runs without lifting the object must not be counted as a successful physical sort.

HTTP exposes `GET /api/status`, `GET /live.mjpg`, and `POST /api/command`. To enable the ROS bridge, use another terminal with the existing ROS environment:

```bash
source ~/catkin_ws/devel/setup.bash
/usr/bin/python3 deployment/task_control_ros_v24.py
```

ROS and the dashboard share one real task-control service. See the [runtime notes](deployment/VISUAL_WEB_CONTROL_V24_README.txt) for interface details.

## Current scope

The current scope is a fixed camera/board installation, same-size upright objects, and two measured support levels. Arbitrary heights, other dimensions, square-object grasping, right-arm collaboration, and automatic verification of retention, slips, and correct-bin delivery remain incomplete. See [Third-party notices](THIRD_PARTY_NOTICES.md) for dependency and reference-source information.

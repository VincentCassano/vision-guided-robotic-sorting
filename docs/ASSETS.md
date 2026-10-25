# 运行资产配置 / Runtime assets

源码可以公开管理，运行资产应来自本机已验收部署。以下内容不会随GitHub源码上传；克隆仓库本身不构成真机就绪。

The source repository is separate from device-specific runtime assets. Restore the following from the accepted installation; cloning the repository does not establish robot readiness.

| 资产 / Asset | 位置 / Location | 配置要求 / Requirement |
|---|---|---|
| 当前模型 / Active model | `src/battery_grasp/models/battery_v3_camera.torchscript` | 黑、红、白圆柱及方形类别顺序须与代码一致 / Keep the class order used by the deployed model |
| 位置标定会话 / Position session | `calibration/position_rgbd_20261020_001259_216986/` | 完整会话及源文件，不手填通过标志 / Restore the complete session and its source evidence |
| 视觉任务绑定 / Visual anchor | `calibration/visual_pick_sort_v14_anchor.json` | 本机抓取腕姿与视觉位置关系 / Binding for this installation |
| 近处绑定与分区 / Near binding and regions | `calibration/visual_pick_sort_v21_near_binding.json`、`visual_pick_sort_v21_wrist_regions.json` | 原已验收近/远腕姿关系 / Accepted wrist-region relationship |
| 工作区 / Workspace | `calibration/visual_pick_sort_v20_workspace.json` | 当前板面选择范围，不是固定槽位抓取坐标表 / Board selection polygon, not a slot-coordinate lookup |
| 睿尔曼SDK / RealMan SDK | `/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package/` | 当前源码默认路径；Python包装与`libRM_Base.so`必须符合已保存来源哈希 / Existing default path and reviewed wrapper/native hashes |

当前TorchScript推理路径只使用PyTorch、NumPy和OpenCV，不要求额外YOLOv5源码树；历史`.pt`检查点入口另有其依赖，不是网页默认入口。

The default TorchScript inference path uses PyTorch, NumPy, and OpenCV without an additional YOLOv5 source tree. Historical `.pt` checkpoint paths have separate dependencies.

硬件身份、SDK源码、相机内参、当前工具/工作坐标系与保护配置由原运行流程核对。不要为了启动而修改校验值或用其他机器的文件顶替。设备、安装、头部或板面关系改变后，应重新处理相应标定与路径依据。

The runtime checks device identity, SDK provenance, camera intrinsics, frames, and protection settings. Do not bypass these checks by editing expected values or substituting another robot's files. Installation changes require the corresponding calibration and route work.

## 文件校验 / File integrity

```bash
sha256sum -c deployment/visual-pick-sort-v23-files.sha256
sha256sum -c deployment/visual-task-control-v24-files.sha256
```

这些命令只检查交付文件是否完整，不连接设备；通过不代表标定精度、物理停稳或实际抓取成功。

These commands only verify delivered files. They do not connect to devices or establish calibration accuracy, standstill, or grasp success.

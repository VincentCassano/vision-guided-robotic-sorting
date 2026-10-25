视觉预抓v7：先空手竖直退高→重新采当前目标→D435画面向左2.8mm→非接触下降

111031实际结果：工具接口1→0→1、恢复115200/100ms；原完整检查通过，
升高/高位对准/140mm悬停/下降4段均实际到位；臂4写、PREPARE手1写、通信2写。
终点用户填u，未验收物理对齐；后续确认无接触，空壳已移走。
一次完成不证明通信永远修复。新v7尚未上机，不包含夹持、提起或投框。

修正定义：手相对本次新目标，向头部D435实时画面的左边水平偏移2.8mm。
程序用原相机位置映射和新目标位置换算World水平偏移，保持World高度及腕姿。
不是直接X减2.8，不修改原视觉点、相机标定、抓取参考或任何设备参数。
原目标/偏移向量/修正目标分别保留。2.8mm为用户现场提出的抓取位修正，未实测验收。

运行前：空壳保持移走，手为空且仍保持111031完成后的臂姿/手型；
头、板、底座、升降高度及手安装不变；关闭其他控制/示教/播放脚本，现场守急停。
若臂手已移位或重启，不手动强行回旧位；本入口会核对并拒绝不匹配起点。
旧目标已失效，本程序只在退高阶段读取其历史来源，对准/下降强制重新采图。

新ZIP放~/test后，在机器人完整执行：
cd ~/test/battery-d435-preview && \
unzip -o ../battery-visual-pregrasp-motion-v7-20261021.zip -d . && \
sha256sum -c deployment/visual-pregrasp-motion-v7-files.sha256 && \
/usr/bin/python3 deployment/run_visual_pregrasp_motion_v7.py \
  --completed-run captures/visual_pregrasp_alignment_v6_20261021_111031_018476.zip \
  --previous-vision-archive captures/visual_task_binding_20261021_054800_335174.zip \
  --restart-tool-rs485 --execute \
  --confirm-empty-clear-exclusive --confirm-onsite-stop --confirm-inert-shell \
  --confirm-fixed-head-board-mount --confirm-shell-removed-for-retreat

运行顺序：
1. 沿用已实际成功的接口关闭/重开各一次；原模式、保护8/True和错误回读通过。
2. 完整配置/反馈/身份检查；新鲜当前臂必须匹配111031已完成下降终点，
   使用原0.08度/0.5mm/0.005rad匹配门，不用故障last_arm_state，不扩大漂移门。
3. 出现r提示时，确认空壳已移走、手为空，臂/整手/相机/线缆竖直升高范围无障碍。
   输入r：保持手型、不写手，固定当前XY和腕姿，竖直退高约150mm（2%）。
   高度为上次实际140mm悬停平面再高10mm，避免端点微差卡在原启动高度边界；
   仍保留原误差/速度/状态/电流/配置/保护及垂直走廊门。退高成功后SDK关闭。
4. 程序提示后，才将同一空壳放到本次要抓的同高完整槽，落底、撤手。
   输入v打开实时视觉；点空壳，S采集。看到PLANE TARGET SAVED后Q。
   不需放回原槽，不重新标定；新采失败则退出，不拿旧目标顶替。
5. 终端显示原始目标、World偏移及修正目标。完整检查通过后按m：
   PREPARE、必要时原位升高、高位对准修正XY、到修正目标上方140mm。
6. 按d前现场检查整段不会接触；偏位/不确定就q。d仅固定腕姿下降到修正预抓位。
   完成后p=实际对齐且各处无接触，n=未对齐/接触，u=未确认。

q/错误后不自动重发动作、松手、回臂、清错或降低保护。
非预期碰撞、滑落、非受控运动、明显升温立即现场硬件停止；退出不证明物理停止。
统一回传captures/visual_pregrasp_alignment_v7_*.zip，包含退高与新目标对齐两个子包。
不要只回传visual_pregrasp_stage_v7子包。

新v7只完成源码审阅、文件/CRC/哈希及交付核对；未运行离线项目测试、编译/AST/mock/FK/IK。
头部映射、同一45.8mm×98mm直立空壳及等高槽限制不变；完整六维/RGBD仍未通过。

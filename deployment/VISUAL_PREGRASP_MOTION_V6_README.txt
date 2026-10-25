视觉新目标预抓v6：一次工具RS485关闭/重开后，接回现有v4实际动作入口

依据：105801原生SDK直接读手，前三次成功，第四次1125×6返回
1: CONTROLLER_DATA_RETURN_FALSE，0完整组。错误后的零是无效输出缓冲区，
不是速度读数或速度写入。SDK直接读取也未解决；此前间隔/超时对照不重复。

本次会修改控制器通信接口：close_modbus_mode(port=1)一次，确认mode=0；
然后set_modbus_mode(port=1,baudrate=115200,timeout=1)一次，确认回到原
mode=1/115200/100ms。不是手复位、断电、清错或保护写入。
厂家协议：https://develop.realman-robotics.com/robot/json/modbus/
即使关闭结果不确定，也仅允许一次恢复原模式；恢复失败不再发包重试。
mode回读及ACK不代表手已修复，必须再通过原完整配置和反馈检查。

运行条件：手空、悬空；停止其他示教、播放、控制脚本和客户端；现场守急停。
目标文件054800对应的同一空壳、板、头部、底座、升降高度及手安装必须未变。
若这些变了，先用现有视觉绑定入口保存当前目标，不执行本条旧目标命令。
不要求回旧臂姿。每次动作仍读取本次实际状态；低于目标+140mm时拒绝启动。

把本ZIP放在~/test后，在机器人终端完整执行：
cd ~/test/battery-d435-preview && \
unzip -o ../battery-visual-pregrasp-motion-v6-20261021.zip -d . && \
sha256sum -c deployment/visual-pregrasp-motion-v6-files.sha256 && \
/usr/bin/python3 deployment/run_visual_pregrasp_motion_v6.py \
  --vision-archive captures/visual_task_binding_20261021_054800_335174.zip \
  --restart-tool-rs485 --execute \
  --confirm-empty-clear-exclusive --confirm-onsite-stop \
  --confirm-inert-shell --confirm-same-shell-scene

程序先核对目标来源和当前身份/错误/保护8与自碰撞True，以及原通信模式。
接口关闭/重开成功后，进入未修改的v4完整启动检查；失败保存退出，不动臂手。
显示m时，按现有提示核对整个扫掠区域无人、无障碍，再确认m：
保持PREPARE，先沿当前腕姿升到max(当前Z,目标Z+300mm)，随后高位对准，
到目标上方140mm。沿用原速度（升降2%、高位MoveJ-P 5%）及所有原门。
到达后按现场提示确认d，才下降至本次目标非接触预抓终点。
中断/拒读/错误后不自动重发动作、张开、归位或降低保护。
非受控运动、非预期碰撞、滑落、明显升温应现场硬件停止；退出不证明已停止。

最终统一结果：captures/visual_pregrasp_alignment_v6_*.zip。
外层包含通信原报文与原模式恢复结果，以及实际v4动作结果ZIP。
失败同样保存；不要失败后直接重跑旧运动命令。

范围：本次最多到视觉非接触预抓，不包含闭指、带载提起或投框。
新关闭/重开恢复路径尚未上机验收；不能保证解决拒读。
原v4、速度/电流/状态/配置/漂移/保护门和历史来源均不改。
本地仅源码审阅、文件字节及交付校验，未运行项目测试/编译/AST/mock/FK/IK。

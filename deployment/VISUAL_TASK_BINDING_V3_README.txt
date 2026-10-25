视觉任务绑定v3：固定采集输入区域（2026-10-20）

本次截图说明：S已经收到，采集已完成，顶中心和体中心仍超过原3mm稳定门；saved targets:0不是界面没收到按键。原201634的成功参考绑定保留，不重采。新一轮截图没有显示具体mm值，本次未收到该次原始ZIP，不能把历史5.476mm写成此次数值。

“视觉坐标抖动”不等于“机器人实际抖动”。旧201634新位置八帧的YOLO框x325/326、y86/87、宽107/108、高143～145；框确实变化。但相同[326,86,107,144]的两帧，顶圈短轴也从48.080变到50.635px，说明不能认定只有框在影响，仍可能有图像分割/拟合输入变化。本次只按源代码与原始字段确定这条输入依赖，不离线重算图像或新拟合。

v3具体修改：
1. S开始后，把这一刻选中目标的检测框固定为整个采集的几何测量框；每帧仍用实时原图独立定位，不把上一帧坐标当答案。YOLO原框照常保存并参与目标身份核对，目标必须保持原0.5重叠要求；丢失/更换目标仍拒绝，不锁死坐标伪造稳定。
2. 原locate几何模块逐字节保持不变，2px轮廓门、体轮廓门、8帧/2秒/3mm仍执行；不做均值/卡尔曼平滑，不删除波动帧后凑通过，不强行把Z锁到旧值，不修改相机、权重或标定。固定输入区域仅去掉框每帧变化这一干扰，不保证剩余分割/图像噪声已经消除。
3. 每一采集帧保存完整RGB PNG、哈希、原检测框、固定测量框及独立几何结果；Q一个ZIP统一回传。保存原图增加文件量，不增加相机、臂或手动作。
4. 采集结果在画面第二行持续显示LAST S REJECTED及具体视觉抖动mm，或LAST S: TARGET SAVED；失败后不再只让你看绿色实时候选猜有没有保存。

操作：旧窗口Q退出；把新ZIP放到~/test，执行下方命令。保留原captures/visual_task_binding_20261020_201634_400974下task_anchor.json及参考JSON/PNG。标题应为v3，原绑定自动加载。空壳保持当前想抓的新位置，选中按S一次；成功显示TARGET SAVED/saved targets:1后Q，失败也Q统一保留，不再反复按S。无需回原槽、重采标定/116mm核对或恢复旧臂姿。头部/工作板/底座/灵巧手安装与同一实测空壳仍须满足旧绑定条件。

cd ~/test/battery-d435-preview && \
unzip -o ../battery-visual-task-binding-v3-20261020.zip -d . && \
sha256sum -c deployment/visual-task-binding-v3-files.sha256 && \
/usr/bin/python3 deployment/run_visual_task_binding_v3.py \
  --position-session calibration/position_rgbd_20261020_001259_216986 \
  --confirm-fixed-camera \
  --confirm-measured-shell-only \
  --confirm-unchanged-setup \
  --anchor captures/visual_task_binding_20261020_201634_400974/task_anchor.json \
  --inference-python /usr/bin/python3 \
  --device auto

范围：只读相机与候选保存，不连接SDK、不移动或闭指；不是完整抓取已验收。现有v2参考可复用，因为几何模块、相机/模型/标定和参考点本身未改变，旧原件和旧ZIP不改；v3运行源码哈希单独记录固定输入策略。新位置稳定性、实际预抓对齐、绝对精度及整轮自动投放仍待真机。本机未运行项目测试、AST/compile/mock、FK/IK或离线图像拟合。

关于之前标定的5mm：它是验收门限，实际各点残差不同；当时使用ArUco角点而非YOLO框。同一批六组中PnP最大约3.431mm，RGBD最大约7.498mm，说明两条视觉测量链路存在差异，不能据残差断言机器人抖动或肉眼可见位移。当前框/顶圈波动也不能反推当时标定的唯一根因；原RGBD/六维失败保留。

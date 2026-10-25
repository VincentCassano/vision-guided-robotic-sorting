视觉同高任务平面v4（2026-10-20）

当前真实进展：模型、局部路径、实物单件夹起/现场入框已有各自证据；完整“实时视觉新位置→对齐→夹取→分类投放”的连续真机流程尚未完成。201634参考绑定已完成，不能把后续拒绝的新目标/绿色实时候选当成已验收运动坐标。本包仍是感知与任务坐标入口，不是自动抓取已完成。

为什么改方法：205634实机v3原包28成员、27项内部哈希、5项实际运行源码哈希、16帧RGB哈希全部一致，旧参考锚点逐字节保留。固定ROI确已生效，第一组8帧/3.195秒仍最大波动7.666mm（体5.870），第二组8帧/3.184秒顶3.147mm（体2.590），目标仍0。第一组分割横向宽度约74.811～74.816px，变化约0.005px；反推椭圆短轴却47.014～50.608px，变化约3.594px。问题主要指向小段上轮廓自由反推完整椭圆/圆面姿态的敏感性，不是测到机械臂实际移动，也不只是YOLO框在变。本次不离线重拟合原图、不重跑模型确认结果。

新的限定任务：你已经确认同一45.8mm×98mm黑色空壳能在另一完整槽完全落底、直立、露出高度相同。本包利用这一条件，使用本次实际图像的相机视线与参考顶面求交，计算连续位置，再平移原已接受预抓TCP。它不是按槽号查表、不是把位置吸附成116mm，也不是学校固定槽抓取；但它是同高场景的2.5D方法，不是自由三维深度/任意高度/任意姿态定位。

平面定义与未知：原参考三维顶中心保持原值；法向来自原合格8帧圆柱轴方向的算术平均，没有删帧或重新拟合图像。该方向是视觉估计，未独立测量为实际板面法向，平面位置/方向的系统误差仍可能使新预抓偏离。相机内外参、手装配、工作板与底座关系保持；不读取或改变头部。若空壳悬空、倾斜、半落槽、不同支撑高度/尺寸，不能使用本模式。不能把“平面法稳定”当作“平面准确”，新位置实物对齐仍是后续必要验收。

计算：相机中心C、变换后的实时视线d，参考面上一点P0与估计法向n，交点P=C+d*[n·(P0-C)/(n·d)]。正前方且处在原0.1～2m相机Z范围内才保留。相机模型公式依据OpenCV官方文档：https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html 。本公式仅为任务平面推导，不证明原外参已达到绝对精度。

每帧仍保留原几何、原自由3D坐标和原3mm判定；同时单独记录equal_height_plane_projection。原轮廓2px、体轮廓/根分支图像门、固定ROI身份门保持。平面法独立使用8帧/2秒/3mm连续稳定门，结果字段明确标记equal_height_plane；这个3mm指标不是原自由3D门，不把原3D失败改判通过。不平滑坐标、不筛除波动帧，不改旧标定、权重、旧task_anchor或原参考点。高度是同高平面约束推定，不能声称是本次独立深度测量。

直接执行（空壳仍在当前同高完整槽，不回原槽重采）：
cd ~/test/battery-d435-preview && \
unzip -o ../battery-visual-task-plane-v4-20261020.zip -d . && \
sha256sum -c deployment/visual-task-plane-v4-files.sha256 && \
/usr/bin/python3 deployment/run_visual_task_binding_v4.py \
  --position-session calibration/position_rgbd_20261020_001259_216986 \
  --confirm-fixed-camera \
  --confirm-measured-shell-only \
  --confirm-unchanged-setup \
  --confirm-upright-equal-height-recesses \
  --anchor captures/visual_task_binding_20261020_201634_400974/task_anchor.json \
  --inference-python /usr/bin/python3 \
  --device auto

操作：旧窗口若仍开着先Q；新ZIP放到~/test后运行上述命令。标题为v4 EQUAL HEIGHT PLANE；原参考自动加载。点当前空壳，S一次；画面PLANE TARGET SAVED且saved targets:1表示平面候选保存，最后Q一个ZIP。若原自由3D仍失败，画面会同时写FAIL(retained)，不影响对旧失败的保留。新平面法若仍失败也Q保留全帧，不重复空重试。

输出：equal_height_plane.json保存假设、估计法向、原参考来源/源码哈希；每帧原图及原始/平面坐标都保存；target_001.json和latest_target.json包含固定腕姿预抓TCP候选、原3D判定和高度推定范围。task_anchor及参考图/JSON原样复制，可按程序退出打印的v4完整命令复用。

后续执行缺口：新目标的实际预抓对齐和全臂/手/线缆通道仍未验收；此包没有连接机械臂SDK或发任何臂手动作，execution_ready与motion_authorized始终false。上方140mm仍仅是接口候选，不是实测净空。夹持、带载移盒及空返需使用新鲜臂手状态和相应实机执行器，不能只拿本JSON强行下降。本次没有运行项目测试、AST/compile/mock、FK/IK、旧图拟合或新方法离线验算，也不声称新现场方法已经稳定。

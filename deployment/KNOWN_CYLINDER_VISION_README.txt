圆柱视觉 v4：修复v3布尔类型崩溃，优先恢复现存A/B数据 / 2026-10-20

本次真实日志：A第一次因3mm抖动失败；第二次A合格并打印A SAVED；B采集后发生bool_序列化异常。
这是程序错误，不是用户操作问题。原相对计算中的NumPy布尔值使打印失败，又连带使退出时result.json失败，ZIP未生成。
v4：相对结果显式转原生bool，所有记录递归转换NumPy标量/数组；JSON先编码再原子替换。
B的观测与状态先落盘再打印；中断/清理/结果保存各自处理错误，最后独立尝试打包。A SAVED同时显示在图像顶部。
不放宽3mm，不删掉失败A，不改变5mm/2°、相对10mm门或原FIT空间保护。

当前先恢复，不重新采集A/B，不用搬电池、摆臂或重启：
将新ZIP放入Ubuntu ~/test/，直接执行：
cd ~/test/battery-d435-preview && \
unzip -o ../battery-known-cylinder-vision-v4-20261020.zip -d . && \
sha256sum -c deployment/known-cylinder-vision-files.sha256 && \
/usr/bin/python3 deployment/recover_known_cylinder_capture.py \
  --recover-latest \
  --reference-spacing-mm 116

恢复入口只读captures中最新的原始known_cylinder_日期目录，不会偷偷选旧成功结果。
先把原文件复制到新恢复目录、逐项留SHA256，再读取已保存的合格A与interrupted_burst中的B帧。
它不打开相机、不跑YOLO、不加载SDK、不查设备、不发臂/手/配置命令，不重新拟合图像、几何或相机外参。
B仅恢复已存帧的统计与A/B相对距离，缺失的原始B图片、完成时刻及最终运行元数据明确标缺，不伪造。
若B帧缺失或不合格，保存具体原因及已有原件，仍生成恢复ZIP，不先要求重采。
终端最终打印SAVED: .../captures/known_cylinder_recovered_*.zip；只需这一包。
原目录不修改，失败A仍保留。A/B通过也不表示绝对5mm、超FIT空间或抓心/TCP通过，execution_ready恒false。
如最新目录并非这次崩溃目录，程序明确退出，不悄悄回退；可将--recover-latest替换为--source-session captures/本次实际目录。

后续需要新现场采集时，才使用修复后的入口（本轮先不用再跑）：
/usr/bin/python3 deployment/run_known_cylinder_vision.py \
  --position-session calibration/position_rgbd_20261020_001259_216986 \
  --confirm-fixed-camera --confirm-measured-shell-only \
  --reference-spacing-mm 116 --inference-python /usr/bin/python3 --device auto

现场A/B顺序保持：当前完整槽选真实空壳并S；A SAVED后才搬入中心相距116mm的同侧等高完整槽，落底撤手，重新选目标并S，Q打包。
8帧/约2秒、按帧检查15秒采样上限，单帧取帧或推理阻塞另计；所有帧的检测/几何记录保留，每个窗口保存末帧原图/显示图。
3mm是新开发稳定性规则，不是绝对精度。相对距离/同高差10mm界是两个5mm端点界的必要条件，不能证明单点绝对误差。
只用实测45.8±0.1mm外径、98mm总高的同一黑色空壳；可见高度仍按图像估计，总高不当露出高度。
RGB相机344322075008、1280×720@30和原内参保持；标定板先保留，头/相机/底座/升降关系不变，不重采16组或6组。
World候选超原FIT采样空间仍标记，不发运动。原RGBD位置与六维失败保留，带载取放未完成。
本次只做用户原文记录、源码审阅和ZIP字节核对，未运行离线项目测试/AST/编译/模拟SDK/FK/IK或新的离线标定。
恢复与v4实机结果均待执行；不将代码修复当成恢复成功或真机验收。

视觉预抓v8：原图像左2.8mm基础上，再向左下追加合计4.5mm

本轮实际回传143310：只收到v7对准子包，3段移臂完成并到140mm悬停位。
只有m权限，没有d权限；等待d约560秒超时后中止，未执行最后下降。
臂3写、PREPARE手1写、配置0写；SDK清理完成。未收到外层包，不能补称接口重开/退高通过。
新图像目标平面重复性约1.213mm；不是绝对定位精度，也不代表小拇指净空。

用户已确认偏移含义：保留D435画面向左2.8mm，再向画面左下方向追加合计4.5mm。
不是总偏移4.5mm，也不是左、下各4.5mm。左下取图像方向(-u,+v)，
用当前目标、原相机位置映射和内参换算水平向量，再将追加向量长度定为4.5mm。
合成后World-Z和腕姿保持不变；画面“下”不是机械臂Z下降。
程序分别保存原始目标、原2.8mm向量、新4.5mm向量、合成向量和修正目标。
不改标定/视觉检测点/参考抓取位，不把用户修正当已验证无碰撞。

当前入口只续接143310已完成的140mm悬停段。当前臂手必须未被其他项目移动/重启，
手为空、保持PREPARE；头、板、底座、升降高度和手安装不变。
不需要把空壳移走或放回原槽，但r前必须确认整手/臂/相机/线缆竖直上方清空。
若当前已移动，程序拒绝这条限定续接，不自动回历史位置；不使用fault last_arm_state。
关闭其他控制、示教和轨迹播放；现场全程守急停。

将新ZIP放~/test，完整执行：
cd ~/test/battery-d435-preview && \
unzip -o ../battery-visual-pregrasp-motion-v8-20261021.zip -d . && \
sha256sum -c deployment/visual-pregrasp-motion-v8-files.sha256 && \
/usr/bin/python3 deployment/run_visual_pregrasp_motion_v8.py \
  --completed-run captures/visual_pregrasp_stage_v7_20261021_143310_559418.zip \
  --previous-vision-archive captures/visual_task_binding_20261021_143310_690371.zip \
  --restart-tool-rs485 --execute \
  --confirm-empty-clear-exclusive --confirm-onsite-stop --confirm-inert-shell \
  --confirm-fixed-head-board-mount --confirm-empty-clearance-raise

输入顺序：
1. 沿用v6的一次接口关闭/开启及原模式回读，保护8/True不变。
2. 完整读取当前臂手；按原0.08度/0.5mm/0.005rad匹配143310已完成悬停段。
   各项通过后，r仅保持手型、固定XY/腕姿竖直抬高约10mm（2%），不写手。
   这给当前140mm悬停边界留10mm高度余量，不放宽任何漂移/反馈门。
3. 抬高成功且SDK关闭后，确认空壳在本次要抓的同高完整槽且落底，撤手。
   v打开视觉，点空壳、S采集；看到PLANE TARGET SAVED后Q。失败不拿旧目标补位。
4. 终端显示原2.8mm、新4.5mm及合成World向量。m=准备手型、必要升高、高位对准、140mm悬停。
5. 明确看清修正后整段下降不会接触时输入d；偏位或不确定就q。
   本轮上次正是停在等待d；这不是自动继续步骤，不输入会按原560秒期限退出。
6. 最后p/n/u只记录实际终点，不自动夹持。统一回传visual_pregrasp_alignment_v8_*.zip。

新v8未上机验收；本次只到非接触预抓，不闭指、不提起或投框。
拒读/错误即中止，无自动动作重试、张开、归位、清错或保护降级。
非预期接触、滑落、非受控运动、明显升温现场硬件停止；程序退出不证明物理停止。
旧六维/RGBD失败及同一45.8mm×98mm空壳/同高完整槽限制不变。
本地仅原始JSON/ZIP/哈希和源码审阅、文件交付，无项目测试/编译/AST/mock/FK/IK或SDK执行。

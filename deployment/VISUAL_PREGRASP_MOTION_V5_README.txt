视觉预抓v5：恢复已有真机使用过的末端Modbus启动步骤（2026-10-21）

本次实际结果
070227 v4完整profile和三组启动反馈通过，只读窗口30.005秒，6次明确拒读、7个只读连接均如实保留。
用户已输入m；动作前再次读手状态1085×6得到read_state:false。随后退出，臂/手/配置0写，没有准备、升高、对准、下降或再次4109。
上一版只解决了启动读取覆盖范围，没有解决正常阶段仍拒读的通信问题。

确定的软件遗漏及历史依据
学校grasp_object.py与ohandtest.py均先Set_Modbus_Mode(1,115200,1,True)。
本项目10月14日214212同类缺失已定位，HandBusStartup模块已实现；221041实际得到set_state:true，随后完整配置/预检及239笔多寄存器、12笔单寄存器读取成功，后因独立的手状态3停机。
新视觉入口没有调用这个已存在的启动模块。这是确定的集成遗漏；它是否造成本次间歇拒读，仍由新一轮实读验证，不能预先宣称根因已解决。
官方协议要求机械臂启动后先配置端口ModbusRTU模式再操作；timeout1的单位是100ms，不是1秒：
https://develop.realman-robotics.com/robot/json/modbus/

本次启动时会发一条配置指令，不是纯只读
--initialize-hand-modbus明确授权一次固定指令：
{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout":1}
它配置控制器末端RS485接口；不写手寄存器、不复位/自检手、不切电、不修改碰撞保护/负载/坐标/限流。
只发送一次，返回失败/不确定不重发，不自动关闭再打开端口，不增加timeout或改变学校波特率。
先读实际身份、健康臂状态与保护8/True。可选工具端模式GET如不支持保留未知，不假装读取成功。
设置ACK后再次核对保护；如模式GET受支持，其主站/115200/timeout1回读也须一致。
随后调用字节完全不变的v4动作入口，重新读实际臂手、完整配置、状态与限流；不是凭通信ACK批准运动。
v4原有动作前只读窗口保留；m许可后的任何新通信错误仍终止，不恢复令牌、吞掉失败或重发动作。
总报告将通信子进程与动作子进程的写入计数相加，不把启动配置写入隐瞒成0。

执行（新ZIP放~/test）
空手悬空，退出示教/播放，关闭其他控制脚本；保护保持8/自碰撞开启，现场守急停。
054800采图后，同一空壳、工作板、头部相机、底座/升降及手安装不变时：

cd ~/test/battery-d435-preview && \
unzip -o ../battery-visual-pregrasp-motion-v5-20261021.zip -d . && \
sha256sum -c deployment/visual-pregrasp-motion-v5-files.sha256 && \
/usr/bin/python3 deployment/run_visual_pregrasp_motion_v5.py \
  --vision-archive captures/visual_task_binding_20261021_054800_335174.zip \
  --initialize-hand-modbus \
  --execute \
  --confirm-empty-clear-exclusive \
  --confirm-onsite-stop \
  --confirm-inert-shell \
  --confirm-same-shell-scene

无需重采图、重新辨识、改负载或回旧臂姿。若目标/场景变了，不能使用旧目标，应先用既有v4视觉入口采当前目标后提供对应视觉ZIP。
同一终端继续原m/d操作：m前确认准备、原位升高、高位对准及下降全段无遮挡；悬停实际对齐、下降无接触才d，偏位/不确定q。
本入口仍只到非接触预抓，不夹持、提起、投放或自动回臂。意外接触或非受控继续运动立即现场硬件停止。
程序Stop/退出/通信ACK不是物理停止、卸扭矩或持力证明。故障没有自动张开、清错或动作重试。
最后只回传captures/visual_pregrasp_alignment_v5_*.zip，已包含通信设置与内嵌完整v4动作/视觉原图证据。

交付核对为原始日志、源码、文件/ZIP/哈希；没有运行项目测试、AST/compile/mock、FK/IK、SDK或设备动作。
新v5现场效果待验收，不把历史通信成功、代码交付或设置ACK当成当前视觉抓取成功。

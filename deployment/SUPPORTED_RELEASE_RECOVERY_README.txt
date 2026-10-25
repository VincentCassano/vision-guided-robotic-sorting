独立有支撑松指恢复 v1 / 2026-10-18

023145实际结果：续接核对通过，g已收到，六路夹持目标一次ACK；第一次后读被控制器read_state:false拒绝。
本轮机械臂动作0次，手部目标1次；未冻结、未提起、未张开。last_snapshot是夹持前读数，不代表当前手型。
零空等读法已撤回；保留已有300ms主机事务间隔，不声称它适用于快速接触控制或解决发热。
目前暂停v3/v4低提离抓握，不重复g；旧发布ZIP不覆盖。此包更新共享依赖后，旧低提离清单不匹配而拒绝启动。

用途：仅处理电池仍在手里的原位置夹持。不移动臂，不新抓，不自动回旧位置，不复位、清保护、改电流/PID/速度。
若已张开/取出/重新上电，不为运行本程序再夹住电池；该入口不是下一轮抓取程序。
若仍有异常喀喀声、持续转动或升温：立即现场硬件停止，不等软件读取，不用本程序代替急停。

运行前：固定支撑或工作板承托空壳全部重量，不能人手托着；张开范围无遮挡，关闭其他控制，现场有人守急停。
程序读取本次臂/手及参数后才提示r，等待输入时不查询SDK；确认后再次读取新鲜状态，变化即退出。
r只允许一次1135起始的前四通道8000；小拇指/对掌目标不写，臂不动。
原有前四路状态3仅限带支撑向张开恢复；不清保护，状态4/新状态3/限流/通信异常仍中止。
单次目标未得到到位反馈不重试、不继续开大、不自动回臂。q或进程退出均不证明停机/断扭矩。
设备、模式、当前目标或方向不符合既有配方就无写退出；不放宽条件迁就未知状态。

cd ~/test/battery-d435-preview && mkdir -p docs && \
printf '%s\n' 'unzip -o ../battery-supported-release-recovery-v1-20261018.zip -d .' >> docs/执行命令日志.txt && \
unzip -o ../battery-supported-release-recovery-v1-20261018.zip -d . && \
sha256sum -c deployment/supported-release-recovery-v1-files.sha256 && \
/usr/bin/python3 deployment/run_supported_release_recovery.py \
  --execute --confirm-supported-inert-shell --confirm-onsite-stop \
  --confirm-exclusive-control --allow-protected-opening

结果保存在captures/position_grasp_release_*.zip。程序自动记录本次实际执行命令。
范围：仅源代码阅读、实际回传分析、包内容/哈希核对；未运行项目测试/离线IK/AST/模拟SDK，未连接设备。
013358已有受承托四路张开成功证据；本包复用旧独立松指worker，入口/安装组合仍待真机使用，不宣称已经完成本次松指。
完整抓握仍缺快速可靠反馈、逐指接触降目标及实际试提验收；本包不开放带载动作。

空手退回PREPARE v1 / 2026-10-18

现场补充：023145报错后电池已取出；取出后无名指继续弯曲到原指定位置，声音停止；尚未张开。
这是现场观察，未测温/力/实际电机转速；不能据此宣布发热、堵转重试或停止问题已解决。
当前臂姿、手指读数、是否重启仍由本次新鲜前检决定；不沿用023145夹持前last_snapshot。

本入口仅限手内无物，向张开扫掠内无电池/桌面/其他障碍，现场守急停。
若仍有异响、继续运动或升温，先现场硬件停止，不等待软件。
不需要重新摆臂或夹电池，不需要虚假确认“电池有支撑”。
只写一次1135起的前四个通道8000；小拇指和对掌目标不写，臂不动。
这是PREPARE；不是全手伸直，不旋回拇指，不自动升降/回旧位置。
如果当前手型已变为OPEN/未知配方，或8000会增加弯曲，前检直接拒绝，不强行套用。

每次新读身份/参数/保护/当前臂手；前后两次当前读数匹配后才允许一次r。
原前四路状态3仅限向张开恢复，不清保护；状态4/新状态3/限流/漂移/通信异常中止。
保留既有300ms主机事务间隔；不再用零间隔读，不改PID/限流/速度/硬件堵转参数。
q/程序退出不证明停止或断扭矩；目标未确认不自动重试。

cd ~/test/battery-d435-preview && \
unzip -o ../battery-empty-prepare-recovery-v1-20261018.zip -d . && \
sha256sum -c deployment/empty-prepare-recovery-v1-files.sha256 && \
/usr/bin/python3 deployment/run_empty_prepare_recovery.py \
  --execute --confirm-empty-clear-exclusive --confirm-onsite-stop \
  --confirm-exclusive-control --allow-protected-opening

程序提示后，再确认空手且张开路径无障碍，输入r；q保持退出。
结果保存captures/empty_prepare_recovery_*.zip；程序自动记录实际启动命令。
本包源代码/文件完整性已核对，未运行项目测试或SDK；本次空手入口尚未真机验收。
完整夹持和20mm提离继续暂停，不能用空手张开成功代替带物接触控制验收。

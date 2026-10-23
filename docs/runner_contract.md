# P0-05 单回合 runner 合同

父进程是唯一进程创建者。runner_core 无 ROS；nav_goal 只进行 ROS 通信和被动采样，不创建子进程。官方 launch autostart=false，由 worker 各发一次 STARTUP。

状态：PRECHECK→SNAPSHOT→START_SIM→SENSOR_READY→START_RECORDER→LOCALIZE→NAV_READY→SEND_ONCE→WAIT_RESULT→OBSERVE_STOP→SHUTDOWN→INSPECT_BAG→FINALIZE。失败不回退重试。

时限：工作360秒，启动累计180秒，目标120秒（含接受5秒），取消额外10秒，停止墙钟30秒且至少3仿真秒；清理60秒、录包检查与写入30秒独立保留。严格以父接收时刻 < deadline 判断按时；迟到成功保留SUCCEEDED但harness=INTERRUPTED。单调时间计期限，UTC只标记。

IPC JSONL protocol_version=1，run_id/seq/request_id/type 必须合法，行最大65536字节；stdin/stdout非阻塞，日志写stderr；worker每0.5秒心跳。重复request_id重放已完成响应，不重复副作用；新的第二次send也拒绝。UUID在发令前持久化且永不使用零UUID取消。

首因只写一次，后续错误追加。manifest早期冻结，sim_start=null；result原子排他发布，调用既有validator完整校验，不改合同。未知碰撞=null。runner=0须行为、bag、清理和记录均通过；2=启动前拒绝；3=已建回合未通过；130=中断。

清理：取消/停止→navigation manager SHUTDOWN及launch leader SIGINT→录制结束标记/recorder flush→localization→bridge/RSP→Gazebo正常停止服务→worker。强制升级或身份不明为FAILED。PID/start_ticks/SID、观察到的后代身份一并核对，绝不全局pkill。项目flock只约束本项目runner；外部ROS owner仍需拒绝。SIGKILL/断电/不可中断I/O无法保证finally。

设计审查结论：状态、发令和进程归属明确；不使用现有P0-04临时实施脚本作为生产runner，不更改原控制参数。后续代码与测试审查记录保存到开发验收目录。

# P0-06 实现前合同（第1～6步）
本次只编译和离线验证；第6步后由本人审查，未批准前不得执行真实探测。
复用P0-05父进程、项目锁、WorkerLink、ProcessSupervisor、DeadlineBudget和结果v1，不增设生命周期。
新ScenarioSpec v2仅支持crossing/S1和observed_occlusion/S3。draft绝不执行；ready_for_probe需带hash的静态准备证据；ready_for_recording另需关闭的real probe证据和逐项实体/时间/扫描/暂停/隔离验证。
本次坐标是离线候选，正式文件均draft。服务名是安装接口候选，未声称运行发现。
普通C++ driver用单一Transport Node，订阅原始Gazebo clock/stats，绝对整数纳秒轨迹，最多一个在途set_pose请求。重复clock不发下一点，回跳失败；相位不依墙钟；超过50ms仿真调度间隔或0.5秒墙钟请求期限失败，不追赶历史点。
WAITING→ARMED→MOVING→HOLD→FINISHED；QUIESCED仅probe暂停阶段，无活动目标。重复同一event不重置，不同event拒绝。迟到arm拒绝；释放=接受采样ROS时刻+已冻结delay，ROS/GZ映射未核实不能开始。
设定pose、服务Boolean和Pose_V实际状态三份独立记录。capture只订阅实际Pose_V/clock/stats/scene/contact，保存无损protobuf JSON及整数源stamp和接收monotonic；原始流不从命令生成。
原始LaserScan完整存MCAP。truth不发布ROS话题，直接sidecar，因而没有真值TF/odom桥；仍必须保存实际Nav2图核对其他输入没有变化。
离线检查以scan.header.stamp逐束time_increment对齐同epoch实际实体；仅短间隔内插，不外推。圆柱射线几何和原始range同时符合才可标visible/occluded；缺数据/边界不明确为unknown，不能false。
停止观察、Action、动态有效性、record合同、清理各自判定；collision始终未知，动态检查不设置碰撞评价。
初始候选阈值：0.02s命令周期，0.05s调度/真值间隔，0.15m/s速度，0.02m位置误差，0.05m激光调查误差；均须第7～9步实测/冻结，不能运行后放宽。
准备资源包括原world和全部model/config/mesh/material、robot/map、派生world/actor/motion、安装库、C++源码/二进制及参数。修改任一受审输入使准备指纹失效。
程序只接收受限JSONL命令；所有事件带run_id/clock_epoch/entity身份。raw Gazebo时刻与ROS时刻同名不代表已对齐；未测映射使动态记录无效。


## 审查时的实现补充
- C++事件的gazebo_sim_ns为整数源时间，未核实的ros_s=null；ROS事件与Gazebo原始流须用共同时间段核验，不事后平移时间轴。
- 轨迹按绝对整数纳秒计算；主循环合并已经到达的clock到最新调度点，不突发追赶旧点；RequestGate限制一个在途请求、0.5墙钟秒应答、50ms仿真请求间隔。目标步长由0.15m/s×50ms=7.5mm约束，名义20ms为3mm。
- 正式触发绑定本轮已接受UUID；probe只用PROBE_ARMED。几何准备指纹覆盖世界、任务、实体/轨迹/阈值、事件及依赖；probe与正式执行purpose/trigger单独校验，不因事件来源不同制造无法转交的准备证据，也不能跨S1/S3借用。
- planned pause先quiesce并有界排空，核实stats.paused和最终clock一致才开始3墙钟秒观察，恢复预算10秒。只有probe且没有目标可用；导航stale条件不变。
- 相同scene回复不能证明暂停实际位姿；要求同paused源stamp的新鲜Pose_V及stats，缺样本保持未核实。
- C++接收器只写原始sidecar，无真值ROS发布者。Nav2实际订阅/控制源审核代码已接入，生产图仍待真实阶段。
- v2归档复制闭包并保留原字节hash，portable索引不依赖当前/opt路径；旧manifest/result继续v1。动态检查先用同一validator检查real记录合同，再复算raw数据；禁止向real记录注入合成scan。
- 当前新world原生SDF check=255，与原world错误输出逐字相同；新actor=Valid/0。没有修原官方模型、没有把继承诊断改成通过。真实加载/物理/激光仍未验收。

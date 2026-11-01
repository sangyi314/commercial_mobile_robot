# P1 Execution v1 — 运动授权合同

2026-11-01。沿用Mission v1的单写者、持久意图和未知占槽。此合同定义最终实现语义；分层实测结果与限制见docs/verification/p1-03-un63b3ef.md。

## 基准和来源

基准HEAD为079e9d701b99f8d9c0bcb81713a50adf4f0ba334；前轮6ab9bb1未提交成果由用户提交，已按文件指纹证明测试输入相同。纯核心17组、A01–A20分层协议及独立审计通过。原始基准在evidence/env/p1-03-dev-xehwla33；实现证据在evidence/env/p1-03-impl-un63b3ef。保护P0冻结输入/旧evidence，不自动提交。

研究twist_mux a8225e2d9bf0171aac447c246376f0981defd17b的src/twist_mux.cpp、include/twist_mux/topic_handle.hpp（BSD-3-Clause）：借鉴来源仲裁、锁、过期，不采用其ROS now超时及消息触发发布作为独立watchdog。研究Nav2 d5d768b7b38a05e7cfd2b6c693b786b402161bed的Collision Monitor实现、test/collision_monitor_node_test.cpp、Velocity Smoother（Apache-2.0）：复用安装组件，借鉴有界输入和独立超时测试；stop_pub_timeout后不再发零，不代替Gate。CMR Gate与停止判定独立实现；直接复用cmr_core有限数/轮速检查，不改原CommandGate。

## 速度链调查与最终派生配置

| 段 | 当前已读依据 | 最终派生配置；r20运行图已核对 |
|---|---|---|
| Controller/Behavior | 安装navigation_launch.py均remap cmd_vel→cmd_vel_nav；nav2_util/TwistPublisher默认enable_stamped_cmd_vel=false | 全链显式true；Controller/Behavior为允许来源 |
| Velocity Smoother | Nav2 1.3.13；velocity_timeout=1s；源码默认stamp_smoothed_velocity_with_smoothing_time=false，保留输入stamp | 保留来源stamp，不在末端重写；冻结输入超时/频率 |
| Collision Monitor | cmd_vel_smoothed→cmd_vel，source_timeout=1s，stop_pub_timeout=2s；publishVelocity保留header | 输出改到本run私有candidate，Stamped |
| Bridge | 官方config cmd_vel，Twist→gz.msgs.Twist，ROS_TO_GZ | 派生配置移除此入口，唯一私有final→唯一私有drive；剥离stamp边界在Gate最终Twist输出 |
| DiffDrive | 冻结expanded robot.sdf；轮半径0.033m、轮距0.287m，订阅/cmd_vel，未配置独立timeout | 新派生SDF改为私有drive；8.15.0源码未发现独立命令timeout；按保持旧命令风险处理 |
| docking/其他 | 原launch启动docking_server并纳入lifecycle列表 | 新launch不启动docking，并同步readiness/lifecycle；不依赖inactive消除端点 |

新配置不改P0；运行前检查ROS类型/发布者/订阅者/GID和本轮进程归属、Gazebo topic来源/partition。图监测有发现延迟，不宣称恶意节点隔离或瞬时旁路检测。实际r20七回合记录及无执行端额外GZ来源负例见验收报告；正式链CM直连Gate，测试relay不在路径。

## 身份、控制、线性化

每次请求携带run、gate_epoch、mission_process_epoch、mission、operation、attempt、lease、request_id、request_seq和challenge。Gate进程生成新随机epoch，默认DISARMED；只接受配置owner，旧epoch永不复活。标识仅关联与防误重放，不是密码学认证。

控制状态DISARMED/ARMED/STOPPING/FAULT_LATCHED与STOP_UNKNOWN/MOVING/STOPPED独立。Begin为已提交SEND_PENDING准备授权，不要求Action先接受；原目标必须解析，并有独立初始静止资格。challenge在Gate本地steady时间发行/到期，单次消费，最多一个未决。Begin相同request_id同内容返回原决定/期限，冲突拒绝；最多64条Begin历史，容量耗尽保守锁存，不驱逐后允许旧ID复活。Renew严格递增seq，同请求重复不延长期限；过期、撤销及故障均不能靠Renew恢复。Revoke幂等清缓存；Reset只回DISARMED，不自动授权，需旧Goal解析和新D窗口。

Gate所有输入、控制和tick/最终publish位于同一个SingleThreadedExecutor；不把先算出的非零移到另一个线程发布。服务Revoke先改变状态、立即发布零后回应。tick热路径无磁盘/SQLite/同步RPC。日志由独立订阅者记录。keep_last(1)、volatile输出；DDS/Bridge已排队的旧值不在本地撤销保证内，单独观测。

## 期限与候选

起始测试预算：tick20ms、lease300ms、challenge100ms、候选接收年龄200ms、来源年龄300ms（ROS域）、clock无进展300ms（steady域）。先测sink调度再冻结，任何调整都新版本/新回合，不追认失败。now==deadline无效。

只允许有限差速分量、指定frame及线/角/轮速限值，拒绝NaN/Inf、非法分量、缺失/未来/旧epoch来源时间。数值原样通过，绝不裁剪冒充合格。重复来源stamp不刷新首次接收时间；同stamp速度变化计审计次数，可在原始年龄窗口内采用合法更新以兼容平滑器，绝不重置年龄。来源回退、clock回退/停滞、候选过期、Revoke/Reset均清缓存并禁止自动恢复。新租约仅接受严格晚于授权边界的来源时间。

TwistStamped没有Goal身份，当前仅在单目标、旧Goal已解析、撤销/停止窗口、清缓存、新鲜来源时间的受控链成立；不声称逐速度UUID证明。未知Goal禁止新的Begin。

## Mission集成设计

保留原FSM/Store/Runtime。真实ExecutionPort承接已持久Send意图，先握手；Gate授予后通过owner事务绑定gate/clock及授权派发意图，再发一次Action。续租每次须经owner重新检查当前活动attempt、允许业务状态、数据库事务健康；无独立续命线程。Revoke先走Gate，精确Action取消仍走既有NavigationAdapter；合成REVOKE_ACK在真实路径关闭。

P1-03已追加ExecutionContext/Bind/Renew/Clock事件，以及BeginLease/SendAuthorized/RenewLease持久意图。绑定Gate的Stop入口要求经过来源、身份、时钟、generation、窗口和接收新鲜度验证；旧fixture模式只在既有测试边界内保留，真实Gate路径拒绝人工注入stop/arrival/resolve。不得直接把消息中的STOPPED布尔写入Record。

适配器用本进程请求发出时刻加回包remaining_ms保守估算租约期限，扣除通信往返，不从回包接收时刻重新延长TTL；发送还要求新鲜的匹配GateState允许。所有授权/续租意图在owner事务提交后才交给ROS层。

G01–G07使用原worker唯一发Goal、授权owner替身；真实Mission失效先以Fake Action+sink单列。不得把二者合并声称真实Mission崩溃后的物理停止。Gate死亡/冻结及下游失联先做保持末值sink负例，无独立下游保护则UNSUPPORTED。

## 显式时钟与输入格式加固

Gate/Observer 的 use_sim_time 必须 false，且初始化后参数冻结；它们不读取 Node.get_clock，而只消费经过 GID 校验的显式 Clock 消息。NodeOptions.use_clock_thread(false)，避免 rclcpp 隐式 TimeSource 线程和负时间转换异常。Nav2/Bridge/Mission 仍按各自角色使用仿真时间。watchdog/服务/输出周期始终为 steady/wall；暂停、回退、跨零及负 Clock 均失效，不自动授权。

依据 rclcpp 28.1.22 的 rclcpp/src/rclcpp/time_source.cpp 实码：use_sim_time 可创建独立时钟 Executor/thread，set_clock 内构造 rclcpp::Time。CMR 明确校验 sec>=0、nanosec<1e9，再用有界 int64 算术转换；非法候选时间直接失效，初始 clock=0 保持禁止，运行后回零使 clock epoch 失效。不能让非法消息终止 Gate 后让下游保留非零。原始源码保存在本轮 evidence。

独立到达验收保持0.25m/0.25rad；新派生 Goal checker 收紧为0.18m/0.20rad，留出停止与观测余量，不能以 Action SUCCEEDED 替代到达。只对本轮配送机器人模型保留全部 pose 样本与 world stats；P0 默认采集不变，队列溢出必须判失败。

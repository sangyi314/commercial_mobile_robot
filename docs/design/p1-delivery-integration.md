# P1-04 集成设计与编码计划

2026-11-02；工程基准dca497335fe7fba8ee2e4825783411cd65a2cec9，工作树接收时干净。证据根evidence/env/p1-04-dev-sdkcx3w5。前三阶段新基准：纯37/37、A01–A20及审计20/20、E01–E16及审计16/16；P0离线470通过4失败5跳过31 ROS另测，与P1-03同名失败，不归咎本轮新代码。

## 单一业务状态机

Mission API/人工终端 → Runtime owner → Store事务（登记/FIFO/活动槽/事件/effect）→原reduce → ExecutionPort → Gate；已绑定SendAuthorized → 原NavigationAdapter → NavigateToPose。原Runner仍管理全部进程，worker只协调启动/观察，真实模式不创建其ActionClient。另一套配送FSM/仿真启动器被拒绝，避免两个Goal发送者和两份状态事实。

## 增量模块

1. delivery.hpp/.cpp：值对象/版本化站点解析、资源hash与纯到达窗口，不建Node，不写DB；系统nlohmann_json3.11.3（MIT）和OpenSSL哈希公共API，仅作解析/指纹，不引入业务框架。
2. 原Record/reduce：配送模式追加明确的成功/到达新鲜度/交接期限与token、总任务期限；保留v1记录解码和fixture行为。新的返程/重试仍走原转移。
3. 原Store/Runtime：可选schema2登记/队列/命令决定、原子升队与取消，迁移默认拒绝；全部写在原owner，无第二连接写者。
4. 原MissionNode/NavigationAdapter/ExecutionPort：新增显式配送配置与接口、冻结目标载荷、独立定位观测和owner驱动；合法收尾后持久Reset意图才允许Gate回DISARMED，Reset本身不授权。跨Goal身份/租约重建，绝不延续旧lease。
5. 原Runner/worker：显式模式、observe worker硬拒send/cancel；受监督Mission重启、版本化session记录与验证器。保留P0旧模式和旧单Goal结果合同。

## 上游研究与取舍

Nav2固定d5d768b7b38a05e7cfd2b6c693b786b402161bed：nav2_behavior_tree/plugins/control/recovery_node.cpp及test/plugins/control/test_recovery_node.cpp（Apache2）核对两子节点、重试次数和halt复位；WaitAction源码计内部恢复，不把Wait等同于业务retry。navigate_to_pose.cpp的goalReceived读取Goal.behavior_tree、onPreempt处理同BT、goal_pose回调有内部self Client，说明必须私有化未用目标入口、核对真实加载BT，不能只看默认路径。bt_action_node.hpp和本机rclcpp_action28.1.22接口用于异步生命周期参考。复用系统Nav2节点/BT插件，独立组装最小P1 XML，不复制源码系统。

SQLite官方lang_transaction与wal文档：保留BEGIN IMMEDIATE、WAL/FULL、有界BUSY；读者与writer职责明确，COMMIT才接纳，复制主DB不是WAL一致备份。Store扩展参考这些事务语义独立实现，不复制SQLite源码。来源和许可更新THIRD_PARTY/AI_USAGE。

## 测试与继续门

先纯站点/到达/交接/队列确定性反例及20seed×200交错，再真实Fake Action+Gate+synthetic sink D01–D18和原回归；任何关键错误不带入运动。随后受控G01–G06，G01等待用户本人交接。版本化manifest/events/result验证器从原始数据核对，不把汇总JSON当证明。A/B/D分离、C ACK未知、Gate自身死亡无独立保护继续保留。

当前仅接收/设计完成，尚未交付P1-04实现或运动验证。后续每模块结果追加到verification，不将计划改写成已运行。


## P1-04 停止证据修复 v1（2026-11-03，本人批准仅P1仿真Oracle辅助门控）

G03反例证明DiffDrive轮关节积分的odom不能单独支撑车体静止。正式simulation的StopReport必须为`reason=WINDOW_PROVEN_BODY_POSE`：原轮式窗全部通过后，以该精确from/to为窗口边界，再核对独立body反馈。BodyWindow使用包围边界的原始位姿样本，逐对计算平移速度/角速度，同时检查反馈twist、漂移、样本/接收间隔、GID与绑定身份；缺失、冲突、过期、重复/回退、跨generation均不合格。Gate与正式ExecutionPort拒绝旧轮式WINDOW_PROVEN。sink模式仍明确synthetic，只验证协议。所有原3s/0.01mps/0.03radps/0.02m/0.03rad/0.5s/200ms阈值不变。

数据源复用Gazebo Sim 8.15.0已安装OdometryPublisher插件：在派生model上读取worldPose，发布私有`<run>/body_odom`；官方Bridge仅GZ_TO_ROS。其odom_frame=body_world、child=body_base，私有TF不桥接到/tf。它不是轮关节积分；插件twist含滚动均值，因此额外直接计算位姿差分，不只信twist。此源仅供独立停止观察器，不供Nav2定位、目标、地图或速度规划，不证明实车能力。P0原SDF、Bridge、轮式odom和导航TF不变。

开源研究：Gazebo官方gz-sim tag gz-sim8_8.15.0，commit446a44335a45b704b4d36dabcc5508ee34eeb3d8，Apache-2.0；阅读DiffDrive.cc、OdometryPublisher.cc、实际integration test/world。直接复用安装插件，未复制上游系统。ros_gz tag1.0.24 commit5912bfe7976a79d2cc9f373b2d40a8a3a3e33da2，Apache-2.0，复用官方Odometry转换/Bridge。Pose_V→TFMessage转换不保留集合header给各pose，故不拿缺时间的桥接PoseV做停止源。原始源码、hash和缺失路径/403调查记录保存在evidence/env/p1-04-stopfix-ds6qdezr/upstream。CMR双流判定器为参考数据语义后独立实现；无额外框架。

本扩展已获本人明确授权，见`docs/adr/p1-oracle-stop-20261103.md`；记录必须标Oracle辅助，不属于sensor-only E2。


## P1-04 terminal heading profile v2（2026-11-03）

G05 r07真实MPPI返航停在map约(-2.072,-0.211)，home(-2,-0.5)，yaw接近0；距home仍约0.298m，位置验收未通过。随后SimpleProgressChecker报Failed to make progress，BT ClearLocal/ClearGlobal/Wait开始，候选过期使Gate安全锁止。保留r07失败，不追认。

研究Nav2固定commit d5d768b7b38a05e7cfd2b6c693b786b402161bed的goal_angle_critic.cpp、goal_critic.cpp、critics_tests.cpp和SimpleGoalChecker（Apache-2.0），并核对本机all-parameters.json。GoalAngleCritic默认半径0.5m、权重3；源码按当前机器人到goal距离启用朝向代价，测试明确0.3m时生效。差速返航需从南向进近再面向0rad，过早朝向可能形成局部极小。

仅派生MPPI配置显式GoalAngleCritic.threshold_to_consider=0.18，与既有位置验收半径一致；权重、速度/加速度、轨迹数、GoalChecker位置0.18m/朝向0.20rad、独立到达0.25m/0.25rad、全部停止阈值、期限及候选新鲜度不变。DWB配置不变。此参数是导航代价启用半径，不是放宽到达或安全验收。版本p104-terminal-heading-v2，必须新run_id验证；不把默认profile失败改成成功。直接复用已安装MPPI插件，只适配配置，没有复制第三方实现。

Nav2内部Wait未持续提供新鲜候选时，Gate仍按合同停止并锁存；本轮没有通过延长期限使该恢复继续。此限制须单独记录，不能声称内部Wait恢复已获运动放行保证。


## P1-04 terminal controller v3：Rotation Shim → MPPI（2026-11-03）

单项GoalAngle启用半径配置v2没有稳定解决问题，G05 r08再次真实无进展，保留失败；v2不作为最终方案。按具体问题复用本机Nav2 1.3.13官方RotationShimController，primary_controller明确为MPPI。它在位置GoalChecker的既有XY容差内接管终点转向，并可先原地对准新路径；MPPI仍负责路径跟踪。避免把位姿同时优化中的朝向局部极小当FSM/队列故障。

研究固定Nav2 commit d5d768b7b38a05e7cfd2b6c693b786b402161bed的nav2_rotation_shim_controller.cpp、test/test_shim_controller.cpp（lifecycle/computeVelocityGoalRotation/acceleration tests）、ControllerServer发布接口；Apache-2.0。直接复用安装插件，未复制控制器代码。ControllerServer最终覆写base frame及本次计算stamp，仍走唯一原速度链。

派生profile版本p104-mppi-rotation-shim-v3：官方Shim插件+primary MPPI，rotate_to_goal_heading=true，原地角速度0.8rad/s、最大角加速度2.0rad/s²与既有Nav2上限一致；primary GoalAngleCritic关闭，终点方向由Shim在既有位置容差内处理。位置0.18m/方向0.20rad、独立到达0.25m/0.25rad、停止、新鲜度和期限标准不变。DWB不变。运行图/参数及manifest必须如实标注组合，不能冒充裸MPPI对比结果。新增exec_depend为已安装的小型Nav2插件，没有引入YASMIN或新任务框架。

独立readiness和审计必须核对实际FollowPath.plugin、primary_controller及rotate_to_goal_heading，不仅相信profile或README。原始源码及本机libnav2_rotation_shim_controller.so/libmppi_controller.so SHA位于本轮upstream和shim-installed-identity.json。必须新的冷启动回合验证，不追认r07/r08失败。


### v3未通过与短段配置v4（仍需新回合验证）

G05 r09真实body在10.311s内移动0.39646m、平均速度0.04692m/s，原0.5m/10s进展条件未达到，原地旋转+短段低速未稳定完成；保留失败。没有降低进展半径或延长10s。源码进一步确认Shim每次新path默认重新接管，官方rotate_to_heading_once参数可仅对新Goal/正在旋转的路径激活；同时本任务0.9m短段一直处于PathFollowCritic默认1.4m停用范围，只剩默认权重5的GoalCritic主导距离推进。

v4沿用官方Shim→MPPI，设置rotate_to_heading_once=true，GoalCritic.cost_weight=20（距离目标的代价权重，不是任何通过阈值）。目的为减少1Hz重规划带来的重复转向，并提高短段位置推进目标的相对代价。Collision/Constraint、安全限值、GoalChecker、0.5m/10s进展条件及全部验收标准不变。必须新run_id验证，未通过不能宣布修复。

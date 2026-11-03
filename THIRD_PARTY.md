# 第三方来源
Nav2：官方 Jazzy 包与配置；保留原始文件、版本、hash 和原许可证。
起步 C++ 策略来自本项目执行指南，属于待审查的协作实现。
后续增加代码复用时逐项记录来源、许可证、修改和贡献边界。


## 2026-10-31 — P1 设计参考（未复制代码）

- Nav2 jazzy commit d5d768b7b38a05e7cfd2b6c693b786b402161bed：nav2_waypoint_follower/src/waypoint_follower.cpp、nav2_collision_monitor/src/collision_monitor_node.cpp；Apache-2.0；参考Action身份、生命周期与速度输出边界。
- YASMIN main commit abbe58d9511a79618023a530870f7d1794a829b7：yasmin/src/yasmin/state_machine.cpp、yasmin_ros/include/yasmin_ros/action_state.hpp；Apache-2.0；参考状态合法性和回调身份/生存期。
- twist_mux rolling commit a8225e2d9bf0171aac447c246376f0981defd17b：src/twist_mux.cpp、include/twist_mux/topic_handle.hpp；BSD-3-Clause；参考锁/优先级/超时，未引入运行依赖。
- SQLite官方C API/事务文档，核心public domain；计划使用Ubuntu已安装3.45.1，目前未链接或实现Store。

上述源码经GitHub读取并按commit复核blob一致；没有编译上游，不推定与本机Nav2二进制逐字一致。此次cmr_mission代码为独立协作实现，SPDX MIT与根许可证一致；cmr_core既有Apache元数据差异未修改。研究详表位于项目管理deliverables/Phase_1_开源源码研究_20261031.md。


## P1-01 S2–S6实现分类（2026-10-31）

直接链接：Ubuntu系统SQLite 3.45.1 C API（核心public domain；未拷贝amalgamation）。接口依据 https://www.sqlite.org/lang_transaction.html 、https://www.sqlite.org/c3ref/prepare.html 、https://www.sqlite.org/c3ref/step.html 。

参考设计后独立实现：mission.cpp参考先前固定YASMIN abbe58d9511a79618023a530870f7d1794a829b7 的yasmin/src/yasmin/state_machine.cpp状态合法性与分层思路，本轮经GitHub连接再次读取；未复制源码，未引入框架、全局信号处理或黑板。原YASMIN Apache-2.0身份保留在研究记录。

独立实现：CMR事务/活动槽/意图语义、RAII封装、Runtime、离线Port、C++/Python测试驱动与CLI，SPDX MIT。标准库线程/condition_variable和CMake/CTest为基础设施调用。Nav2/twist_mux仅保留研究参考，没有链接到此纯离线模块。


## 2026-10-31 — P1-02来源与复用分类

直接使用系统公共组件：Jazzy rclcpp/rclcpp_action 28.1.22（Apache-2.0）、nav2_msgs 1.3.13及rosidl生成工具/运行库；SQLite沿用系统3.45.1。没有拷贝第三方实现文件。新cmr_interfaces定义和CMR节点/适配/测试为MIT协作实现；不更改P0元数据。

参考设计后独立实现：Nav2 waypoint_follower.cpp，commit d5d768b7b38a05e7cfd2b6c693b786b402161bed（Apache-2.0），借鉴Goal UUID和资源管理，不照搬cancel_all；YASMIN action_state.hpp，abbe58d9511a79618023a530870f7d1794a829b7（Apache-2.0），借鉴晚接受/epoch/弱生命周期，不引入框架。两份本轮重读blob与既有研究一致。

ROS2官方测试参考：ros2/rclcpp 7c76a00345f304c7dd145975b872f7ec66a92c9d，rclcpp_action/test/test_client.cpp，blob4371f48a0e46537a4d8bd215d8c6833b620993de，Apache-2.0。实际接口以本机client.hpp/create_client.hpp/any_service_callback.hpp为准，已保存头文件指纹。固定上游源码不作为本机二进制逐字证明。

独立实现：MissionCommand延后响应、ActionCorrelation、CMR Port队列/单调期限、Fake控制屏障、Python监督器与独立审计器；未增加Waypoint Follower/YASMIN运行依赖。详见docs/design/p1-action-design.md。

P1-02测试监督器还直接调用系统rclpy与rosidl_runtime_py消息转换公共API，并使用rosgraph_msgs/Clock合成时钟；本机rclpy/client.py保留上游Apache-2.0声明，remove_pending_request仅作为公开接口调用，不复制实现。静态预检路径不导入这些ROS运行库。


## 2026-11-01 — P1-03 来源与复用分类

直接使用系统组件：Jazzy rclcpp 28.1.22、rclpy、rosidl、geometry/nav/rosgraph_msgs；Nav2 Smoother/Collision Monitor/Controller/Behavior 1.3.13；ros_gz_bridge 1.0.24；Gazebo Sim 8.15.0。均保留系统包及其许可证，不拷贝组件实现；CMR已有 cmr_core 轮速检查直接链接。Gazebo/ros_gz/Nav2 使用 Apache-2.0；完整许可证新增 third_party/licenses/Apache-2.0.txt。

参考设计后独立实现（MIT）：Gate租约/挑战/单线程线性化/StopWindow/ExecutionPort/Runner扩展/测试。twist_mux a8225e2d9bf0171aac447c246376f0981defd17b，src/twist_mux.cpp、include/twist_mux/topic_handle.hpp，BSD-3-Clause；只参考锁、来源及超时，不链接或复制源码。Nav2 d5d768b7b38a05e7cfd2b6c693b786b402161bed：nav2_collision_monitor/src/collision_monitor_node.cpp、test/collision_monitor_node_test.cpp，nav2_velocity_smoother/src/velocity_smoother.cpp，nav2_controller/src/controller_server.cpp，nav2_bringup/launch/navigation_launch.py，Apache-2.0；借鉴保留来源stamp、消息/生命周期与非组合launch设计。原TimedBehavior每插件创建TwistPublisher的接口在本机头文件复核。

配置适配：派生Nav2参数来源为项目冻结P0配置，Bridge来源为安装的Nav2 minimal TB3官方配置（Apache-2.0）；新profile逐字段记录原hash和派生hash。Navigation launch独立组装官方节点，移除docking。SDF只修改私有DiffDrive路由、world只添加SceneBroadcaster，不复制DiffDrive实现。上游 ros_gz 1.0.24 的 ros_gz_bridge/src/bridge_config.cpp 核实分离ROS/GZ端点；gz-sim8_8.15.0 的 src/systems/diff_drive/DiffDrive.cc 核实joint position里程计及无命令timeout，不能据此声称独立硬件保护。

原始研究文件：evidence/env/p1-03-dev-xehwla33/upstream；本机安装API及运行参数优先于固定上游。未使用YASMIN/twist_mux替换现有Mission，没有整套复制第三方系统。

P1-03 时间输入加固：另读 rclcpp tag28.1.22→commit3aa906a2c7ad13d1623b31a55731993f56538e72 的 rclcpp/src/rclcpp/time_source.cpp（Apache-2.0），确认隐式时钟线程及负时间转换异常边界。仅参考并配置公共 NodeOptions；CMR 时间字段验证与显式 Clock 处理独立实现，没有复制上游实现。原始源码保留在本轮 evidence。

P1-03 DWB配置核对：Navigation2固定d5d768b7b38a05e7cfd2b6c693b786b402161bed 的 nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp，Locus Robotics 2017 BSD-3-Clause，实际组件许可证区别于Nav2多数Apache-2.0文件。只研究并运行系统组件，未复制源码到CMR实现。确认FollowPath.xy_goal_tolerance由Critic独立读取后，将派生配置同步为0.18，与外层Goal checker一致；独立验收0.25不变。完整上游许可声明随原始研究文件保留。


## 2026-11-02 — P1-04 配送增量（实施中）

固定Nav2 d5d768b7b38a05e7cfd2b6c693b786b402161bed：RecoveryNode源码与test_recovery_node.cpp、WaitAction、NavigateToPoseNavigator、BtActionNode真实实现均经GitHub读取，完整版权/许可和hash见evidence/env/p1-04-dev-sdkcx3w5/upstream/index.json（Apache-2.0）。仅研究恢复预算、halt、BT选择、self-client和异步生命周期，不复制实现。P1新BT将复用系统Nav2插件，不启用未经观测支持的恢复Spin/BackUp。

SQLite官方 https://www.sqlite.org/lang_transaction.html 、https://www.sqlite.org/wal.html 用于原Store新增登记/FIFO/命令事务设计；仍直接调用系统3.45.1 C API（核心public domain），无amalgamation复制。新CMR业务逻辑为参考设计后独立MIT实现。

新增直接使用系统nlohmann_json3.11.3头文件公共API（MIT/Expat）、OpenSSL EVP SHA256公共API（系统OpenSSL3，Apache-2.0），用于有界严格JSON解析和资源指纹，不引入任务框架。系统包完整版权说明复制到third_party/licenses/nlohmann-json-3.11.3-COPYRIGHT.txt，未复制库实现到CMR源码。tf2_ros0.36.23作为已有ROS通信组件直接使用，TransformListener显式spin_thread=false，不创建第二Executor线程。

P1-04 map_io逐文件许可补充：固定Nav2 d5d768b7b38a05e7cfd2b6c693b786b402161bed的nav2_map_server/src/map_io.cpp为BSD-3-Clause（Rover Robotics 2019、Brian Gerkey 2010、Willow Garage 2008），不是泛称Apache。直接链接安装map_io公共API；CMR不复制其实现。原始声明完整保留在third_party/licenses/nav2-map-io-BSD-3-Clause.txt与本轮upstream源码。


P1-04 sdkcx3w5 离线审计补充：参考 SQLite 官方 URI immutable=1 与 WAL/SHM 生命周期说明（https://www.sqlite.org/uri.html 、https://www.sqlite.org/wal.html）。仅借鉴接口语义，未复制实现。闭合库审计持有现有 CMR owner 共享锁、拒绝非空 WAL/journal，使用系统 SQLite immutable 只读连接，显式关闭；不用于活动库/崩溃 WAL 恢复。增加原始证据逐字节不变及活动 owner 拒绝测试。停止 oracle 绑定实际 APPLIED Stop→Resolve 的身份/epoch/generation/精确时间窗，并增加轮式静止但车体移动反例；均为 AI 辅助独立实现，未改变运行阈值或追认旧失败。


## P1-04 停止证据修复 v1（2026-11-03）

G03反例证明DiffDrive轮关节积分的odom不能单独支撑车体静止。正式simulation的StopReport必须为`reason=WINDOW_PROVEN_BODY_POSE`：原轮式窗全部通过后，以该精确from/to为窗口边界，再核对独立body反馈。BodyWindow使用包围边界的原始位姿样本，逐对计算平移速度/角速度，同时检查反馈twist、漂移、样本/接收间隔、GID与绑定身份；缺失、冲突、过期、重复/回退、跨generation均不合格。Gate与正式ExecutionPort拒绝旧轮式WINDOW_PROVEN。sink模式仍明确synthetic，只验证协议。所有原3s/0.01mps/0.03radps/0.02m/0.03rad/0.5s/200ms阈值不变。

数据源复用Gazebo Sim 8.15.0已安装OdometryPublisher插件：在派生model上读取worldPose，发布私有`<run>/body_odom`；官方Bridge仅GZ_TO_ROS。其odom_frame=body_world、child=body_base，私有TF不桥接到/tf。它不是轮关节积分；插件twist含滚动均值，因此额外直接计算位姿差分，不只信twist。此源仅供独立停止观察器，不供Nav2定位、目标、地图或速度规划，不证明实车能力。P0原SDF、Bridge、轮式odom和导航TF不变。

开源研究：Gazebo官方gz-sim tag gz-sim8_8.15.0，commit446a44335a45b704b4d36dabcc5508ee34eeb3d8，Apache-2.0；阅读DiffDrive.cc、OdometryPublisher.cc、实际integration test/world。直接复用安装插件，未复制上游系统。ros_gz tag1.0.24 commit5912bfe7976a79d2cc9f373b2d40a8a3a3e33da2，Apache-2.0，复用官方Odometry转换/Bridge。Pose_V→TFMessage转换不保留集合header给各pose，故不拿缺时间的桥接PoseV做停止源。原始源码、hash和缺失路径/403调查记录保存在evidence/env/p1-04-stopfix-ds6qdezr/upstream。CMR双流判定器为参考数据语义后独立实现；无额外框架。


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


P1-04最终检查点（2026-11-03）：实际采用p104-mppi-short-leg-v4（已安装Nav2 Rotation Shim→MPPI，once=true、GoalCritic权重20、primary GoalAngle关闭）及本人批准的Oracle辅助停车。G02 r07、G03/G04 r11、G05/G06 r10独立审计5/5通过；v2/v3及此前失败全部保留，G01本人未执行，P1-04仍PARTIAL。具体源代码、复用、参数与证据见docs/verification/p1-04-stopfix-ds6qdezr.md。未复制第三方控制器源码进工程运行模块。

## P1-05可靠性验证（2026-11-03）

复读实际ROS28.1.22 single_threaded_executor.hpp（Apache-2.0）、Nav2 1.3.13安装BT、既有固定d5d768b7b38a05e7cfd2b6c693b786b402161bed源码和测试、SQLite官方wal.html。CMR可靠性计划/资源采样/独立模型/驱动/流式审计为独立MIT实现，复用既有CMR与已安装ROS/Nav2/Gazebo/system SQLite，不复制第三方源码、不新增系统框架。原上游版权和许可证文件全部保留。证据见evidence/env/p1-05-dev-b1sg_5gd/research.md。元数据兼容性另审计，候选不等于正式发布许可放行。

P1-05增补：AI独立实现RenewLease规范effect序号解析和每attempt单调回执，保留原一次性effect去重及上限；修复Delivery service默认QoS10与11并发不匹配，采用本机Jazzy公共Services QoS接口，显式depth64与既有pending64一致。保留原D06失败、5000回执/格式反例及新候选实际协议/L2重测；无上游源码复制。

P1-05 r07录制修复：本机rosbag2/ros2bag/mcap_vendor0.26.11，官方ros2/rosbag2 tag0.26.11 commit cb8fa198b93bfed90e2417b52b8645fb9ef89c2b（Apache2），研究SequentialWriter、mcap_storage.cpp的YAML转换及实际test_mcap_storage.cpp。默认ROS双缓存100MiB和MCAP chunk使文件mtime不代表订阅健康；仅profile5显式max-cache-size=0、MCAP noChunking=true、enableDataCRC=true，不降低三秒文件进展门槛，P0及profile4录制不变。数据CRC保留，文件可见增长不等于掉电fsync保证。短测r06失败与原MCAP保留；新候选重跑。窗口失败也单独记录已开始/中止和真实steady时长，不拼接。成熟组件直接使用，CMR参数和监控独立适配，无上游实现复制。源码/测试/许可在evidence/env/p1-05-dev-b1sg_5gd/upstream-recording-r07。

MCAP传递组件核对：rosbag2 mcap_vendor0.26.11的CMake实际取foxglove/mcap releases/cpp/v1.3.1，核心MIT而非笼统Apache-2.0；保留third_party/licenses/mcap-1.3.1-MIT.txt及官方tag核对证据。本项目直接调用安装rosbag2插件，没有拷贝MCAP实现或将系统库改许可。P0 cmr_core/cmr_sim包元数据Apache-2.0与根MIT声明的范围仍待作者审查；历史P0元数据不自动改写，正式发布阻断，见本轮license-metadata-audit.json。

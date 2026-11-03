# AI 使用与个人所有权
允许 AI 协作生成代码、配置和测试。
模型、假设、接口、oracle、验收、结果解释由项目作者负责。
核心模块合并前必须能解释关键分支并独立修改。
每次变更记录：问题、AI 辅助范围、人工审查、测试、局限。


## 2026-10-20 — P0-01 增量测试与证据合同（审查草稿）

- 问题：当前五组简版源码与历史扩展覆盖描述不一致，须以现有源码补实际缺口。
- AI 辅助范围：远程只读核查与基准证据复核，五组内增加 P01–P07/A01–A06/N01–N02/W01–W07/G01–G10 覆盖草稿，编写接口合同、案例定位与复核附记，保存差异和草稿指纹；生产 API 未改。
- 人工审查：尚未收到负责人审阅意见，不将个人理解或掌握自动标通过。
- 测试：核验既有 `p0-01-gfGjKp8O` 修改前 Release 5/5 的文件指纹与日志，本次未重跑基准；新增草稿未编译/执行，步骤 7–10 待审查后依次运行。
- 局限：wheel_rates 极端有限输入溢出、时间差合理范围、无主动停车/会话恢复/线程保护；未运行 ROS/Nav2/Gazebo/实车。证据与审查差异见 [本步报告](docs/verification/p0-01-gfGjKp8O-draft.md)。


## 2026-10-31 — P1 五阶段指南与纯准入切片

AI承担只读现状调查、上游六文件固定commit研究、五份指南和共享合同、请求/准入纯C++及八用例。实际Ubuntu Release测试8/8、既有核心回归5/5。没有完整FSM、SQLite、ROS或仿真；没有提交/发布。个人理解、合同最终接受和核心合并审查待本人完成。详见[首步报告](docs/verification/p1-01-admission-20261031.md)。


## 2026-10-31 — P1-01 S2–S6离线实施

AI独立编写轻量FSM、系统SQLite封装、owner邮箱、离线Port/屏障/测试和文档。用户提供合同、任务书与使用边界；本人理解/验收尚未确认。没有复制第三方系统；实际命令、失败修复和验证范围见docs/verification/p1-01-complete-20261031.md。


## 2026-10-31 — P1-01取消意图覆盖修复

AI在用户授权下修复pause中首次终止意图，新增纯FSM/真实Runtime-Store/并发屏障三组回归；修复前3组失败，修复后Release与ASan/UBSan各16/16，原反例Send由2变1。未改合同/schema/第三方依赖，未迁移历史DB，无ROS或Git提交。个人验收待本人；详见docs/verification/p1-01-cancel-intent-fix-20261031.md。


## 2026-10-31 — P1-02完整工程实施

用户提供任务书、合同与无Gazebo/真实目标边界；AI负责上游实际源码/本机API核查、接口、薄适配、Fake、监督器、矩阵、故障排查与六模块教学文档。未重写Mission状态机；增量扩展导航CanceledResult和事件审计。个人理解、反例设计与正式放行仍待用户验收。

原P1-01 16组基准重跑；当前OFF核心/关联17组及ASan/UBSan通过，ON colcon17组通过；真实ROS A01–A20分层结果和13案instrumented协议结果见docs/verification/p1-02-action-20261031.md。A11非法线协议只按共用适配边界单元报告。TSan空程序运行时也失败，P0布局23通过1冻结系统库哈希失败均保留，不宣称全仓库绿色。未commit/push/tag/release，未启动Gazebo/Bridge/Nav2栈或真实Gate。


## 2026-11-01 — P1-03 工程实施

用户授权全范围实施、逐步验证与受控Gazebo，要求保护P0、保留失败、不自动Git提交。AI完成基准重跑、固定上游/本机API核对、Gate/Observer/接口/真实ExecutionPort、独立分析器、原Runner的可选派生扩展、测试与教学文档。所有工程改动在Ubuntu原仓库，无Mac工程副本。纯核/协议/仿真及sanitizer结果以 docs/verification/p1-03-un63b3ef.md 的实际证据为准；真实Mission sink与授权fixture Gazebo分列。

AI辅助不替代用户理解、人工代码审查和实际机器人安全放行；未作硬件认证，未进入P1-04，未commit/push/tag/release。失败及NOT_EXERCISED不得删除或改判。


## 2026-11-02 — P1-04 连续实施（未验收）

用户授权完整配送集成、有限FIFO与受控仿真，G01本人交接不得代签。AI已完成前置实查与新基准、固定上游研究、集成/队列合同、原FSM/Store/Runtime可选配送扩展、站点/到达纯逻辑、原Action/ExecutionPort跨段授权及真实定位监测、隔离协议驱动。工作仍在Ubuntu原仓库，原始失败保留；没有Git自动提交。当前结果和未执行项以docs/verification/p1-04-sdkcx3w5.md为准，不将实施中写成阶段通过。

P1-04续行：AI实现原Runner显式Mission模式、业务编排、独立闭合MCAP复算与只读汇总、真实进程崩溃负例和模块教学指南。自动测试中的交接标记synthetic；本人G01仍必须外部用户亲自调用，AI不得调用human。测试超时/原P0失败与后续修复日志分别保留。


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

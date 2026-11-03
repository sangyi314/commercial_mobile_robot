# P1-04 最终工程检查点 — stopfix-ds6qdezr

2026-11-03（Asia/Shanghai）。**PARTIAL：G02–G06工程验收通过；G01本人交接未执行，所以不能宣布P1-04全部通过。** 本人已说明“暂时不能，先完成其他验证”。没有代发human确认，没有进入P1-05。

## 交付与环境

实际工程仅在Ubuntu `/home/robot/commercial_mobile_robot`，通过`ssh cmr-ubuntu`实施。HEAD仍为`dca497335fe7fba8ee2e4825783411cd65a2cec9`，全部代码保留在工作树，未commit/push/tag/release。Mac仅保存说明文档副本，不是工程副本。

本轮证据根 `/home/robot/commercial_mobile_robot/evidence/env/p1-04-stopfix-ds6qdezr`（以下记E）。之前完整实施与原始失败位于`evidence/env/p1-04-dev-sdkcx3w5`及对应`evidence/missions`，没有覆盖。ROS Jazzy ARM64，rclcpp/action28.1.22、Nav2 1.3.13、Gazebo8.15.0、ros_gz1.0.24、系统SQLite3.45.1。编译C++17，保留ROS OFF/ON模式。

主要交付：

- 原`src/cmr_mission` FSM/Store/Runtime、NavigationAdapter、ExecutionPort的真实配送集成；`DeliveryCommand.srv`、站点快照、到达连续窗、有限FIFO与有界恢复。
- `src/cmr_execution/include/cmr_execution/stop.hpp`新增BodyWindow；`stop_observer.cpp`双来源窗口；正式Gate/Port拒绝轮式单来源STOPPED。
- `tools/delivery/`业务编排、派生profile与独立记录审计；`tools/mission/` CLI/Fake协议监督器/汇总；原Runner负责全部生命周期，worker观察模式不创建ActionClient。
- `docs/contracts/p1-runner-v1.md`、`p1-queue-v1.md`、扩展的`p1-stop-evidence-v1.md`，集成设计、模块学习指南及本人G01操作说明。

## 已批准的Oracle边界

总纲原有真值仅供独立评价器。已就“停车评价结果影响任务放槽及下一次授权”的具体扩展征求本人决定，得到明确答复：**“允许仅用于仿真停车资格，明确标为 Oracle 辅助门控”。** 见`docs/adr/p1-oracle-stop-20261103.md`和E/truth-boundary-decision.json。

派生机器人使用已安装官方OdometryPublisher读取worldPose，私有body_odom经单向Bridge仅进入独立停止评价器与只读记录器。它不进入Nav2定位、规划、目标或速度计算；不能将本轮称为sensor-only E2或实车制动证明。profile、manifest、result、bag与运行图均明确`oracle_stop_assisted=true`。运行图拒绝Nav2/Mission直接订阅body反馈。

## 原阻塞如何修复

旧G03实际消费的轮式窗口18.828–21.852s中，独立车体在18.849s速度0.01086448m/s，超过0.01m/s。新C++判定器回放旧原始车体179个样本，即使把报告twist设零，也判`MOVING BODY_POSE_VELOCITY`（E/original-g03-cpp-replay.json）；没有修改旧记录或阈值。

现在原轮式StopWindow先合格，再由BodyWindow对同一精确from/to的包围样本逐对计算位姿速度/角速度，并同时检查反馈twist、漂移、缺样、接收新鲜度、时钟与generation。缺失/冲突保持STOP_UNKNOWN。正式Gate/ExecutionPort只接受`WINDOW_PROVEN_BODY_POSE`；domain72的sink_body始终synthetic=true。

停止仍保持3s、至少5点、源间隔≤0.5s、接收新鲜度<200ms、v≤0.01m/s、w≤0.03rad/s、漂移≤0.02m/0.03rad。Gate tick20ms、lease300ms及候选/时钟限制未放宽。取消ACK、Goal终态、到达、授权撤销、零输出和车体静止分别评价。

独立oracle从SQLite APPLIED Stop→Resolve绑定实际消费的身份/UUID/epoch/generation及窗口，不挑后续合格样本。G06没有业务消费Stop，须绑定实际LEASE_EXPIRED代次、故障标记和重启前停止标记，不能复用初始静止。GateState用自带ros_ns；不同DDS writer的录包到达行序不当因果屏障；Reset在窗口后发生不回写先前事实。独立worker的steady间隙、闭合MCAP、另一条Gazebo原始body pose分别交叉核对。

## 最终导航配置与原始失败

G05裸MPPI与两种中间配置曾实际无进展，原0.5m/10s进展条件与安全期限均未改。研究官方源码、真实参数与测试后，最终`p104-mppi-short-leg-v4`采用**官方RotationShimController→primary MPPI**：位置容差内由Shim处理终点朝向，仅新目标做起始对齐；GoalCritic权重20，primary GoalAngleCritic关闭。保持Nav2位置0.18m/方向0.20rad、独立到达0.25m/0.25rad、速度0.18m/s/0.8rad/s及角加速度2.0rad/s²。DWB保持原派生配置。

这是具体控制组合，不能冒充裸MPPI比较。实际FollowPath.plugin/primary_controller/rotate_to_goal_heading/rotate_to_heading_once/GoalCritic权重由readiness及独立审计核对；manifest如实记录。G05 r10在一个会话完成两单四Goal；G03/G04随后用同一最终组合重新验证。

Nav2内部Wait恢复不提供新鲜候选时，Gate仍按合同锁止；没有延长新鲜度让它继续。本轮不声称这种内部Wait已实现无缝运动恢复。外层NAV_ABORTED白名单和一次共享重试在协议矩阵验证；真实持续占道注入仍NOT_EXERCISED。

## G矩阵（真实ROS/Gazebo，Oracle辅助停车）

全部会话在`/home/robot/commercial_mobile_robot/evidence/missions/<run_id>`，冷启动、独立录制与归属清理。下表“业务状态”是预期场景结果；G03/G04/G06测试通过不等于配送COMPLETED。

| ID | 最终采纳回合 | 持久业务状态 | Goal数 | 精确停止窗数 | Runner / 独立审计 |
|---|---|---|---:|---:|---|
| G02 | `p104_20261103_g02_r07` | COMPLETED | 2 | 2 | PASS / PASS |
| G03 | `p104_20261103_g03_r11` | CANCELED | 1 | 1 | PASS / PASS |
| G04 | `p104_20261103_g04_r11` | FAILED | 1 | 2 | PASS / PASS |
| G05 | `p104_20261103_g05_r10` | COMPLETED | 4 | 4 | PASS / PASS |
| G06 | `p104_20261103_g06_r10` | PAUSED | 1 | 1 | PASS / PASS |

G01：**NOT_EXECUTED / HUMAN_PENDING**。未创建其运行目录，未伪造本人确认。准备好的最终profile为E/inputs-r10/G01；完整两终端步骤见`docs/guides/p1-04-human-handover.md`。

G02 DWB没有被MPPI组合改动影响，沿用r07已通过配置；G03 r11/G04 r11/G05 r10/G06 r10为最终MPPI+Shim组合。每个窗口的body反馈与独立世界位姿均通过，C层仍UNKNOWN。最终五会话的闭合重新审计为 **5/5**（E/final-selected-summary-v2.json），首次准入7个订单、持久Submit调用27次，原生Goal共9个。重复Submit没有计成新配送。

G03 r11实际移动后受控取消；worker同一steady时钟观测到授权禁止延迟约8.38ms。G06真实owner自有PID被监督SIGKILL后，约296ms观测LEASE_EXPIRED；独立车体静止窗口在重启前成立，重启保留PAUSED/UNCERTAIN、一个未决Goal及一项WAITING，没有自动重发/升队。这些是本回合观察结果，不是硬实时上限。

本轮真实仿真共12回合：**7个原始会话通过、5个失败完整保留**。最终矩阵选定上述5个通过回合；其余通过是配置回归，不能删掉失败来汇总成功率：

- G03 r06：收尾oracle局部变量遮蔽函数导致TypeError；同时强化了故障前body位移资格，旧故障时body位移尚未达到0.05m。没有追认该轮通过。
- G05 r07：裸MPPI返航无进展，BT等待期间候选过期，安全失败，已交接事实保留。
- G05 r08：终点朝向启用半径v2未解决无进展，失败保留，未作为最终方案。
- G05 r09：Shim初版在10.311s仅移动0.39646m，仍未达到原进展要求，失败保留。
- G03 r10：车体停止合格，但oracle错误要求跨topic的报告行序屏障，A/B被拒绝；修复源时间关联后另存重分析，仍不改旧失败会话；r11冷启动重测通过。

完整列表E/final-trial-history.json；早前sdkcx3w5回合、编译告警、ASan首次找不到cmr_interfaces、GitHub403/缺失路径、回放准备字符串sec错误等原日志/说明分别保留。没有关闭检查或提高超时追认。

## 构建、回归与边界

| 层级 | 实际结果 | 原始证据 |
|---|---:|---|
| Release纯Mission OFF | 36/36 | E/mission-off-final.xml、mission-off-final-commands.json |
| Release ROS ON Mission | 37/37 | E/cmr_mission-release.xml、release-commands.json |
| Release Gate/Core | Gate15/15、core5/5 | E/execution-release-final.xml、cmr_core-release.xml |
| ASan/UBSan Gate纯核 | 15/15，assert/check保留 | E/asan-execution-r02.xml、sanitizer-r02-commands.json |
| ASan/UBSan ROS停止观察器 | 1/1；detect_leaks=1；移动/静止/停流/有界SIGINT退出 | E/asan-body-r02.xml及tmp原始日志 |
| 全部默认Python | 492通过、4失败、5跳过、33 ROS另测 | E/pytest-source-time-final.log/xml |
| 隔离ROS回归 | 33/33 | E/final-ros.log/xml（同最终worker；后续oracle纯时间关联由新单测及实际MCAP覆盖） |
| P1-02回归 | A01–A20 20/20，独立审计通过 | E/action-regression、action-regression-audit.json |
| P1-03回归 | E01–E16 16/16，独立审计通过 | E/execution-regression、execution-regression-audit.json |
| P1-04协议 | D01–D18 18/18；22个子库独立审计22/22 | E/delivery-full-r01、delivery-independent-audit.json |
| D06额外真实并发 | 11个请求同时在途；容量/幂等/冲突合格，2个子库审计通过 | E/D06-concurrent-final、D06-concurrent-final-audit.json |
| 队列/交接固定seed | 两组各20seed×200事件，全部保留 | E/ros-build/cmr_mission/delivery-seeds及delivery-handover_seeds |
| 六个真实进程崩溃窗口 | submit/promote/handover各提交前后_exit(86)，回归通过 | 同build下delivery-crash_windows及CTest原始日志 |

4个Python失败与接收基线同名：旧动态cohort依赖身份漂移、3个旧crossing路径检查；不修改历史输入/锁让它们变绿。P0影响检查不是全绿。前轮Mission纯核ASan36/36等结果见sdkcx3w5原日志；本轮未做全量Mission ROS Action/Gazebo sanitizer，不能把观察器ASan当全部异步对象无缺陷证明。

## D01–D18触发、依据与结果

全部协议使用真实NavigateToPose类型、实际Mission/Store/Port/Gate、Fake Server；运动/TF/地图输入明确synthetic，不能冒充Gazebo结果。表中目录相对E/delivery-full-r01，额外目录另列。suite根保留supervisor-source.py和protocol-base-source.py；每个子回合含trace.json、native logs、mission.db和cleanup；SQL/原生日志由独立工具重推。屏障竞态/崩溃窗口及seed是纯核/进程层，层级分列。

| ID | 前置/注入与预期 | 独立依据与补充 | 实际 |
|---|---|---|---|
| D01 | 活动任务正常去返程，两个不同UUID/lease；重复交接不增Send | D01 native logs/trace/DB + G02/G05实际运动 | PASS（声明层） |
| D02 | Action success同时关闭TF/odom | D02 trace、PAUSED及未交接DB | PASS（声明层） |
| D03 | 去程/返程受控取消 | D03及return_cancel子库；G03 r11 | PASS（声明层） |
| D04 | 错误任务/token/version及交接期限 | D04；确定性交接/取消屏障；G04 r11 | PASS（声明层） |
| D05 | 交接后返航ABORTED两次，预算耗尽 | D05，handed_over=true，3次总Send | PASS（声明层） |
| D06 | 活动槽+有限队列；重复/冲突/满队；11个同时在途服务请求 | D06 + D06-concurrent-final（原日志/两个独立DB审计） | PASS（声明层） |
| D07 | 取消等待项；取消与升队两个确定顺序 | D07协议；delivery_store_test.races真实owner屏障 | PASS（声明层） |
| D08 | 首单完整完成后提升下一单 | D08；G05 r10两单4个Goal | PASS（声明层） |
| D09 | 真实自有Mission进程SIGKILL后监督重启 | D09；六个真实_exit(86)事务窗口；G06 r10 | PASS（声明层） |
| D10 | 真实owner SIGSTOP，Gate失效；恢复后不续命 | D10 synthetic协议；G06真实owner退出 | PASS（声明层） |
| D11 | 拒绝、晚接受、接受未知、server退出/重启 | D11/accept_unknown/server_loss，发送未知不重放 | PASS（声明层） |
| D12 | 真实Revoke控制与停止反馈停流 | D12/stop_stream_missing；G03/G06与sink_body停流 | PASS（声明层） |
| D13 | 错地图/站点/站点版本、队首资源hash改变 | D13，0个Goal，等待项保留并暂停 | PASS（声明层） |
| D14 | 共享一次额外attempt，去返程不可各自重置 | D14 Fake ABORTED；真实持续障碍注入NOT_EXERCISED | PASS（声明层） |
| D15 | 健康资格后注入第二Goal client/禁止节点 | D15 trace；派生Bridge/SDF/插件/记录负例与E08 | PASS（声明层） |
| D16 | 错hash/缺记录/坏schema、自有子进程exit7 | D16负例不抹去COMPLETED；闭合审计拒绝活动owner | PASS（声明层） |
| D17 | 电量无来源、未实现低电服务 | D17 unavailable；不伪称真实低电回航 | PASS（声明层） |
| D18 | 早取消、未决恢复拒绝、新单不插队、失败后暂停 | D18；持久FIFO及20×200事件；G06 waiting保留 | PASS（声明层） |

D14真实持续障碍注入NOT_EXERCISED；曾实测无进展→Nav2 Clear/Wait→Gate失效，但不是受控障碍恢复成功。D15实际第二Goal client/禁止节点与静态路由/lifecycle约束分列，没有在运动中制造旁路。D17电量unavailable。G01本人操作未执行。以上没有用“矩阵PASS”覆盖各层缺项。

## 开源来源、复用与自研

- [Gazebo Sim8.15.0固定源码](https://github.com/gazebosim/gz-sim/tree/446a44335a45b704b4d36dabcc5508ee34eeb3d8)：DiffDrive轮关节积分、OdometryPublisher worldPose、integration测试与SDF；Apache-2.0。直接复用安装OdometryPublisher，CMR双来源窗口独立实现。
- [Nav2固定源码](https://github.com/ros-navigation/navigation2/tree/d5d768b7b38a05e7cfd2b6c693b786b402161bed)：Goal/GoalAngleCritic及测试、RotationShim及test_shim_controller、ControllerServer消息发布；Apache-2.0。直接复用安装Shim/MPPI；独立适配profile/验证，没有拷贝第三方框架。lib SHA见E/shim-installed-identity.json。
- [ros_gz1.0.24固定源码](https://github.com/gazebosim/ros_gz/tree/5912bfe7976a79d2cc9f373b2d40a8a3a3e33da2)：官方Bridge转换；Apache-2.0。复用安装组件。Pose_V集合时间不自动传入每项TF，未采用缺时间的PoseV桥接作停车源。
- [SQLite URI](https://www.sqlite.org/uri.html)和[WAL生命周期](https://www.sqlite.org/wal.html)：闭合库reader排除活动owner和非空日志后使用immutable，显式关闭；不删除WAL。原系统SQLite C事务/单写者复用，未引入ORM或并行任务框架。

实际读取源码、SHA、缺失路径和license均在E/upstream；THIRD_PARTY.md及AI_USAGE.md已更新。CMR FSM/Store/FIFO、Gate/BodyWindow、adapter适配与验证器为独立实现；上游设计参考与安装组件复用明确分开。

## 原始证据与复现入口

- 最终会话汇总：E/final-selected-summary-v2.json、final-selected-facts-v2.json，逐会话audit保留。
- 本人G01：`docs/guides/p1-04-human-handover.md`，只读check、原Runner冷启动、本人真实服务确认、独立验收四步；没有自动human调用。
- 源码调用关系和模块五问：`docs/guides/delivery-mission.md`，含构建/CLI/异步/所有权/取消/乱序/持久化/外部副作用窗口。
- 构建原命令：E/ros-build-command.json、release-commands.json、mission-off-final-commands.json、sanitizer-r02-commands.json。重建必须指定新的build/install/log根，避免改变冻结二进制。
- 最终代码包/差异/指纹：E/final-worktree-source-v2.tar.gz、final-tracked-v2.patch、final-source-manifest-v2.json；未自动提交。

## 保护、限制与退出条件

E/protection-final.json核对138个已有实验/仿真/bringup/fixture输入均未变，基准文件无缺失；没有未知进程清理、第二Gazebo或本轮进程残留。P0四项原有失败保留。所有测试数据库、构建、profile和日志在独立目录；旧evidence不改写。

C消费者ACK仍UNKNOWN；Gate自身崩溃/主机失效/Bridge断链/下游保持末值缺少独立硬件保护。名称/GID/namespace不是恶意节点隔离。worldPose用于本阶段获准的Oracle停车资格，不证明实机、安全认证、碰撞为零或持续障碍恢复能力。collision=null；全量Mission ROS sanitizer、真实低电和实物验证尚未执行。

当前必需的最后验收项是**G01本人真实交接并完成返航、记录及清理**。完成后再合并该会话审计，方可更新P1-04完整结论。当前不自动开始P1-05。

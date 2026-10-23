# P0-05 单回合 runner 实施与真实验收报告

交付日期：2026-10-25，Asia/Shanghai。执行主机：Ubuntu ARM64 cmr-ubuntu。
实施仓库：/home/robot/commercial_mobile_robot。

**工程验收结论：通过本次限定范围的 P0-05。MPPI、DWB 各一个新正常回合完整通过；缺初值显式受控失败回合按预期失败并正常清理。个人理解/RViz 目视及独立碰撞评价仍未确认。**

## 1. 实际起点和保护

- 起止 HEAD 均为 def8f2fd3f09e70d0c1aa98f0b1717f482d77710。Ubuntu 仓库及祖先没有适用 AGENTS.md；Mac 镜像 sources 为只读，实施未在镜像或其他机器人仓库进行。
- 起始 Git 状态只有既存未跟踪目录 evidence/env/p0-05-dev-yJ6gR8aZ/，保留原样。本次未 commit、push、reset、clean。
- 核对并继承已完成 P0-04：evidence/runs/p0-04-mppi-hhmvi_i0、evidence/runs/p0-04-dwb-76fqnpi5、evidence/env/p0-04-pair-q2MXrdh3。原两轮 real、现有 validator VALID/0、Action 和 cleanup SUCCEEDED；没有把指南旧快照当作 DWB 未完成。
- 开发目录：/home/robot/commercial_mobile_robot/evidence/env/p0-05-dev-bmzezluh。
- before.json 保存 65 份保护文件、1454 份既有 evidence 文件指纹。protection-final.json 核对全部仍存在且指纹相同；HEAD 未变，git diff --check=0，受跟踪文件无差异。cmr_core、三份原基线 YAML、旧启动器、通信环境、Schema、validator 均未修改。
- 这些核对证明本次起点文件保持不变，不能证明更早历史从未删除记录。
- VM UTC 与客户端日期有偏差，保留原始时间，没有改系统时钟。monotonic 用于期限，header/bag 用 ROS 仿真时间，各自标注，不互相冒充。
- 安装包版本与 profile 逐项核对一致，见 installed-versions-final.json；Nav2 1.3.13、rosbag2_py/MCAP 0.26.11、rclpy 7.1.12，没有升级或重调控制器。

## 2. 新增文件

以下 18 份源码/配置/文档均为本次新增并增量完善。原受保护文件没有修改。

| 相对仓库路径 | 职责 |
|---|---|
| tools/runner_core.py | 纯状态归约、嵌套期限、首因/后续失败、严格 JSON、原子且不覆盖的结果写入 |
| tools/runner_processes.py | 项目锁、Linux PID/start ticks/PGID/SID/独立归属标记、后代检查、有界幂等清理 |
| tools/runner_analysis.py | 从原始样本复算位移、定位估计误差、三段最终命令、连续停止 |
| tools/run_trial.py | 父进程唯一 spawn；早期 manifest、输入/源码快照、监督、清理、结果校验 |
| tools/nav_goal.py | 独立 ROS worker；真实生命周期/参数/TF/传感器/录制门，异步单 UUID Action、心跳与观测 |
| tools/inspect_bag.py | 只读 SequentialReader 反序列化 MCAP、覆盖/时间/frame/固定 TF/同 UUID 终态与停止核验 |
| tools/summarize_runs.py | 只读枚举缺失/坏合同/fixture/真实失败，不修结果、不以 VALID 计成功 |
| experiments/runner_profile.yaml | 继承 P0-04 资源、插件映射、DDS、Boost/Gazebo 修正、预算及门槛 |
| experiments/recording_qos.yaml | 12 话题录制 QoS：静态 TF/map/status transient_local，传感器 best_effort |
| experiments/scenarios/p0_05_official_static.yaml | 独立合法 draft 静态执行输入，限定原短任务，不伪改 ready |
| tests/test_runner_lifecycle.py | 37 项：状态/期限竞争、未知值、原子写、停止负例、预检无子进程、录制失败保留终态 |
| tests/test_runner_processes.py | 12 项：锁、信号升级、leader 退出/改 SID 后代、PID 复用、sentinel、大日志、IPC/心跳 |
| tests/test_runner_summary.py | 1 项：缺记录/坏记录仍枚举且输入不变 |
| tests/test_runner_bag_content.py | 6 项：精确合成 MCAP、缺静态边/错 frame/缺终态/损坏/缺 metadata |
| tests/test_runner_ros_contract.py | 22 项：domain72 隔离 Action/生命周期/clock/TF/录制和真实安装定位组件 |
| docs/runner_contract.md | 实现前冻结责任、状态、协议、预算及范围 |
| docs/runner_usage.md | 已验证命令、退出码、只读预检/检查/汇总、编号冲突规则 |
| docs/verification/p0-05-bmzezluh.md | 本报告：真实验收、失败索引、保护核对与限制 |

另新增 8 个真实回合目录及开发证据。每轮早写 manifest，dirty 源码真实打包，关闭输入后固定 hash。final-source.json 证明三个要求回合执行的六份 runner/worker/analysis/inspection 快照与当前源码一致；最后只补测试，没有改已实测执行代码。

## 3. 设计和代码审查

design-code-review.md 与 docs/runner_contract.md 在真实仿真之前完成。纯层不 import rclpy，父进程唯一 spawn，worker 无 launch/子进程。JSONL IPC 有 run_id/序号/请求号/大小限制、非阻塞读写、重复请求去重；心跳不等同 READY。

工作360秒、累计启动180秒、目标总计120秒含接受5秒、取消额外10秒；局部等待取父期限较小值。停止最多30墙钟秒，要求推进的仿真窗口；清理60秒、bag/最终写入30秒独立预算。截止后真实 SUCCEEDED 保留，harness 按现有合同为 INTERRUPTED，首因保留竞争，不伪改终态。

自有进程有 PID/start ticks/会话/独立归属标记。正常对 launch leader 一次 SIGINT，后代单独检查，强制升级即 cleanup FAILED；外来 ROS owner/项目锁冲突拒绝，不接管、不全局 pkill。正常清理顺序：完成停止观察，导航 manager SHUTDOWN/退出，录制结束/flush，再定位/bridge/RSP/Gazebo，worker 最后退出。

STARTUP/SHUTDOWN 不自动重发；GetState/is_active 为有界只读查询，每次2秒子期限，迟到响应归档。本轮缓存 Client，STARTUP前创建、SHUTDOWN复用，记录SERVICE_REQUEST/RESPONSE。目标唯一非零 UUID 先持久化再异步发送一次；取消按此 UUID，取消响应不冒充终态，未获接受响应不能填 REJECTED。

当前原仓库没有 .github/workflows，本次未创建远程 CI；以下测试实际在 Ubuntu 执行，不能说远程 CI 已运行。SIGKILL/断电/内核不可中断 I/O 不保证 finally。

## 4. 实际测试

final-test-matrix.xml/.log：195项通过、无skip；final-gap-tests-complete.xml/.log：另5个不同负向案例通过、无skip。合计200个不同案例，见 final-test-counts.json。

| 层次 | 数量 | 实际断言 |
|---|---:|---|
| 纯状态/期限/原子记录/预检 | 37 | 接收时刻严格小于截止；取消确认不冒充终态；NaN/路径/run_id拒绝；禁用Popen预检仍通过 |
| Linux自有假进程 | 12 | sentinel存活，忽略信号升级记失败，leader消失/后代setsid仍检查，PID复用拒绝，IPC/心跳有界 |
| 只读汇总 | 1 | 缺失/坏记录仍索引，输入不变 |
| 合成MCAP精确内容 | 6 | 151个预定odom；缺固定边/错frame/缺同UUID终态/损坏/缺metadata拒绝 |
| 隔离ROS和实际录制 | 22 | domain72独立命名空间；成功/拒绝/ABORTED、晚接受、同UUID取消和不响应、manager false/晚/不响应、无clock/TF、插件不符、重复请求只发一次、clock暂停/回跳、晚加入tf_static确实收录 |
| 原记录合同回归 | 122 | tests/test_run_schema.py保持不变 |
| 合计 | **200** | fixture只验证接口，不计真实导航成功 |

隔离测试还三次独立启动当前安装的真实Nav2定位组件，autostart=false，单次STARTUP到active再SHUTDOWN正常退出，无Gazebo/初值/目标。22项ROS案例不是22次真实导航。

两方法最终只读预检均退出0；三轮现有validate_run.py run均real/VALID/0，仅说明记录合同。三份关闭MCAP又通过独立inspect_bag CLI实际完整遍历、退出0；开发目录*-bag-independent.json保留新输出。实际命令/退出码见final-cli-checks.json、inspection-cli-exits.json。

## 5. 三个要求回合

| 回合 | 绝对路径 | runner退出码 | startup / Action | harness / cleanup | bag及合同 |
|---|---|---:|---|---|---|
| MPPI正常 | /home/robot/commercial_mobile_robot/evidence/runs/p0-05-mppi-20261025-06 | 0 | READY / SUCCEEDED | COMPLETED / SUCCEEDED | full-navigation有效；VALID/0 |
| DWB正常 | /home/robot/commercial_mobile_robot/evidence/runs/p0-05-dwb-20261025-01 | 0 | READY / SUCCEEDED | COMPLETED / SUCCEEDED | full-navigation有效；VALID/0 |
| 缺初值受控失败 | /home/robot/commercial_mobile_robot/evidence/runs/p0-05-noinitial-20261025-01 | 3（预期） | FAILED / NOT_SENT | STARTUP_FAILED / SUCCEEDED | partial-localization-fault有效；VALID/0 |

Action成功、runner通过、记录VALID、bag有效、清理成功分别判定。

### 正常回合原始证据

#### MPPI

根目录：/home/robot/commercial_mobile_robot/evidence/runs/p0-05-mppi-20261025-06。

- runtime/controller_server.yaml的FollowPath.plugin、runtime/nav-readiness.json和launch加载日志交叉核对实际插件为nav2_mppi_controller::MPPIController，不只看expected_plugin。
- runtime/all-parameters.json保存14个节点实际参数；nav-readiness记录active、速度链端点及目标足迹代价；runtime/nav-final-gate.json保存发令前新鲜TF/clock/scan/odom及Action ready门。
- 唯一UUID：d444fbf04beb42fc9d4817daff14d2dc。logs/action-client.jsonl第1行请求、第2行接受、第847行同UUID SUCCEEDED，终态ROS时间17.709秒，不以CLI退出0替代。
- runtime/observations.jsonl第391至1325行：以发令前最后odom为原点，目标期净位移0.783129350米，最大位移0.783129350米，排除原地直接成功。
- 同文件第1864行：成功后新鲜map→base估计stamp=28.620，年龄0.015000秒；位置误差0.240185817米，归一化角度误差0.124682811弧度，符合实际goal checker 0.25米/0.25弧度。这是AMCL/TF估计，不是独立物理真值。
- 停止窗口ROS 19.152–24.120秒，跨度4.968秒；原始第1407至1643行有139个odom，最大间隔0.036秒；最大线速0、角速绝对值0、最大两两位置漂移0。窗口内clock推进4.893秒，odom新鲜，无缺测补零。
- /cmd_vel最后样本原始第1384行，ROS 18.648，vx=0/wz=0，之后无新非零最终命令。三段最终命令均满足0.001。导航关闭另产生的零cmd_vel_nav保留，不能拿关闭后的零命令制造运动停止窗口。
- runtime/behavior.json为worker停止阶段复算；开发目录offline-independent-review.json又从关闭后的完整JSONL独立算术复核，不读取result数字、不import runner_analysis。后者取更晚新鲜定位，行号不同而误差相同。runtime/bag-check.json另从实际MCAP反序列化、组合录制TF、同UUID status=4与停止复算；bag轴明确为ROS时间。
- runtime/cleanup.json记录两个manager成功SHUTDOWN且节点finalized；recorder、launch、bridge、RSP、Gazebo、worker实际exit0/forced=false/remaining=[]。Gazebo先/server_control stop:true再自然退出；上述停止窗口在仿真仍运行时完成。
- runtime/validation.json、runner-exit.json、failures.json分别记录合同、整体验收与首因/后续失败；result引用关闭日志、bag metadata/分片及报告的指纹。

#### DWB

根目录：/home/robot/commercial_mobile_robot/evidence/runs/p0-05-dwb-20261025-01。

- runtime/controller_server.yaml的FollowPath.plugin、runtime/nav-readiness.json和launch加载日志交叉核对实际插件为dwb_core::DWBLocalPlanner，不只看expected_plugin。
- runtime/all-parameters.json保存14个节点实际参数；nav-readiness记录active、速度链端点及目标足迹代价；runtime/nav-final-gate.json保存发令前新鲜TF/clock/scan/odom及Action ready门。
- 唯一UUID：75dd53f55ee94b3c83423953a630e22d。logs/action-client.jsonl第1行请求、第2行接受、第777行同UUID SUCCEEDED，终态ROS时间17.097秒，不以CLI退出0替代。
- runtime/observations.jsonl第393至1251行：以发令前最后odom为原点，目标期净位移0.782054577米，最大位移0.782464258米，排除原地直接成功。
- 同文件第1777行：成功后新鲜map→base估计stamp=27.720，年龄0.024000秒；位置误差0.241534373米，归一化角度误差0.229888276弧度，符合实际goal checker 0.25米/0.25弧度。这是AMCL/TF估计，不是独立物理真值。
- 停止窗口ROS 18.612–23.544秒，跨度4.932秒；原始第1338至1572行有138个odom，最大间隔0.036秒；最大线速0、角速绝对值0、最大两两位置漂移0。窗口内clock推进4.773秒，odom新鲜，无缺测补零。
- /cmd_vel最后样本原始第1312行，ROS 18.078，vx=0/wz=0，之后无新非零最终命令。三段最终命令均满足0.001。导航关闭另产生的零cmd_vel_nav保留，不能拿关闭后的零命令制造运动停止窗口。
- runtime/behavior.json为worker停止阶段复算；开发目录offline-independent-review.json又从关闭后的完整JSONL独立算术复核，不读取result数字、不import runner_analysis。后者取更晚新鲜定位，行号不同而误差相同。runtime/bag-check.json另从实际MCAP反序列化、组合录制TF、同UUID status=4与停止复算；bag轴明确为ROS时间。
- runtime/cleanup.json记录两个manager成功SHUTDOWN且节点finalized；recorder、launch、bridge、RSP、Gazebo、worker实际exit0/forced=false/remaining=[]。Gazebo先/server_control stop:true再自然退出；上述停止窗口在仿真仍运行时完成。
- runtime/validation.json、runner-exit.json、failures.json分别记录合同、整体验收与首因/后续失败；result引用关闭日志、bag metadata/分片及报告的指纹。


### 受控失败原始证据

logs/events.jsonl第56行为有期限withhold命令，第61行FAULT_ACTIVATED：单调111097.738323974、ROS1.737，initialpose_published=false、goal_sent=false。分支前真实map_server/amcl active和sensor/TF门已通过，不是自然启动失败冒充故障注入。第82–83行规定超时/首因CONTROLLED_INITIALPOSE_TIMEOUT，约10.001159单调秒后结束。

不存在initialpose-request.json、dispatch-intent.json、goal-dispatch.json。MCAP没有initialpose、Action状态或速度命令，只有本阶段应有clock/TF/scan/odom/map/events：317 odom、57 scan、1 tf_static、27 events。覆盖partial-localization-fault，不能写full-navigation或到达/停车成功。清理所有操作成功，无升级/剩余；合同VALID/0，runner保持3。

## 6. 公共条件比较

common-conditions-final.json保存实际参数全差异与内容hash；不能仅凭constraints hash相同判一致。

- 世界、展开机器人、目标、初值pose/covariance、约束、QoS内容相同。robot.sdf SHA256=2e176104db94a82c46d9bf37b88a2b1faa62743c2e443f5dfcefead263411541；world SHA256=d70fad169fc277f4430f7afb488c025dda5b49aaa36518ed33b7a624865c1bd3。
- 出生(-2,-0.5,0.01)，roll/pitch/yaw全0；初值(-2,-0.5,0)，平面协方差对角0.25/0.25/0.0685389；每轮当前stamp不同。目标map(-1.9568372509483436,0.40347409679604745)，yaw1.557737172558183。
- 官方map/模型依赖/Ogre1软件渲染/真实登录桌面DISPLAY和XAUTHORITY一致。Gazebo单独预加载已验证Boost线程库，stop服务后自然exit0；DDS为domain71/FastDDS/SYSTEM_DEFAULT/原loopbackUDP XML，不套旧LOCALHOST配置。
- 14个实际参数节点中13个完全相同，覆盖AMCL、规划、双costmap、平滑器、collision_monitor及其他公共节点。controller_server有144处差异：143个FollowPath算法/插件私有字段，另一个是MPPI声明controller_server.verbose=false而DWB未声明，属于日志差异。
- 按含义映射公共约束：MPPI vx_min/vx_max对DWB min_vel_x/max_vel_x=-0.1/0.3m/s；wz_max/max_vel_theta=1rad/s；ax_max/acc_lim_x=1、ax_min/decel_lim_x=-1；az_max/acc_lim_theta=2。公共平滑器与保护参数完全相同。
- 实际足迹/radius/padding、goal checker（0.25m/0.25rad）一致；目标足迹代价未达到lethal/unknown，但前置余量检查不是独立碰撞评价。
- 目录、UUID、stamp、场景revision和bag hash不同合理。未发现本次公共运行条件实质不一致，不等于证明所有潜在环境因素均受控。

## 7. 所有失败与未完成尝试

p0-05-index.json只读枚举8个真实回合：2个完整正常通过、1个受控失败案例通过、5个开发真实失败。Action成功有3次，因为第五轮导航成功但清理失败，不能算完整通过。没有fixture混入真实runs，本次无missing/invalid；不据此计算成功率。

下表目录前缀均为/home/robot/commercial_mobile_robot/evidence/runs/，原记录均保留：

| 目录 | 最早阻塞/原结论 | 有证据支持的最小修正 |
|---|---|---|
| p0-05-mppi-20261025-01 | SENSOR_READY到期，NOT_SENT，cleanup成功 | profile重名键将展开robot替成raw xacro，Gazebo literal topic无有效odom；修新profile并加hash/XML拒绝测试 |
| p0-05-mppi-20261025-02 | localization STARTUP RPC超时，NOT_SENT，cleanup成功 | 增加只读GetState/is_active门，不重发原STARTUP |
| p0-05-mppi-20261025-03 | 首次只读is_active超时，NOT_SENT，cleanup成功 | 父期限内2秒只读子查询、迟到归档；mutating不重复，不扩大总预算 |
| p0-05-mppi-20261025-04 | 最后一次map/base TF过旧，NOT_SENT，cleanup成功 | 相同0.5秒阈值改有界等待，存最终样本，不降低阈值 |
| p0-05-mppi-20261025-05 | SUCCEEDED、位移/停止/full bag通过，localization SHUTDOWN超时，cleanup FAILED | 缓存复用Client/提前发现，加service事件，三次真实定位组件测试后新建第六轮；旧失败不改 |
| p0-05-mppi-20261025-06 | 完整通过 | 不替换前五轮 |
| p0-05-dwb-20261025-01 | 完整通过 | 同冻结任务 |
| p0-05-noinitial-20261025-01 | 预期CONTROLLED_INITIALPOSE_TIMEOUT，NOT_SENT，cleanup成功 | 计受控失败验收，不计正常导航 |

manager超时的精确内部传输机制没有被独立证明；日志证明对应响应未收到。安装版本源码显示服务创建与处理线程启动存在顺序窗口，因此采用只读门和Client复用；这不证明DDS永久根治。[Nav2 1.3.13源码](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.13/nav2_lifecycle_manager/src/lifecycle_manager.cpp)

development-attempt-index.json列全开发XML和失败用例，包括早期失败、补充测试引号生成导致收集失败、缺profile测试夹具失败。原日志未覆盖；最终通过凭final-test-matrix.xml与final-gap-tests-complete.xml，不写成“全程从未失败”。

## 8. 已验证命令

在Ubuntu：

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/commercial_mobile_robot
~~~

实际退出0且不创建目录/子进程/节点的预检：

~~~bash
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-05-check-mppi-20261025-final --check
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method dwb --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-05-check-dwb-20261025-final --check
~~~

实际执行的单回合命令如下。**编号已存在，原样重跑应拒绝；新运行使用runner_usage中的自动唯一编号方式。** 以下是执行记录，无PATH/ID占位符：

~~~bash
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-05-mppi-20261025-06
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method dwb --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-05-dwb-20261025-01
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-05-noinitial-20261025-01 --fault withhold-initial-pose --ack-test-fault
~~~

实际只读检查/汇总命令，退出0；输出已存在，工具应拒绝覆盖，再查需新输出文件名：

~~~bash
python3 -B tools/inspect_bag.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-05-mppi-20261025-06 --output /home/robot/commercial_mobile_robot/evidence/env/p0-05-dev-bmzezluh/p0-05-mppi-20261025-06-bag-independent.json
python3 -B tools/summarize_runs.py --root /home/robot/commercial_mobile_robot/evidence/runs --prefix p0-05- --output /home/robot/commercial_mobile_robot/evidence/env/p0-05-dev-bmzezluh/p0-05-index.json
~~~

## 9. 放行边界及本人待确认

**已实现且已实测**：受限官方静态任务的一回合自动化，两种方法正常真实路径，一次缺初值真实故障注入，分层测试、合同验证、只读汇总、失败保留。结束按8轮归属token额外只读检查，无剩余非僵尸自有进程，项目锁可用，见post-trial-ownership-check.json；现在的检查不冒充当时readiness。

**未实施/未验收**：独立接触/碰撞/净距评价，真实导航中人为中断/clock回跳/取消不响应的端到端故障（只有隔离接口和纯逻辑覆盖），SIGKILL/断电/内核不可中断I/O保证收尾，动态任务、批量调度、性能/实时性/成功率统计、实车安全。不得把接口测试当成所有真实故障已验收。

collision/delivery_success等未知保持null，evaluation coverage=none/evaluation_valid=false。AMCL初值不是物理真值；odom是运动证据；AMCL/TF是定位估计；bag完整不是无碰撞证明。场景维持合法draft，仅限已核对P0-04官方短静态任务，复杂场景资源闭包/几何对齐不被自动证明。

本人仍需确认：区分READY/Action/harness/cleanup/VALID，手画成功/启动失败/取消未知路径，解释monotonic/ROS/UTC、首因/后续失败、唯一UUID与进程归属，理解RViz中的路径与行为。AI不代签，本次未打开RViz。

工程验收完成，可把本runner作为P0-06的受限输入。后续批量/动态任务须单独放行，不推导控制器排名或实车安全。

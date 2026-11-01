# P1-03 Execution Gate：运行、原理与独立验证

本阶段只处理单目标运动授权和停止收尾。真实 Mission 集成在 domain72 的 Fake Action + synthetic motion sink 中验证；Gazebo 七回合使用原 Runner worker 发送唯一 Goal、显式授权 fixture 续租。两者不能合称“真实 Mission 驱动实物”。消费者 ACK、实车制动与独立硬件 watchdog 不在已证明范围。

## 1. Gate 核心与 ROS 外壳

配送问题：导航仍在发布速度，不代表当前订单仍有运动权限。Mission 卡死时，独立 Gate 必须自行让租约失效。

位置：Controller/Behavior → Velocity Smoother → Collision Monitor → execution_gate → 私有 final → execution_bridge → 私有 drive → DiffDrive。普通 cmd_vel 直通被派生 Bridge/SDF 消除，docking 不启动。

源码：src/cmr_execution/include/cmr_execution/gate.hpp、src/gate_node.cpp。Gate::issue/control/candidate/clock/tick 决策；GateNode 在单线程 Executor 中串行处理请求、候选与 wall tick，publish 在同一边界内执行。纯核心不创建 ROS 节点、不写数据库、不发布速度。输入时钟暂停仍有 wall tick。

借鉴 twist_mux 的来源/锁/过期概念，但它的消息驱动与 ROS 时间不足以满足此租约。复用安装的 Nav2 Smoother/Collision Monitor；不引入另一个仲裁框架，也不让 Gate 再做平滑或裁剪。cmr_core 只复用轮速检查。

必须掌握的五问：

1. 为什么租约不能用 ROS 时间？仿真暂停时 ROS 时间不动，steady 租约仍须过期。
2. challenge 防什么？防止旧排队请求晚到授权；它不是密码学认证，也不是零延迟失联检测。
3. Revoke 回应证明什么？同 Executor 已禁止后续新非零发布；DDS/Bridge 在途旧消息另算。
4. 相同 stamp 为何不能刷新年龄？否则冻结的来源消息会靠重发无限续命；平滑器同 stamp 合法变化只允许原年龄窗口。
5. Reset 会恢复运动吗？不会，只回 DISARMED；还要新初始静止、旧 Goal 解析、新握手和新候选。

## 2. 停止观察器与独立分析

配送问题：订单取消不能因为“发布了零”就释放活动槽，机器人可能还在滑动、链路可能没收到。

源码：stop.hpp 的 StopWindow、stop_observer.cpp 的 Observer；独立 Python oracle 为 tools/execution/stop_analysis.py。Observer 只读 final、GateState、clock 和真实 Gazebo 里程计，绝不发布运动。GateState 绑定 run/epoch/lease/generation，odom 原始样本检查速度、位置、朝向、间隙与时间推进。接收缺口 >=200ms 会重建窗口。3s 是推进的仿真时间，不是墙上等了3秒。

A=Gate禁止，B=外部观测连续零，C=具体下游边界，D=运动反馈窗口。Gazebo 总线端点存在不等于 DiffDrive 回调 ACK；C 消费者级保持 UNKNOWN。DiffDrive 8.15.0 源码用轮关节位置计算 odom，属于运动反馈，但不独立证明世界碰撞/轮滑；原始世界 pose 另存，不冒充碰撞评估。

五问：

1. 零速度但位置漂移能通过吗？不能；每点相对窗口首点检查漂移，朝向也检查。
2. clock 暂停时能报告 STOPPED 吗？新鲜度失效后必须 UNKNOWN；恢复后收集新窗口。
3. 为什么还要 generation？同一次订单重新授权/撤销后旧停止事实不能跨运动周期复用。
4. 跨进程 steady 时间能直接相减吗？接口不依赖此假设；延迟用同一个观察者或请求前后标记括区间。
5. D 能证明消费者收到零吗？不能；A/B/C/D 各有自己的证据边界。

## 3. 真实 Mission ExecutionPort

配送问题：持久订单与外部动作之间有不可消除的不一致窗口，不能绕过 Store 把内存入队当成订单接纳。

源码：cmr_mission/src/mission_node.cpp、mission.cpp、mission_store.cpp、execution_port.cpp。调用链：

Mission API → Runtime owner → Store SQLite事务 → Send意图 → ExecutionPort inspect → owner事务 ExecutionContext/BeginLease → challenge/Begin → owner事务 ExecutionBind/SendAuthorized → 原 NavigationAdapter → rclcpp_action。发送者仍只有一个。

续租：MissionNode 每80ms最多一个 owner future；owner 查活动 attempt 和 FSM、提交 ExecutionRenew/RenewLease 意图、dispatch 后，ROS 外壳才执行 challenge/renew。DB失败、owner冻结、邮箱阻塞不会由另一个续租线程掩盖。ROS RPC/future 期限有界，pending≤8，commands≤32，关键事件≤64，context≤64，effect幂等历史≤4096；满即保守失败。

停止：Revoke ACK、Action terminal、D 分开进入 owner 事务。消息接收时间在同一 Mission 进程内转为剩余有效期，不给排队旧报告重新加200ms。释放前再检查当前 Gate 快照 identity/clock/generation；不能被旧 epoch 报告骗过。

五问：

1. 为什么先授权再发 Goal 不会死锁？Begin 为已提交待发送 attempt 准备权限，不要求 Action 已接受；没有新鲜候选时仍为零。
2. DB事务能与 DDS发布原子吗？不能；持久意图、身份关联、恢复 PAUSED 和禁止自动重发共同控制窗口。
3. Cancel ACK 等于终态吗？不等；Goal 未决或 D 缺失都继续占槽。
4. 晚接受如何处理？已有 P1-02 适配器按精确 UUID 取消，不 cancel_all，不让旧回调影响新 attempt。
5. Mission 崩溃后为什么不自动重发？无法确定外部 Goal 是否已生效，恢复保留活动槽并要求解析。

## 4. Runner、fixture 与可重放证据

源码：tools/runner/trial.py 为唯一试验父进程；tools/navigation/nav_goal.py 保留唯一 ActionClient。tools/execution/runner_gate.py 是可选 profile_version=3 扩展，复用已有 ProcessSupervisor、WorkerLink、生命周期与 MCAP录制，不建立第二套导航系统。owner_fixture.py 没有 ActionClient，也不接收或制造运动反馈。

prepare_gate_trials.py 先重新验证不变的 P0 profile、依赖 hash 和基准 Run，再生成新目录中的 SDF、Bridge、参数和七个 profile。DiffDrive 只改订阅话题；world 只添加只读 SceneBroadcaster；Nav2 只改明确列出的 stamped/routing/低速配置及移除 docking。限值 .18m/s、.8rad/s 在试验前冻结；Gate .2/1.0 保留余量，绝不悄悄裁剪。

五问：

1. 为什么每个 G 回合都冷启动？避免上一回合残留 Goal、缓存、epoch 和控制状态污染。
2. 为何 owner-loss 不杀 Gate？本机 DiffDrive 无独立命令 timeout，杀 Gate 会让最后命令保留；只在 sink 暴露 UNSUPPORTED。
3. 故障资格是什么？唯一 Goal 已接受且 ACTIVE、传感器新鲜、至少3个推进样本、运动≥.05m且速度≥.01m/s；提前自然停车属于 NOT_EXERCISED。
4. 谁造成测得的停止？先观测 Gate A/B 和独立 D，再请求 Action Cancel；不能由取消或关 Gazebo 替 Gate 停车。
5. 为什么 MCAP 再分析？在线报告可能有程序错误；关闭后的独立读取核对实际类型、计数、唯一UUID、标记和原始运动窗口，不播放 bag。

## 5. 构建与运行

在 Ubuntu 仓库执行，所有输出用新的唯一目录；不要复用示例目录覆盖旧证据：

```bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
CMR_VERIFY=$(mktemp -d "$PWD/evidence/env/p103-user-XXXXXX")
cmake -S src/cmr_execution -B "$CMR_VERIFY/core" -DCMAKE_BUILD_TYPE=Release -DCMR_EXECUTION_BUILD_ROS=OFF
cmake --build "$CMR_VERIFY/core" -j2
ctest --test-dir "$CMR_VERIFY/core" --output-on-failure
colcon --log-base "$CMR_VERIFY/log" build --base-paths src/cmr_core src/cmr_interfaces src/cmr_execution src/cmr_mission --build-base "$CMR_VERIFY/build" --install-base "$CMR_VERIFY/install" --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMR_EXECUTION_BUILD_ROS=ON -DCMR_MISSION_BUILD_ROS=ON
source "$CMR_VERIFY/install/setup.bash"
python3 -B tools/mission/run_execution_tests.py --suite experiments/mission/execution-sink-v1.yaml --install-prefix "$CMR_VERIFY/install" --output-dir "$CMR_VERIFY/sink" --check
ROS_DOMAIN_ID=72 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -B tools/mission/run_execution_tests.py --suite experiments/mission/execution-sink-v1.yaml --install-prefix "$CMR_VERIFY/install" --output-dir "$CMR_VERIFY/sink"
```

Gazebo 需要授权及无现存仿真的前置检查。prepare_gate_trials.py --help 给派生输入入口；tools/run_trial.py --help 与 --check 是只读计划入口。按照生成 plan.json 的 scenario/profile/method/run_id 逐回合执行，失败停止排查，保留旧 run_id。不要手工发布 final/drive 或同时启动另一 Goal 客户端。

离线停止重算（不启动ROS、不播放bag）：

```bash
python3 -B tools/mission/analyze_stop.py --observations <本轮runtime/observations.jsonl> --marker EXECUTION_FAULT_BEFORE --output <不存在的新报告.json>
```

验收数量、原始失败、适用 sanitizer、P0 回归及七回合路径见 docs/verification/p1-03-un63b3ef.md；任何尚未取得的证据不能从这里的设计说明推导出来。用户个人学习、面试表达与最终接受不由自动测试代签。

## 显式时钟与真实格式反例

Gate、Observer 和测试 relay 不启用 Node 的隐式 use_sim_time；Gate/Observer 明确拒绝启用并冻结参数。它们消费独立 Clock 数据，用自己的 steady/wall 控制时间，Nav2/Mission/Bridge 则仍按角色使用仿真时间。这避免了 rclcpp TimeSource 的隐藏时钟线程和负时间构造异常，不能把 Node 系统时钟当作仿真来源时间。

新增真实 ROS wire 反例覆盖 NaN、Inf、非法差速分量、超限、错误 frame、负/缺失 stamp、nanosec>=1e9、负 Clock、运行后 Clock 回零。注入在已授权且刚续租时发生，120ms内检查零输出和 Gate/Observer 仍存活，不能靠300ms租约自然过期冒充格式检查。原始非有限输入在 JSON 证据里用 nonfinite 标记保留。

每个正式 Gate 仿真 Run 保存 artifacts 下的ELF并核对profile SHA；实际运行execution_gate与execution_observer两份，candidate_relay仅归档为隔离夹具且不在正式链执行。后续开发构建不改变这些 Run 使用的二进制。

DWB还有一条必须理解的配置关系：RotateToGoalCritic独立读取FollowPath.xy_goal_tolerance。只收紧外层Goal checker会使内层在0.25m开始只允许旋转，而外层仍要求0.18m，出现未推进。当前两者均为0.18m，外部验收仍0.25m。这是配置一致性修复，不能靠放宽到达验收或Gate期限处理。

正式链采用Collision Monitor直接输出私有candidate到Gate，不使用candidate_relay；该文件只保留为隔离测试夹具。G05短暂SIGSTOP本轮拥有的CM子进程350ms，再恢复以避免lifecycle bond超时；A/B在恢复前应已失效，恢复候选不能自动授权。该变更移除额外测试传输跳数，所有200/300ms期限保持不变。

批量复现：prepare_gate_trials.py生成plan.json后，先执行run_gate_matrix.py --plan <plan.json> --output-dir <新目录> --check，再在已授权且无既有Gazebo的环境运行相同命令去掉--check。audit_gate_trials.py读取关闭后的Run，独立重算A/B/D、世界pose交叉核验与同一观察者延迟；不启动Node或播放bag。


## 6. 一次完整的新七回合复现

以下命令使用新目录、新suffix，不能复用验收r20的run_id。已加载上文新构建install后执行；prepare只生成派生输入，不启动仿真。--check通过不代表有运行证据。正式执行前仍由Runner检查现存Gazebo和本轮图，存在未知进程时停止，不能清理它们。

```bash
cd /home/robot/commercial_mobile_robot
P103_BATCH=$(mktemp -d "$PWD/evidence/env/p103-reproduce-XXXXXX")
P103_SUFFIX=$(basename "$P103_BATCH" | tr 'A-Z-' 'a-z_')
python3 -B tools/mission/prepare_gate_trials.py --base-profile "$PWD/experiments/runner_profile.yaml" --install-prefix "$CMR_VERIFY/install" --output-dir "$P103_BATCH/inputs" --suffix "$P103_SUFFIX"
python3 -B tools/mission/run_gate_matrix.py --plan "$P103_BATCH/inputs/plan.json" --output-dir "$P103_BATCH/matrix" --check
# 本阶段已授权的受控仿真入口；串行冷启动，失败即停止，不覆盖失败回合。
python3 -B tools/mission/run_gate_matrix.py --plan "$P103_BATCH/inputs/plan.json" --output-dir "$P103_BATCH/matrix"
```

预期：七个独立Run均exit0、case_pass=true，关闭过程无升级强杀；G01/G02目标SUCCEEDED，故障回合先停止后CANCELED。若NOT_EXERCISED、D未知、图不唯一或cleanup失败，必须保留并停止，修复后再新建suffix。不能重写result或手动把状态改绿。

独立学习路径：先 `ctest --test-dir "$CMR_VERIFY/core" -R execution_boundaries --output-on-failure` 看人工时间边界；再读execution_stop_good/stop_bad，手算3s推进窗口；再运行E04/E12子集观察SQLite事件、服务端Goal数量与BUSY；最后对关闭的G03/G06原始记录做只读分析。E子集可用 `run_execution_tests.py` 原完整参数加 `--cases E04,E12`，输出须指定新的不存在目录。独立分析不启动任何Node或执行端。

本轮读报告时特别检查：G06的A/B从暂停前计，D从恢复后计；丢失output_seq不是可测的调度延迟；C消费者ACK保持未知。理解这些边界后，才能把“测试通过”准确解释给其他工程师。

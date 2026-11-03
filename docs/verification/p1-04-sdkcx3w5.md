# P1-04 交付检查点 — sdkcx3w5

**结论：PARTIAL / BLOCKED_STOP_EVIDENCE。P1-04 没有全部通过。** 后续运动仿真暂停，未进入 P1-05。本报告以实际 Ubuntu 代码和原始记录为依据，替代先前过程检查点的当前状态；历史日志、失败回合和原证据不改写。

## 仓库与边界

实际仓库 `/home/robot/commercial_mobile_robot`，通过 `ssh cmr-ubuntu` 工作。基准与当前 HEAD 都为 `dca497335fe7fba8ee2e4825783411cd65a2cec9`，没有 commit/push/tag/release。改动保留在工作树；未建立 Mac 工程副本。

证据根：`/home/robot/commercial_mobile_robot/evidence/env/p1-04-dev-sdkcx3w5`（以下记 E）。`before.json` 保存接收时 360 个 tracked 文件指纹；`protection17.json` 核对 138 个已有实验输入、仿真/bringup 资源及测试 fixture 全部未变，无基准文件缺失。原 Runner/worker 的集成改动单列在 git-status17.txt；不是宣称整棵 P0 工具未变。`git-diff-check17.log` 无空白错误。

## 阻塞事实：G03 的停止证据不能支撑当前任务放槽

真实回合：`/home/robot/commercial_mobile_robot/evidence/missions/p104_20261102_g03_r04`。故障前实际 UUID `9d17a37fcb27d1c170ea895b83111168` 为 ACTIVE，42 个 odom 样本显示位移 0.0513153 m、速度 0.0691056 m/s。受控 cancel 后，独立 worker 首次观测 Gate 禁止非零的延迟约 0.00425125 s，原因 REVOKED；这只证明 A 层观察结果，不是硬实时或停车保证。

SQLite APPLIED Stop **seq391** 被 Resolve **seq392** 采纳，generation=3，窗口为 ROS 时间 **18.828–21.852 s**。轮式 odom 的速度、角速度、漂移和输出零记录通过，但独立 Gazebo body-pose 在 **18.849 s / raw_seq1237** 的差分平移速度为 **0.010864479736189024 m/s**，超过冻结 **0.01 m/s**。因此该精确时间窗无法证明连续三秒车体静止。DB 的 CANCELED 是实际业务状态，不能据此声称安全取消验收通过；这是需要修复的停止证据可信性缺口。

原 `runtime/bag-check.json` 已报 CONTRADICTION，harness 为 RECORDER_FAILED，Runner exit3；归属进程 cleanup 通过。严格绑定实际消费窗口后的 `E/g03-r04-raw-exact17.json` 仍失败，没有选取较晚样本掩盖问题。`E/BLOCKED_STOP_EVIDENCE.json` 保存机器可读阻塞事实。

任务书前置条款与附加请求明确：停止证据不可信属于核心缺口，必须报告阻塞，不能绕过。因此没有启动 G04/G05/G06，也没有启动需用户真实确认的 G01。未放宽速度、漂移、窗口、时钟或接收期限；未实施“多等一会”或两段窗口替代方案，未在运动中杀 Gate/Bridge。

## 已实现与验证层级

| 模块 | 实现与核心关系 | 当前证据 / 限制 |
|---|---|---|
| 站点与到达 | StationCatalog → 不可变 snapshot；DeliveryMonitor → ArrivalWindow → owner 事务 | 严格 JSON/资源 hash/角度/连续窗口测试；G02 实际 TF 到达重算通过 |
| 真实业务 | DeliveryCommand → 原 MissionRuntime 单写者 → Store → 原 FSM → 持久 effect → 原 NavigationAdapter / ExecutionPort | 双段独立 operation/attempt/UUID/lease；无另建 ROS FSM |
| 有限 FIFO | registrations/queue_events/command_decisions → 原 Store 事务提升 | 容量、幂等、并发、失败暂停、六个真实进程崩溃窗口、20×200 queue seeds |
| 交接 | WAIT_HANDOVER entry token/version → owner 校验与同事务 handover + return intent | source 精确 human/synthetic；20×200 handover/cancel/deadline seeds；G01 本人操作未执行 |
| 执行与收尾 | 原 ExecutionPort Begin/Renew/Revoke/Reset，Goal terminal 与 A/D 分开 | 早取消、晚回调协议通过；真实 G03 的 D 可信性失败，不能验收 |
| Runner | 原 Runner + DeliverySession，worker 显式 observer，不创建 ActionClient | 32 项隔离 ROS worker/Runner 回归；G02 实际 worker 无目标所有权 |
| 记录 | SQLite/MCAP/独立 body pose → exact consumed evidence → session audit | 新增停止报告精确关联反例；旧记录未覆盖。审计版本变化会导致旧报告 strict equality 拒绝，需明确区分原验收与新 raw 重算 |

新增主要文件：`docs/contracts/p1-runner-v1.md`、`docs/contracts/p1-queue-v1.md`、`docs/design/p1-delivery-integration.md`、`docs/guides/delivery-mission.md`；`src/cmr_mission/include/cmr_mission/delivery*.hpp`、`src/cmr_mission/src/delivery*.cpp`、`src/cmr_interfaces/srv/DeliveryCommand.srv`；`tools/delivery/`；`tools/mission/delivery_command.py`、`prepare_delivery_trials.py`、`run_delivery_tests.py`、`audit_delivery.py`、`summarize_delivery.py`；派生 BT/站点位于 `experiments/mission/p1-delivery/`，新 launch 位于 `src/cmr_mission/launch/`。详细调用关系、可复制命令与各模块五个问题及答案见学习指南。

## 构建与测试结果

不同层级分别计数，不相加冒充整体验收。

| 检查 | 实际结果 | 原始证据 |
|---|---:|---|
| 接收纯 C++ 基准 | Mission19 + Gate13 + core5，37/37 | E/baseline-*.xml、baseline-commands.json |
| 接收 P1-02 / P1-03 协议 | A01–A20 20/20；E01–E16 16/16，独立审计通过 | E/baseline-action*、baseline-execution* |
| 当前 Release OFF | Mission36/36 | E/offline18-release-mission.xml、offline18-release-commands.json |
| 当前 Release ROS ON | Mission37/37、Gate14/14、core5/5 | E/slice15-*.xml、slice15-test-commands.json |
| ASan/UBSan OFF（Debug -O1 -g，assert 保留） | Mission36/36；此前 Gate14/14 | E/asan15-O1-mission.xml、offline10-asan-ubsan-cmr_execution.log |
| ASan 原始失败 | O0 合并 seed 项 180s 超时，34/35；相同20seed各200事件逐进程全部通过；O1同180s通过 | E/offline10-asan-ubsan-cmr_mission.log、asan-split-seed-results.json；未删 seed/关断言/提高原时限 |
| 当前默认 Python 离线 | **486 passed / 4 failed / 5 skipped / 32 deselected** | E/python18-all.log/xml、python18-command.json |
| 隔离 ROS 回归 | **32 passed / 493 deselected** | E/ros16-all.log/xml；domain72、loopback FastDDS |
| 最新证据/所有权/反例专项 | 16 passed | E/offline17-oracle-regression.log/xml |
| D01–D18 真实 ROS 协议 | 原完整 matrix09 18 PASS；TF/运动全部 synthetic | E/delivery-matrix-09/results.json，各目录 trace/cleanup/native logs |
| D15 实际坏图注入 | PASS，注入前健康已确认 | E/delivery-D15-qualified-10；实际额外 ActionClient/禁用节点 + 离线配置负例 |
| D16 闭合审计修复重测 | PASS，独立审计 PASS | E/delivery-D16-closed-18、delivery-D16-closed-18-audit.json |

原完整协议独立审计 `protocol-independent-audit-03.json` 为22/22。后来的闭合库限制在 `protocol-independent-audit-17.json` 中给出22/23（含另一次D15）：旧D16留有非空WAL，被新reader正确拒绝，未丢弃WAL或修改旧库追认。新D16增加活动owner拒绝、退出后读取，重测及独立审计通过；旧拒绝样本仍保留。

四个 Python 失败与接收基线同名：旧动态 cohort 依赖身份漂移，以及 `tests/runner/test_runner_gui.py` 三个旧 crossing 输入路径缺失检查。原失败不通过改历史锁/补旧输入来变绿。ROS ASan 进程协议未执行；Gazebo sanitizer 未执行；不能从纯核 sanitizer 推导 ROS 回调无生命周期问题。

## G01–G06 实际结果

| 用例 | 结果 | 说明 |
|---|---|---|
| G01 MPPI 人工完整配送 | NOT_EXECUTED | 必须用户本人交接，AI 未发送 human 确认；当前先受停止证据阻塞 |
| G02 DWB 完整配送 | r04 实际完成，原独立验收 PASS；新 raw 精确重算 PASS | 两个实际 UUID/lease、synthetic 业务交接明确标记；真实运动/TF/两段车体停止。不是 G01 人工证据 |
| G03 运动中取消 | **FAIL / BLOCKED** | 故障命中、A层约4.25ms、目标终态与cleanup存在；D层被独立车体记录否定 |
| G04 交接超时 | NOT_EXECUTED | 因停止证据阻塞未启动 |
| G05 同会话两任务 FIFO | NOT_EXECUTED | 同上 |
| G06 真实 owner 退出/监督重启 | NOT_EXECUTED | 同上；协议验证不能代替真实运动验证 |

G02 r01 BT本地清除服务名错误、r02停止证明重复文件写入导致harness失败、r03交接source字段格式被严格验证器拒绝，均保留，不能以r04覆盖。

G02 的原 `E/g02-r04-independent-audit.json` session_pass=true。修改停止关联后，`E/g02-r04-exact-consumed-audit16.json` 因报告结构/消费窗细节与旧 `runtime/bag-check.json` 不同而 strict equality 拒绝；没有改旧报告来消除差异。新的 **E/g02-r04-raw-exact17.json** 对实际消费的两段窗口均 valid=true、独立车体 STOPPED，两个 body 窗口 span 均3.006s。该 raw 重算与原会话验收分列，不宣称旧产物已按新格式重新完整验收。

G03 的旧完整审计还出现过 SHM 引用缺失，普通只读 SQLite 连接涉及临时文件生命周期；不重建旧证据追认。新的闭合审计改为已有 owner 锁共享持有、拒绝非空 WAL/journal，再 immutable=1，显式关闭。`*-raw-exact17-preservation.json` 证明这次 G02/G03 raw 重算前后所有实际证据文件 hash/集合不变。SHM 不作为未来闭合 session 的持久业务证据；非空WAL不能静默排除。

## D矩阵边界与未验证项

D01/03/04/08/10/12需要代表性Gazebo的部分尚未全部满足；D03取消真车体停止失败，D04超时/G05队列/G06owner失效未执行。D14持续真实占道故障为 **NOT_EXERCISED**，协议ABORTED/预算耗尽不等于真实障碍恢复。D17电量来源 unavailable，无真实低电回航证据。C层消费者ACK仍 UNKNOWN；Gate自身崩溃、Bridge断链、下游保留旧命令缺少独立保护；collision=null（未知）。

## 后续修复门（方案，尚未实施）

1. 先核对安装 Gazebo DiffDrive 的真实里程计来源、轮子运动与车体运动为何可能不同；阅读对应源码/配置和原始数据，不能只调等待值。
2. 在停止证据合同明确区分轮式静止估计和可信车体反馈。评审独立只读车体运动观察的来源、边界、身份、epoch、连续窗口、超时与缺失处理；仿真真值不接入 Nav2 定位/目标/速度规划，不能扩张为实车能力。此处涉及反馈合同设计，未擅自修改关键语义。
3. 用现有反例证明缺失/冲突/陈旧反馈必须保持 STOP_UNKNOWN/PAUSED、占槽并锁存调度；不能仅靠离线报告发现问题，而运行时仍放槽。
4. 修复后先纯逻辑/ROS sink和 P1-01/02/03回归，再新run_id重做G03与G02，全部门槛满足才进行剩余回合。G01最后由本人实际确认；AI不代确认。

本轮停止处是任务书规定的安全阻塞检查点，不是P1-04最终验收。

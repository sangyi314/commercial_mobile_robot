# P1-05 交付报告 b1sg_5gd

状态：**PARTIAL_1H_NOT_COMPLETED**。本次完成了独立于正式一小时的研发、修复、回归、核心仿真、同机复现和 development 候选打包。用户最新明确“现在不方便，先完成其余交付”，所以 **R16 没有启动**；不宣称工程版A/v0.2验收通过、不自动进入P2。

工程权威目录 `/home/robot/commercial_mobile_robot`，通过 `ssh cmr-ubuntu`；Mac仅存阅读用交付文档。接收HEAD `dca497335fe7fba8ee2e4825783411cd65a2cec9`，最终观察HEAD `73fe998f2929a430b1ae099d656a88a1b60379bd`，包含原有与本轮未提交工作树；没有commit/push/tag/release。证据根 `/home/robot/commercial_mobile_robot/evidence/env/p1-05-dev-b1sg_5gd`，原始仿真在 `/home/robot/commercial_mobile_robot/evidence/missions/`。

## 授权、门槛与适用范围

- 原human G01到达实际WAIT_HANDOVER，但未获得本人操作，`p104_20261103_g01_r10`保留失败；**human G01未通过**。
- 您明确授权AI代交接后，建立ADR `docs/adr/p1-delegated-handover-20261103.md` 和准入记录。G01D真实Mission/Gazebo闭环、交接来源synthetic；这是获准的P1-05准入例外，不是补造human记录。
- `oracle_stop_assisted=true, sensor_only_E2=false`。Gazebo worldPose只用于获准的仿真停车资格；不进入Nav2定位/规划/速度，不代表实车安全或sensor-only E2。
- 实际控制器为Rotation Shim + MPPI，另有DWB回归，不称裸MPPI，不做性能排名。
- 正式连续3600秒、本人讲解确认/独立练习、正式发布均未完成。P0-09继续暂缓。

## 状态分层

| 层级 | 结果 | 原始依据（相对证据根） |
|---|---|---|
| 已实现/静态 | 可靠性合同、固定负载、驱动、独立模型、资源与录制监测、审计及打包 | source快照、changes-from-intake、合同/指南 |
| 纯逻辑/SQLite | 新目录OFF Release 37/37 | reproduce-r14/pure-tests.xml |
| ROS Release CTest | Mission38/38，Execution15/15，cmr_core5/5 | reproduce-r14/*-tests-r16.xml |
| 真实ROS隔离协议 | A01–A20 20/20，E01–E16 16/16，D01–D18 18/18，重建产物再次54/54 | reproduce-r14/action-r16、execution-r16、delivery-r16 |
| ROS Python专用 | 33/33 | ros-python-r05.xml |
| 当前Python普通集合 | 514通过，4失败，5跳过，33专用ROS项另跑 | python-final-r16.xml/log |
| Sanitizer | Mission纯核心36/36；Execution纯核心15/15；实际ExecutionPort容量/生命周期1/1 | cmr_mission-asan.xml、cmr_execution-asan.xml、asan-port-r06.xml |
| 固定种子 | queue模型20×200；handover模型20×200；原Mission v1 20×200；新SQLite/旧身份v2 20×200 | *model-r01.json、fixed-seeds-final-r16.json、原生CTest原始目录 |
| Gazebo核心回归 | 正常MPPI/DWB、运动取消、交接超时、owner死亡重启5/5符合预期 | core-l2-r10-results.json、final-valid-audits-r16.json |
| 短时资格 | 连续300.020842秒、3单COMPLETED、6个Goal和确切停止窗口 | p104_20261103_g05_p105r7、session-statistics-r16.json |
| 干净产物代表L2 | 新目录重建二进制真实G01D闭环通过 | clean-l2-r16-results.json |
| 正式一小时 | NOT_EXECUTED，最新监督安排延期 | supervision-deferred.json |

这些计数来自不同测试集合，不能相加当作独立随机样本。ASan/UBSan未覆盖完整Nav2/Gazebo进程系统，未运行TSan；不宣称全仓全绿。

## 实际修复及调用关系

1. `src/cmr_mission/src/execution_port.cpp`、新 `include/cmr_mission/effect_sequence.hpp`：原seen集合4096项会把每次Renew永久计入，单次配送实测283次，正式负载预测5660次必溢出。改为同attempt的已提交effect版本单调收据；结构effect仍限4096、context64、Gate Begin64，旧版本/撤销后续租不能复活。实际ROS port单测5000续租，保留原过载和早取消反例。不是增大安全阈值。
2. `src/cmr_mission/src/mission_node.cpp`、`tools/mission/run_delivery_tests.py`：11个真正并发请求时原DDS默认depth10丢第一条，DB与DELIVERY_RX均无该请求。服务/client显式Services QoS64，业务FIFO4与owner邮箱16保持；D06重新通过。原失败在delivery-regression/D06和D06-first-failure-analysis.json。
3. 新 `tools/reliability/` 与 `tools/mission/run_reliability.py`：使用既有Runner/DeliverySession，不创建第二个Goal客户端或进程管理器。候选冻结→固定计划→owner业务服务→SQLite提交→真实ExecutionPort/NavigationAdapter→Nav2；Gate独立有限租约，独立Stop窗口再允许业务推进。
4. `resources.py`按PID+start_ticks只读拥有进程，steady每秒采样。recording使用系统rosbag2 cache0和MCAP noChunking/dataCRC，t0前要求实际文件进展，停流拒绝；文件增长不证明fsync或断电持久性。首次短回合因默认100MiB cache无增长失败，0个原生Goal，失败目录保留。
5. `records.py`与`inspect_delivery_bag.py`：启动manifest与终态hash双核对，SQLite逐行核对导出事件及计数，重建UUID和业务守恒，消费窗口按attempt/epoch/generation/from/to匹配。`world.py`再从原始Gazebo样本跨边界检查同一窗口，不选择后续好片段。
6. 原生interleaving新增独立注册v2，完整events/trace/每步ledger/DB/外部发送记录；实际152次BUSY，重启不重发。queue和handover各有独立简化Python模型，不调用reducer作Oracle；SQLite/Action流使用SQL/receiver不变量，不冒称另写了全功能独立FSM。

架构、线程/Executor/回调数据流图及20问答见 `docs/guides/p1-reliability.md`；本轮新增/修改的绝对路径见 `changes-from-intake-final.json`。初始已有改动与本轮新增按intake SHA区分，不将整个Git diff算作本轮贡献。

## 开源研究、复用与许可

研究本机rclcpp28.1.22 Executor/Action接口、Nav2 1.3.13源码/实际BT与固定commit `d5d768b7b38a05e7cfd2b6c693b786b402161bed`、SQLite官方WAL设计、Gazebo8.15.0固定 `446a44335a45b704b4d36dabcc5508ee34eeb3d8`。沿用成熟安装组件；不照搬默认多轮Spin/BackUp恢复，不增加YASMIN/Waypoint Follower框架。研究过程见research.md及上一轮固定上游文件。

录制修复正式编码前读取rosbag2 tag0.26.11 commit `cb8fa198b93bfed90e2417b52b8645fb9ef89c2b`的SequentialWriter、mcap_storage和测试/许可；Apache2。MCAP releases/cpp/v1.3.1 commit `1a64fd40358998775b27fee516a28e07a97db27e`，MIT，保留 `third_party/licenses/mcap-1.3.1-MIT.txt`。使用系统组件，不复制上游实现。CMR可靠性/审计为理解设计后独立实现；AI辅助范围在AI_USAGE.md。

`license-metadata-audit.json`记录根MIT与P0 cmr_core/cmr_sim包Apache2元数据差异；未改冻结P0来自动解决授权归属。**正式发布仍需所有者核对包级许可范围**。这不等同法律判断，不阻断本地工程测试；本次无发布。

## R01–R16实际矩阵

| 编号 | 内容 | 结果 | 精确测试映射 |
|---|---|---|---|
|R01|正常完整配送|PASS|G01D,D01,G02,G05|
|R02|运动中取消|PASS|D03,G03|
|R03|交接超时|PASS|mission_delivery_expire,D04,G04|
|R04|重复与冲突|PASS|mission_admission_*,D06,G05|
|R05|Owner死亡重启|PASS|mission_crash_windows,D09,G06|
|R06|拒绝/接受超时/迟接受|PASS|A04,A08,A10,D11|
|R07|结果取消竞态|PASS|mission_termination_*,A06,A07,A09|
|R08|Server消失恢复|PASS|A12,D11/server_loss|
|R09|SQLite故障崩溃|PASS|mission_store_contract,mission_delivery_fault,mission_delivery_crash_windows,A16|
|R10|有限FIFO并发|PASS|mission_delivery_*,D06,D07,D08,G05|
|R11|旧身份租约|PASS|mission_action_correlation,execution_identity,execution_idempotence,A11(adapter-unit+ROS),E04,E05|
|R12|停车缺失陈旧矛盾|PASS|execution_stop_bad,execution_body,test_body_stop_ros,D12|
|R13|时钟异常|PASS|execution_clock,execution_time_format,A14,E06,E10|
|R14|有界恢复|PASS|mission_state_timelines,D14; physical persistent obstacle NOT_EXERCISED|
|R15|Runner记录清理异常|PASS|D16,test_delivery_audit_exit,test_reliability|
|R16|连续3600秒|NOT_EXECUTED|run_reliability.py, profile5|

矩阵完整前置、资格、故障点、期限、原始路径和限制见 `reliability-matrix-results-r16.json`；设计矩阵 `experiments/mission/reliability-v1.yaml`故意不伪装运行结果。A11为adapter注入+ROS，不能称原生server伪造UUID协议实验。R14的有界失败/重试已用Fake Action实际协议验证，真实持续占道障碍NOT_EXERCISED。Gate死亡/下游断链仅sink负例UNSUPPORTED，不在运动仿真杀Gate。

R07额外实际协议反例 `cancel-gap-r17-results.json`：仅暂停本轮拥有的Fake Server，原生cancel无响应超过冻结1秒；Mission收到CANCEL_ACK_TIMEOUT，保持PAUSED/BUSY、不释放槽、不重复Goal。恢复同一server后同UUID晚ACK/终态，再synthetic停止解析为CANCELED。它验证取消应答缺失窗口，不声称网络丢包实验。

I1–I10逐项见invariants-r16.json：单槽、单未决、幂等、终态不复活、unknown占槽、重启不动、提交失败无派发、旧身份隔离、总恢复预算、到达/停车/交接/返航分离。范围是已执行调度集合，不是任意并发形式化证明。

## 原始L2及统计口径

| 会话 | DB最终状态 | Goal数 | 独立审计/清理 |
|---|---|---|---|
|p104_20261103_g05_p105r7|COMPLETED:3|6|PASS|
|p104_20261103_g01d_p105r10|COMPLETED:1|2|PASS|
|p104_20261103_g03_p105r10|CANCELED:1|1|PASS|
|p104_20261103_g04_p105r10|FAILED:1|1|PASS|
|p104_20261103_g06_p105r10|PAUSED:1|1|PASS|
|p104_20261103_g02_p105r10|COMPLETED:1|2|PASS|

核心5回合的正常配送完成2/2；预期故障行为3/3；harness5/5；独立证据5/5；结束仍有PAUSED/UNCERTAIN订单1（G06），等待订单保留，不计为失败或完成两次。短时3/3 COMPLETED单独统计；每个session从空DB起算。DB审计逐条验证接纳=COMPLETED+FAILED+CANCELED+非终态活动+WAITING，原生UUID去返程分开。

本轮原始11次L2启动尝试（另加同机重建代表回合）保留在all-attempts-final.json；初始human未操作、默认缓存停流、错误PYTHONPATH启动等失败不从分母删除。R04早一次观察器晚到未命中为NOT_EXERCISED；后r10实际NAVIGATING同UUID资格后重复/冲突请求才算通过。不把期望CANCELED/FAILED/PAUSED当成功配送。

短时300次1Hz采样，最大gap1.029896秒，峰RSS1496846336B，峰CPU137.653%（多核总和），最小磁盘余量59428966400B，最后可见bag50536448B；准确sim跨度/RTF见session-statistics。仅300秒，不推断内存泄漏、MTBF或商用可靠性。MCAP chunkless读取提示无message index并降级file order已保留，raw身份和时间仍逐条检查。

审计反例audit-negative-r13：1个正常对照接受、11个损坏副本拒绝，12/12验证通过，故障注入均实际完成；含缺日志/错hash/重复Goal/错停止窗/伪human/队列台账/遗留等待/事件导出/冻结输入/伪计数/采样缺口。原件前后hash一致。早r12两个子项先被未关闭WAL挡住，已标NOT_EXERCISED并保留，r13才算语义命中。

## 全部失败、跳过与边界

- 当前Python4失败为旧dynamic crossing资源/依赖问题：test_new_dynamic_inputs_are_explicit_drafts的冻结worker hash期望a599…，此前实际766c…，本轮3f16…；首因同为历史依赖锁漂移，但实际内容不同，不能只按同名豁免。另test_check_zero_side_effects_and_old_plan[flags0/1/2-dynamic]为Referenced file must exist and be nonempty。逐条旧/新错误及影响在historical-python-final-r16.json。它们影响旧动态P0复现，不是当前static P1路径；未改冻结输入求绿。
- 本轮首次随机v2模型审计在Store fault后调用get而拒绝；改为独立SQLite核对再reopen，旧失败日志保留。不是忽略BUSY。
- 新目录第一轮空环境没有ROS日志目录导致1个ROS单测失败；指定独立ROS_LOG_DIR后曾错误复跑同一CTest DB目录，7个一次性fixture拒绝/残留状态失败。全部保留；最终新build-r16首次执行58/58。部分历史CTest不是可重复fixture，**每次必须全新build/install**，不删除旧DB。
- Python-final-r15漏source ROS导致额外7个import失败；正确环境r16只剩已核对4历史失败。错误marker ros曾发现0项exit5，后ros_contract实际33项通过；不算首轮已执行。
- 下游接收C仍CONSUMER_ACK_UNKNOWN；Gate发零不证明Bridge收到，也不证明实车停止。collision=null，没有可靠碰撞计数。Gate崩溃、Bridge断链、下游保持旧命令缺少独立保护；sink负例明确UNSUPPORTED。
- 低电量unavailable；真实持续占道恢复未执行；没有实车、硬件制动或安全认证；一小时稳定性未验证。

## 同机干净目录复现

解包candidate-r14完整416文件源码，逐文件hash通过；`reproduce-r14/source`不借原工作区源码构建。env -i仅PATH/LANG/PYTHONDONTWRITEBYTECODE后source系统ROS，显式新ROS_LOG_DIR；日志保存完整环境，未加载旧ros-install-r06。OFF Release37/37；新ROS build-r16/install-r16全部CTest58/58；使用解包目录自己的协议工具和新产物完成A20/E16/D18及独立审计。

实际命令/退出码：reproduce-r14/commands.json、build-command-r16.json、test-commands-r16.json、test-environment-r16.json、independent-audits-r18.json。旧环境错误/重测目录保留。

不同路径重建ELF有链接地址、动态表/构建ID/.text差异，.rodata相同，详见elf-comparison-r16.json；不能称逐字节可复现。额外用新install的Mission/Gate/Observer经权威原Runner运行 `p105_20261103_g01d_cleanr16`，真实双Goal闭环、synthetic交接、确切停车窗和清理独立通过。主工程运行代码与解包版本相同，原Runner仍是唯一生命周期owner。此为同机复现+代表L2，不宣称新机器、全部故障L2或新产物一小时通过。

## 候选与可复制命令

开发包目录 `/home/robot/commercial_mobile_robot/evidence/env/p1-05-dev-b1sg_5gd/package-final`，SHA清单、源码/patch、原ROS安装tar、clean构建证据、配置/矩阵、依赖、docs、raw-index和限制均在此。最终完整candidate_id/source_id以package-final/DELIVERY.json和candidate-final/candidate.json为准；不能只看HEAD。R14源码冻结ID `3fd363a3eb6ead07722d38617b628d232124c3f5d7b9c6e690bed2af03f6617f`；其后仅文档/矩阵元数据修订，兼容分析列逐文件差异；运行代码/二进制与已测对象绑定。

```bash
ssh cmr-ubuntu
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
source evidence/env/p1-05-dev-b1sg_5gd/ros-install-r06/setup.bash
python3 -B tools/mission/run_reliability.py --help
# 只读复核会话；输出必须用新的路径，不能覆盖已有报告
python3 -B tools/mission/run_reliability.py audit --session evidence/missions/p104_20261103_g05_p105r7 --output /tmp/p105-audit-NEW.json
```

首次重建参考package-final/REPRODUCE.md；每次新建解包与build/install路径、只运行一次有状态CTest。协议环境固定Domain72+loopback/FastDDS，无真实执行端。完整命令以保存JSON argv为准，不需反推shell引号。

正式输入在formal-final，使用20单固定计划、60秒机会、每5次重复、前两相邻FIFO、总60attempt额度；Gate64历史上限不改；冻结300ms lease、3秒stop、1秒采样、3秒gap、6GiB RSS、700%CPU、8GiB余量、180秒drain。正式目标3600秒，至少10个正常完成且前中后各有完成。先前formal-r10保留但来源已旧，不启动。

**现在不运行formal-final**。将来您明确能监督后，先复核指纹、运行目录未占用和先决证据；漂移则新候选/新输入/必要重测，监督记录必须是当时有效答复，不能复用旧肯定答复越过最新延期。最小启动命令只放package-final/NEXT.md并明确待监督，不自动执行。中止方式“停止仿真”，由原Runner撤销/解析/停车/flush/归属清理；不全局kill。

## 保护与本人验收

P0/已有输入基线140/140 SHA不变；Git diff/check与最终归属进程清单在final-protection-and-git-r19.json。期间观察到HEAD发生delivery_workflow提交，来源未独立确认；本执行未调用Git写操作、未回退。head-change-impact-r19.json确认src/tools/tests与冻结R14逐字节一致、原R06产物hash不变；最初“HEAD应不变”的失败检查记录也保留。未删除未知进程或历史文件、未覆盖旧evidence，未改冻结输入、超时/停止/速度阈值求绿。原始大录包保存在repo evidence持久目录，包内绝对路径+SHA索引，不依赖/tmp。

本人学习材料已准备，尚未由您确认掌握。按指南5分钟讲全链、20分钟解释真实取消竞态/崩溃窗口，再在独立练习副本关闭AI改一条准入规则并写反例；AI不代签。P2仅有交接限制清单，不开始：先满足R16、个人验收、许可范围审查，再由您决定下一阶段。

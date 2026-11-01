# P1-03 验证矩阵 — un63b3ef

2026-11-02；最终结果与限制见 [验收报告](p1-03-un63b3ef.md)。

本表的 PASS 表示该行声明层级的预期行为得到验证。E11 的测试 PASS 特指缺少保护的负例被复现，保护能力为 UNSUPPORTED；不能把测试通过解释为安全能力通过。

## 公共前置条件与证据

实际仓库 `/home/robot/commercial_mobile_robot`。以下 E 根目录为 `evidence/env/p1-03-impl-un63b3ef`。

Release：`E/sink-full-07/results.json`、`E/sink-full-07-audit.json`；ASan/UBSan：`E/final-asan-sink/results.json`、`E/final-asan-sink-audit.json`。均16/16，原始日志为各自 `<E编号>/trace.json`、`gate.log`、`observer.log`、`cleanup.json`。真实 Mission 回合另存 `mission.log`、`server.log`、SQLite DB/WAL，嵌套子回合也各自保留。trace 含接收顺序、请求与故障标记，SQLite 由独立只读审计核对事件、effect 与 Goal 数量。

ROS Domain72、loopback、FastDDS；专用随机 namespace/Action 名称；单回合关闭本轮拥有的进程后才开始下一回合。`--check` 不建节点、进程、DB或输出目录。synthetic odom 仅证明协议及失效状态，不证明机器人停止。纯核心13组和 Mission OFF19组的 XML/log 位于 E/final-off-*；ASan同范围另列，不与ROS协议混算。

| 编号 | 前置条件 | 注入/输入 | 独立验证依据及预期 | 实际结果 |
|---|---|---|---|---|
| E01 | DISARMED；候选非零 | 错误 owner；纯核心另测 gate/operation/attempt/epoch | 外部 final 恒零；身份拒绝 | PASS：ROS + execution_identity |
| E02 | 新初始 D、challenge、合法 Begin | 新鲜 .1m/s 候选→主动 Revoke | 数值原样通过；Revoke 后 control_seq 对齐的输出零，独立 D | PASS：ROS |
| E03 | 已授权，记录原 deadline | 重复/冲突 Begin、重复 Renew、过期 challenge、Revoke 后旧请求 | 期限不刷新；冲突与旧令牌拒绝；状态不复活 | PASS：ROS + execution_idempotence |
| E04 | 真实 MissionRuntime/Store/ExecutionPort、Fake NavigateToPose、合成运动 sink | 实际 Mission SIGSTOP；SIGKILL+同库重启；外部 SQLite BEGIN IMMEDIATE 锁 | Gate 存活且≤600ms失效；服务端 Goal 请求只1个；恢复 PAUSED/BUSY；DB fault 锁存 | PASS：3 个真实 Mission 子回合；不等于物理停止 |
| E05 | 授权且刚续租；租约300ms | 停候选但继续 Renew；重复 stamp；10种畸形 ROS wire | 可定位 CANDIDATE_EXPIRED/SOURCE_TIME/INVALID_CANDIDATE/CLOCK_INVALID；120ms内零且节点存活 | PASS：ROS；未来 stamp 等另有纯核心反例 |
| E06 | ROS Clock 推进、已运动 | 暂停→恢复；回退；重新开始后不自动授权 | wall tick 持续；缓存与 clock epoch 失效；恢复保持禁止 | PASS：ROS + execution_clock |
| E07 | 保存旧 Begin/epoch | 停止旧 Gate，再启动新 Gate；等待实际服务回复 barrier | 新 epoch、默认禁止；旧请求 IDENTITY；不以图就绪冒充服务可用 | PASS：ROS restart |
| E08 | 未授权，sink无执行端 | 额外 candidate/final 发布者；错误类型独立 participant；Bridge命名订阅者 | 预检拒绝且未授权；额外原生 GZ 来源由 native-route-01 单独检查 | PASS：ROS graph + 真实 GZ Transport 无执行端 |
| E09 | 真实 Mission 已导航 | 停 odom；非 Observer 同 topic 伪造 STOPPED；合成 stop API | 实际 owner 保持 PAUSED/BUSY；FIXTURE_STOP_PROHIBITED；恢复证据才释放 | PASS：真实 Mission + synthetic sink；旧/错身份/间隙另有纯核心/Store测试 |
| E10 | sink空闲→本轮有界CPU负载 | owned 2s CPU loop | Gate steady tick 分布/最大值；不宣称硬实时 | PASS：Release 157 tick，p50 20ms/p99 21ms/max22ms；负载失效见G07 |
| E11 | 无物理执行、保持最后命令的 synthetic sink | Gate SIGSTOP/SIGKILL；逻辑接收订阅断链/重连 | 故障期间保持 .1m/s 的负例必须出现；重连后只收零；D不能假STOPPED | 测试 PASS；失效保护 UNSUPPORTED |
| E12 | 真实 Mission + Fake Server | Cancel ACK但延迟终态；另回合终态已有但D缺失 | 两种顺序实际 owner 均BUSY；三者齐备后释放 | PASS：2种顺序，真实协议 + synthetic运动 |
| E13 | 有效候选持续发布 | Revoke与发布交错；纯核心确定性并发 | 线性化之后的新 publish 零；DDS在途观察另记 | PASS：ROS + execution_concurrency |
| E14 | 人工时间纯核心 | now 精确等于 lease/challenge/candidate deadline | 边界拒绝；无sleep猜边界 | PASS：execution_boundaries；不算真实ROS期限协议 |
| E15 | 旧运动已撤销/停稳→Reset→新Begin | 缓存固定旧 stamp；旧 generation 停止报告 | 旧缓存不通过；新 generation；旧事实不可释放 | PASS：ROS + stop_bad + Mission Store反例 |
| E16 | STOPPING但新D不足 | 过早Reset、production模式、lease=0、未知参数、错Domain Observer | Reset不ARM；配置失败有界退出2；真实Gate入口禁止fixture stop/arrival/resolve | PASS：ROS startup + Mission集成 |

E05 的10个实际 wire 子目录：wire_nan、wire_inf、wire_dimension、wire_limit、wire_frame、wire_negative_stamp、wire_missing_stamp、wire_nanoseconds、wire_negative_clock、wire_zero_clock。时间格式错误不会经过 throwing rclcpp::Time 转换。注入后观察窗口120ms小于新续300ms租约，避免把自然过期冒充格式检查。

E08原生边界证据：`E/native-route-01/result.json`。实际 Gazebo Transport 进程，独立 loopback partition，不启动 Gazebo Server/Bridge/actuator，始终只发零；生产预检函数接受1来源、拒绝2来源。旧cmd_vel直通在新派生Bridge/SDF中消除，七个实际Run再检查图；不在机器人运动时注入旁路。

## G01–G07：真实仿真，不冒名为真实 Mission 驱动

共同资格：本轮唯一worker Goal已接受、Action ACTIVE、传感器新鲜、至少3个推进反馈样本、位移≥.05m且速度≥.01m/s。故障前若已自然停止必须 NOT_EXERCISED。G01/G02为正常目标，正常收尾撤销。G03–G07先独立证明Gate A/B及D，再取消唯一Goal，不由Cancel制造停车。

正式输入 `E/gazebo-inputs-r20/plan.json`，调度/退出 `E/gazebo-matrix-r20/`，独立审计 `E/gazebo-r20-audit-final2.json`。每个Run根目录 `evidence/runs/p103_20261101_gNN_r20/`：manifest/result，artifacts的源文件与ELF及hash，logs，runtime/observations.jsonl、execution-check.json、p0-08-gazebo.jsonl，关闭的MCAP bag。所有7回合均冷启动、独立run_id、清理PASS。

| 回合 | 真实输入/注入 | 预期及独立依据 | 实际 |
|---|---|---|---|
| G01 | MPPI正常短目标 | 唯一速度出口；实际参数；独立到达≤.25m/.25rad；收尾A/B/D | PASS；SUCCEEDED |
| G02 | DWB同约束 | 同上，不作性能排名；内部RotateToGoal XY与Goal checker均.18m | PASS；SUCCEEDED |
| G03 | 运动中主动Revoke | 实际运动资格；A/B先到；D新窗口；随后精确取消 | PASS；REVOKED；CANCELED |
| G04 | SIGSTOP本轮持久授权owner fixture | Gate存活，单调租约到期；不自动恢复 | PASS；LEASE_EXPIRED；CANCELED |
| G05 | SIGSTOP本轮CM进程350ms后恢复 | 候选到期瞬间租约仍有效；A/B在恢复前；恢复不重启运动 | PASS；CANDIDATE_EXPIRED；CANCELED |
| G06 | 真实暂停仿真1s再恢复，world stats确认 | wall输出不停止；暂停D未知；恢复后新D窗口；禁止自动运动 | PASS；CANDIDATE_EXPIRED；CANCELED |
| G07 | 两个owned CPU loop下owner失效 | CPU时间确认负载命中；与G04同配置，测响应而不保证硬实时 | PASS；LEASE_EXPIRED；CANCELED |

每回合A/B/D均有非synthetic仿真原始证据；世界model pose提供额外交叉检查，不积分命令冒充运动。C的消费者回调ACK均为CONSUMER_ACK_UNKNOWN。停止分析保留减速前缀；须在最后一次运动/漂移违规之后找到连续≥3s合格后缀，持续移动反例不能通过。原始前缀没有删除。

真实 Mission 集成由E04/E09/E12证明；G回合授权owner是显式fixture，Action sender为原Runner worker。没有执行“真实Mission崩溃→Gazebo物理停止”的组合，不得拼接两个层级的证据声称已验证。

## 其他必须保留的核对

- 纯核心确定性并发、20固定seed×200输入；P1-02 A17另有20×200业务事件接收顺序/重放；二者不是同一测试。
- Release/ASan、P1-01/02回归、Runner31个ROS测试、离线470/4/5/31、TSan不可运行及历史失败，完整数量在验收报告。未执行/不支持部分不计通过。
- 不做运动中Gate SIGKILL。没有消费者ACK和独立下游watchdog，不声称Gate自身死亡/Bridge断链仍能停车；硬件制动、碰撞安全、P4延迟传播均未验证。

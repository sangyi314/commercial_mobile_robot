# P1 Mission 合同 v1

2026-10-31；依据 Master Plan §4.2、§5、§6、§12 P1。本合同冻结本轮开发语义，未来变更需新版本及迁移测试；不是个人验收签字。实现状态见 verification，不以本文推定功能存在。

## 1. 层次与身份

`MissionRequest → validate/decide_admission → MissionStore transaction → dispatcher → NavigationPort/ExecutionPort → events → reducer → transaction`。
纯决策函数无 I/O、无线程、无 ROS。单写者串行处理事件；只有提交成功后才能回复 ACCEPTED 或执行新副作用。

请求 v1：schema_version=1；mission_id、delivery_station、return_station、map_version、station_version 均为 1–64 字节 ASCII `[A-Za-z0-9_.-]`，区分大小写，不 trim、不折叠、不接受隐藏 NUL/Unicode。站点为标识符，中文显示名在目录元数据中；同站去返允许，后续按目录检查合法性。
created_at_unix_ms 为调用方创建时间，整数且 >0；只作审计，不作 TTL。mission_id 不允许复用。

同 ID 的业务相等按 schema_version、两个站点、map_version、station_version 逐字段精确比较；created_at 不参与比较，重复请求保留最初时间。不比较 JSON 键序、不用 hash 代替原字段。不允许缺省字段隐式变更含义。
请求不带 mission_status/retry_count/termination_reason：这些是管理器拥有的持久记录字段，初始为 ACCEPTED/0/NONE。后续记录增加 handed_over、state_version、event_seq、process_epoch、operation_id、attempt_id、nav_phase、pending_termination、frozen_station_poses。

Mission ID 是订单身份；operation ID 区分去程和返程；attempt ID 区分重试；Goal UUID 是 ROS Action 身份。在发送前占用未决导航槽，rclcpp_action 得到 UUID 后立即关联并持久化；UUID 未知的崩溃窗口必须保留 UNCERTAIN，不能借用 Python worker 的自选 UUID API 假定 C++ 同样支持。

## 2. 准入优先级与首个实现切片

顺序：拒绝非法请求 → 如查到同 ID，业务相同返回 EXISTING，不同返回 CONFLICT → 新 ID 遇占槽返回 BUSY → 否则返回 CREATE_INTENT。
EXISTING 必须返回已有当前状态/终态，不生成副作用；CREATE_INTENT 只是事务输入，绝不是已接单的响应。
`AdmissionContext` 是同一写事务快照中的已有记录及活动槽；纯函数验证已知记录确实对应请求 ID、已知记录本身合法。存储层承担完整性、并发串行化及状态查询，不允许网络调用方自行提供这个 context。
首切片只实现值对象、相等规则、纯准入决策和测试；没有可启动任务管理器，没有 DB，没有实际 FSM 跳转，也没有运动输出。

## 3. 状态与事件转移表

IDLE 为管理器无活动槽状态，任务记录不回写 IDLE。任务状态只用 ACCEPTED、NAVIGATING、WAIT_HANDOVER、RETURNING、PAUSED、RECOVERING、COMPLETED、FAILED、CANCELED。
nav_phase 独立为 NONE、SEND_PENDING、ACTIVE、STOP_PENDING、UNCERTAIN。PAUSED/UNCERTAIN 是两个字段组合，不增设业务状态 UNCERTAIN。
所有合法变化在同一事务写入状态、event_seq+1 和外部 effect 意图；无变化的重复/迟到事件记录审计结果，不创建重复 effect。非法转换拒绝并记录原因。

| 原状态/事件 | 前置条件 | 新状态/外部意图 | 失败与日志 |
|---|---|---|---|
| IDLE/Submit | 合法新单、无活动槽 | ACCEPTED；仅接纳 | DB 失败不接纳；记录 request/decision |
| ACCEPTED/PrepareOutbound | 站点地图匹配、当前 epoch、无未决目标 | 保持 ACCEPTED，SEND_PENDING；提交 Send 意图 | 持久失败不发；记录 operation/attempt |
| ACCEPTED/GoalAccepted | 匹配当前 attempt | NAVIGATING/ACTIVE；记录 UUID | 迟到接受且已取消须跟踪并精确取消，不能静默丢弃 |
| NAVIGATING/GoalSucceeded | UUID 匹配 | 保持 NAVIGATING/STOP_PENDING；撤销授权 | 缺到达/停止证据不能进交接 |
| NAVIGATING/ArrivalStopped | 到达容差、目标终态、撤销确认、新鲜停止证据全部具备且无取消意图 | WAIT_HANDOVER/NONE | 错 epoch/缺测则 PAUSED；记录证据引用 |
| WAIT_HANDOVER/HandoverConfirmed | mission/handover ID、期望 state_version 匹配且未超时 | 保持 WAIT_HANDOVER，handed_over=true，准备返程意图 | 重复确认返回既有结果；错订单拒绝 |
| WAIT_HANDOVER/ReturnAccepted | 交接已提交、当前返程 attempt 匹配 | RETURNING/ACTIVE | 未知发送结果 PAUSED/UNCERTAIN |
| RETURNING/GoalSucceeded | 当前 UUID | RETURNING/STOP_PENDING；撤销授权 | 不等于任务成功 |
| RETURNING/ArrivalStopped | 返航容差、终态、撤销、新鲜停止、已交接 | COMPLETED，释放活动槽 | 事务失败不释放；记录所有证据引用 |
| 任意非终态/Cancel | 按单写者事务顺序，首次意图获胜 | PAUSED，pending_termination=CANCELED；撤销并精确取消 | Cancel ACK 不是终态；缺 UUID 保持 UNCERTAIN |
| 任意非终态/Fatal 或 HandoverTimeout | 确定失败；超时边界 now>=deadline | PAUSED，pending_termination=FAILED；撤销/取消 | 默认不自动返航；handed_over 保留 |
| PAUSED/TerminationProven | 旧目标已终止或证明从未发送，授权无效且新鲜停止；未产生新运动 | CANCELED 或 FAILED；释放槽 | 任何证据缺失继续 PAUSED，占槽 |
| 任意非终态/Restart | 数据库可读 | PAUSED，未决保留 UNCERTAIN；绝不重发/续租 | 不恢复旧 monotonic deadline；记录新 epoch |
| PAUSED/Resolve | 人工决定与外部目标/停止证据齐全 | 终止；或经显式新授权政策进入 RECOVERING | 人工点击不代替外部证据 |
| NAVIGATING 或 RETURNING/RecoverableFailure | 目标确定终止、停止已证实、类别白名单、预算未耗尽、无取消 | RECOVERING；有限等待 | 不确定状态不可重试 |
| RECOVERING/RetryReady | 全部健康检查、新 attempt、总期限和预算仍有效 | 按去返程记录 SEND_PENDING；接受后回原导航状态 | 预算耗尽转终止处理；禁止自动倒车/旋转 |
| 任意终态/任何业务事件 | 已持久化终态 | 终态保持，无新 effect | Cancel 返回 ALREADY_TERMINAL；迟到结果审计 |

取消意图先提交则后到 SUCCEEDED 仍如实存导航结果，但不继续交接/返航；COMPLETED 先提交则晚取消不改终态。
从未发送、从未授权的新任务取消可使用可审计的 no-motion 证明；不能将此推广为运动过的任务已停车。

## 4. 不变量及时间

I1 活动槽最多一；I2 未决或活动导航尝试最多一（发送未回调也算）；I3 同 ID 不重复外部效果；I4 终态不复活；I5 未知目标/停止不释放槽；I6 重启不自动运动；I7 提交失败不派发；I8 旧身份不能改变新任务；I9 恢复有总预算；I10 交接与到站、停车、返航成功分开记录。
UTC 只作审计；ROS 时间用于 pose/仿真观测；steady_clock 用于进程内期限。不可跨进程相减单调绝对时间。到期 now>=deadline；暂停、回退、进程 epoch 变化使旧证据失效，不自动续跑。

## 5. 持久化边界（待实现）

SQLite C API + 单写者事件循环 + 非阻塞进程排他文件锁。表 missions（ID 主键、规范字段/状态）、events（追加、全局序号）、operations（operation/attempt、意图/结果/UUID）、manager（唯一活动槽、schema version）；FIFO 在 P1-04 增加独立表。
BEGIN IMMEDIATE 中重新读取 ID/槽位并决策；写任务、事件、意图、槽位后 COMMIT，成功才回复/派发。用 RAII 关闭 statement/connection，事务析构 rollback 不抛异常。foreign_keys=ON；journal_mode=WAL、synchronous=FULL 作为候选并读回核对；busy 有界，不关闭日志追求测试速度。并发多进程必须由文件锁拒绝第二任务所有者。
崩溃窗口：提交前无接单；提交后发送前也保守暂停待核对；发送后 UUID 落库前 UNCERTAIN；结果后提交前不重发；终态提交后重读返回。DB+ROS 无原子事务，不承诺 Exactly Once。未知 schema/损坏拒绝写入，不自动删除重建；提交失败有外部活动时保守撤销，日志无法落库要单列 stderr/监督证据。

## 6. 后续共享接口与有限 FIFO

NavigationPort：Ready、Send、CancelExact、Observe；事件带 process_epoch/mission/operation/attempt，已知时带 UUID。ExecutionPort：Begin、Renew、Revoke、StopEvidence；租约带 gate_epoch/owner_epoch/operation/lease/seq。停止证据分 A 授权撤销、B 最终零输出、C 下游收到、D 实际静止；缺 C 标未知，不能由 B 推 C。D 必须同一推进时钟 epoch 内的里程计/实际 pose 窗口，不能以仿真暂停替代。
P1-04 FIFO 等待容量默认 4（不含活动任务），本阶段容量 0 并返回 BUSY。排队状态用独立 queue_state=WAITING，不伪称 active ACCEPTED；同 ID 检查先于满队列判断。排队取消无运动效果并持久化，队列号单调；重启保留顺序但调度暂停，人工恢复且旧活动运动已解析后才升队首。升队和占槽原子提交，旧槽未释放禁止启动下一单。
任务级新导航重试上限 1，总期限不重置；Nav2 BT 内部恢复另记并审查，不能再机械叠加“局部一次+全局一次”。无可靠电量输入标 unavailable，不编造低电量状态。

## 7. 分层验证与边界

P1-01 纯逻辑/数据库；P1-02 实际 ROS Fake Action 协议且 synthetic=true；P1-03 授权纯测/sink，并在合格环境获得独立门控仿真证据；P1-04 真实 Mission 配送；P1-05 全矩阵与一小时目标。实车始终单列未测。
P4 复用本合同的身份、授权和输出证据，再增加延迟估计/历史传播/限幅后重验证；P1 不声称这些研究能力已具备。P0 schema 和历史 evidence 不修改。

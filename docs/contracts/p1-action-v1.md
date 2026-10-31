# P1 Action合同v1（2026-10-31）

状态：实施合同，实际验收结果单列；继续使用p1-mission-v1，不另建ROS状态机。

## API与所有权

MissionCommand服务合并Submit/Query/Cancel/Handover四种操作，必需字段是schema_version、业务字段、稳定command_id、expected_state_version；Query不要求command_id。返回code/state/version/pending/operation/attempt/UUID。响应延后至owner事务完成；入队不是ACCEPTED。提交通过原Store，重复订单返回既有记录。Cancel/Handover命令ID在事件记录内绑定请求内容，重复不增加effect，内容冲突拒绝。

SyntheticEvent为仅测试模式服务，明确标synthetic，独立于Fake Action Server；提供授权撤销确认、停止、到达、Resolve等事件，不由SUCCEEDED自动触发。正式模式当前缺ExecutionPort实现，启动拒绝，不静默启用fixture。

| 资源 | 所有者 |
|---|---|
| FSM/SQLite/effect | Runtime唯一owner线程 |
| ROS Node/Action Client/contexts | 主进程单线程executor，所有客户端操作在该线程 |
| owner→adapter | Port只入有界命令队列，不阻塞网络 |
| callback→owner | 非阻塞Runtime::call，future由wall timer轮询；关键事件保留有界重试，过载锁存禁止新Send并诊断 |
| service response | 延后回调保存request header；owner结果ready后send_response，不在callback中get未就绪future |
| Fake Server | 单独进程，受控多线程executor；延迟接受使用有限屏障，控制服务独立callback group |
| 协议进程 | supervisor独占Popen进程组，有限INT/TERM/KILL，仅自有进程 |

每个Goal context捕获不可变effect的mission/owner_epoch/operation/attempt。response关联UUID，Result再校验UUID。接受事件先入owner队列；提前Result缓存在context内，最多2条；Feedback可合并。反复effect不再Send。迟到接受且cancel_pending必须精确取消。UNKNOWN结果不能当终态；CANCELED原始事实通过独立事件kind记录，不代表物理停止。

## 时间与故障

期限使用steady_clock：readiness、acceptance、execution、cancel ACK、收尾、shutdown分别配置正毫秒数；now>=deadline到期。ROS clock暂停/回退不影响wall timer。已调用Send后异常或超时保持PAUSED/UNCERTAIN，不重发；readiness失败记录NOT_SENT，不能虚构UUID。Server重现不会重新提交effect。客户端重启调用原Store恢复，无自动Send。

本阶段只允许domain72、loopback与/cmr_test/专用action。测试站点：delivery=a、return=b、map=map、station=stations，frame=map，去程(1,0,0)，返程(0,0,0)，单位四元数；这些是合成输入，不读取P0导航资源。所有候选请求显式版本校验。

关闭：拒新业务/Send→owner提交Cancel与撤销→executor继续处理有限收尾→失联保存暂停→标回调不再投递→stop_callbacks→owner排空退出→销毁Node/Client→context shutdown。主线程拥有executor，无detach线程；外部结果未知不宣称停车。

## 开源依据与代码边界

Nav2 waypoint_follower.cpp，jazzy d5d768b7b38a05e7cfd2b6c693b786b402161bed，Apache-2.0；参考UUID匹配和生命周期，不照搬cancel_all或阻塞future。
YASMIN action_state.hpp，abbe58d9511a79618023a530870f7d1794a829b7，Apache-2.0；参考晚接受/epoch/回调guard，不采用通用框架等待重试。
ROS2 rclcpp_action/client.hpp、create_client.hpp、rclcpp/any_service_callback.hpp以本机28.1.22头文件为准，Apache-2.0；直接使用公共API。官方jazzy test_client.cpp的真实服务端/客户端协议测试用于参考，源码blob 4371f48a0e46537a4d8bd215d8c6833b620993de。CMR适配与Fake独立实现，不拷贝上游实现；只链接Jazzy成熟库。

## 实施补充

command_id作用域为mission_id，Submit幂等键仍为mission_id；Cancel/Handover的操作名与expected_state_version作为稳定内容落在原events中，重复查询先比较内容再返回，不新增effects。Event.detail追加审计文本，Record编码与数据库user_version保持1，P0 schema不变。新增CanceledResult仅追加事件枚举，不增业务状态。

取消应答期限cancel_ms与终态收尾collect_ms分开配置；执行总期限execute_ms自本次async_send_goal计时（包含接受等待），readiness另计。Shutdown有限协议收尾窗口1600ms，不把其等同硬实时进程退出或停车上界。

回调关联遇到重复终态不改状态；矛盾终态产生CONFLICTING_TERMINAL审计事件交回owner，按Feedback无状态效果落库。超出有界关联容量锁存故障。高频反馈可丢弃，关闭日志输出feedback_dropped计数。

接受/结果回调执行时也检查单调期限，不能依赖5ms tick先于回调运行。只有关联校验通过的终态才停止该Goal的期限监控。旧operation/attempt的Fatal在owner入口降为STALE_OPERATION_DIAGNOSTIC审计，不得暂停当前返程；旧epoch仍由原合同拒绝。SyntheticEvent的每条任务事实也先核对epoch/operation/attempt/UUID，owner_pause是单独的测试故障开关。

## 字段、错误码与测试控制

业务标识沿用S1：1–64字节ASCII `[A-Za-z0-9_.-]`；schema_version=1、created_at_unix_ms>0。UTC毫秒只作审计，时间变化不构成同ID业务冲突；期限使用本进程steady毫秒，不跨进程比较绝对值。expected_state_version为uint64，交接沿用原FSM版本检查。Query不制造IDLE；不存在返回NOT_FOUND。

| 场景 | code |
|---|---|
| Submit事务提交/相同业务/同ID冲突/槽忙 | ACCEPTED / EXISTING / CONFLICT / BUSY |
| 不合法请求/站点版本/操作/命令ID | INVALID / INVALID_STATION_VERSION / INVALID_OPERATION / INVALID_COMMAND_ID |
| Query存在/不存在 | FOUND / NOT_FOUND |
| Cancel挂起/交接已提交/已终态 | CANCEL_PENDING / CONFIRMED / ALREADY_TERMINAL |
| 同command_id不同内容/不合法阶段版本 | COMMAND_CONFLICT / REJECTED |
| 数据库异常/邮箱容量/关闭拒绝 | STORAGE_ERROR / OVERLOADED / CLOSED |

Submit重试使用同mission_id；Cancel/Handover使用同mission_id+command_id及相同expected版本。服务客户端自身超时不能推断事务回滚，必须重试/查询。detail为诊断，不能用其文本解析替代稳定响应字段。

Adapter默认readiness/accept/execute/cancel ACK/terminal collect预算分别为1500/1500/10000/1000/3000ms，必须均为正；supervisor按case冻结更短测试值，见trace.json启动参数。owner默认容量16，配置允许1–256；Adapter命令32、关键事件64、早到关联2，邮箱满有明确拒绝/保守故障路径。关闭协议窗口固定1600ms。

FakeControl位于本案namespace的control服务；command支持reject、reject_cancel、delay_accept、release_accept、cancel_delay、success、abort、stats。value_ms用于开关或0–5000ms延迟；case_id用于原始日志关联，goal_uuid指定目标。空UUID表示本案当前活动Goal（不是跨namespace全局取消），仅用于受控单活动测试；监督器与独立审计仍检查所有实际Goal请求。success/abort仅在找到目标时ACK success=true；ACK观察时间由trace的time_monotonic记录，Server日志另给实际CONTROL/ACCEPT_BARRIER/RESULT顺序，不能当物理实时上界。故障退出通过监督器SIGKILL本轮自有进程，不提供任意PID控制服务。

关闭补充：closing为跨executor/owner的atomic标记；入队但尚未开始的业务在owner检查后返回CLOSED，不形成Mission/Send。NavigationAdapter::begin_shutdown先禁止新的发送，未派发的上下文记录NOT_SENT SHUTDOWN；已有Goal仍精确取消/收尾。关闭用Store::active()在owner查询持久活动槽，不依赖API响应是否已发；关闭入队遇邮箱满会在有限窗口内重试。A18有实际反例和修复后的Release/ASan复测。

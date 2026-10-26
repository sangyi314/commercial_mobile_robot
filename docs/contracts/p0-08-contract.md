# P0-08 冻结合同

本阶段为仿真设施重复性与受控失效验收。复用原 runner，调度器只串行调用它；不改控制器、原 profile、P0-06/07 资产或历史记录。用户授权由 AI 工程审查后继续，个人理解/抽查/目视保持待确认。

正常矩阵 20 槽位，10 对；奇数对 MPPI→DWB，偶数对 DWB→MPPI。故障独立 6 槽位，controller-deactivate、short-deadline、pause-gazebo 各 MPPI/DWB。所有回合新进程、新 run_id、新 partition、新 epoch，保留第一次与补采的全部尝试。不会为获得十次成功隐藏失败。

正常 profile 继承 P0-04/05 的世界、地图、出生位姿、初值、目标、两基线和公共约束。新 profile 加入原父进程内 Gazebo Transport 的实际 pose/stats 记录和冷启动检查；这仅作记录，不连接 Nav2 输入。不具备静态世界完整几何评价 scope，本轮 P0-07 评价 not_evaluated、collision=null。

故障资格：READY、唯一 UUID 被接受、无终态、controller active、新鲜传感器，接受后至少三个推进的 odom 样本、净位移≥0.05 m、最新线速度≥0.01 m/s。20 秒内不成立或目标提前结束为 NOT_EXERCISED。F-C 单次 deactivate 后查询实际状态，观察自然反应 2 秒，必要时取消。F-T 从资格成立起 0.5 monotonic 秒到期。F-P 暂停 4 墙钟秒，3 秒内要求 INPUT_STALE；本轮 partition 的实际 stats.paused 和新鲜同源暂停样本证明激活。父 pump 持续调度恢复，不依赖 ROS 时间或 cancel 完成；恢复服务≤1.5秒，随后确认推进与新鲜输入。

work 360、cleanup 60、finalize 30 秒保持。故障资格/自然反应/取消/恢复/停止均嵌套于 work，停止独立保留 30 秒，启动最多180秒、正常目标120秒。调度器不使用短外部 timeout 杀 runner；超过450秒+60秒外层观察界只报警和停止派发，不能据此重复派发或杀其他进程。

异常停车不要求到达或 SUCCEEDED。取消响应、真实终态、最终零命令、odom 停止分别记录。停止在恢复/取消后推进的仿真时间内：至少3秒、5样本、间隔≤0.5秒、v≤0.01、abs(wz)≤0.03、最大两两位置漂移≤0.02m；三段最终 abs(vx)/abs(wz)≤0.001。无数据不补零，暂停静止不算停车。未能证明停止仍保留原紧急清理后备，故障用例不通过。F-C 清理按实际 GetState 选择合法 shutdown，不关闭 bond/保护。

batch 使用独立锁；runner 保留原项目锁。意图 fsync 后才派发，journal 追加序号与链 hash。截断、未知/未决身份、输入或执行代码漂移都停止。恢复先核对旧 runner PID/start_ticks/boot/token 与后代、旧目录/结果/清理；锁空闲不等于进程消失。完成槽位不重跑；失败须显式原因恢复，重复首因两次拒绝盲重试。代码/profile/阈值变动必须新 cohort，旧记录不能凑最终数量。

汇总将计划、意图、实际调用、启动前拒绝、真实 Gazebo 启动、有效冷启动、READY、accepted、各终态、完整通过分开；正常、故障、fixture、版本分开。ledger 与归属标注的目录双向对账。VALID 不等于行为通过；Action SUCCEEDED 不等于完整正常通过；故障预期非零不是正常通过。


## 真实证据触发的冻结修订
- r01 官方静态世界没有 pose/info 发布；新增 profile 内派生世界仅添加官方 SceneBroadcaster，其他 XML 语义逐项相同。旧世界不改。r02 起采用该观测资源。
- r02 DWB F-C 清理竞态：由实际 manager 稳定状态协调 shutdown，子预算25秒嵌套原cleanup60；bond和自动恢复不改。
- r03/r04 DWB 成功后数秒定位变化导致最后样本超差。r05 的新 profile 固定到达取样为：源时间在 [终态,终态+0.5仿真秒] 内且新鲜的首个定位样本，不按误差择优；旧 profile 保持 last_fresh。误差阈值仍来自实际 goal checker；后期位置误差另报，不能据此声称最终定位一直在容差内。旧失败保持原结论。停车仍独立观测至少3仿真秒。
- 首因比较去掉事件时间戳，按 code/phase/detail 比较，避免重复故障因新时间戳绕过停止门。

## 当前长路线cohort交接（2026-10-30）

当前已迁移的p0_long_route_v1：map起点(-2.0,-0.5,0)，目标(1.75,-0.5,0)。原文继承条件按当前冻结矩阵解释；旧短目标r05不混入本批。运行中未改采样规则、0.25m阈值、控制器或输入。

正常21次尝试（MPPI11、DWB10），20次完整通过，1次F002保留失败；故障6次有效用例全部通过。见 [验收报告](../verification/p0-08-long-20261029-3cff94.md)。个人理解、抽查及阶段放行待本人确认。

# P0-03 p0-03-nq1hmjaf：真实失败回合已归档

日期：2026-10-22（Asia/Shanghai）。工作区 `/home/robot/commercial_mobile_robot`，run=`/home/robot/commercial_mobile_robot/evidence/runs/p0-03-nq1hmjaf`。

**工程结论：本轮导航未通过。唯一目标被 inactive Action server 拒绝，没有重发目标或重新运行仿真；本轮资源已经释放，但关闭存在异常，cleanup=FAILED。真实 result.json 已生成并通过 record_contract 校验，VALID/0 不代表导航成功。**

## 证据表

| 检查 | 本轮真实证据 / 结论 |
|---|---|
| 目标 UUID / Action | 无 accepted UUID；唯一请求被明确 REJECTED，action_success=false；CLI exit=0 不代表成功 |
| 动作期间 odom 位移 | 未知；dispatch 至 return 无观测。原 observer 提前 328.672130532 秒结束，不能填零或声称达到 0.1 m |
| 后续 map 位姿 | x=-1.99480618494453 m，y=-0.450261128866438 m，yaw=0.0154153854842457 rad；stamp=5504.877 |
| 定位估计误差 | 位置 2.06113929010702 m > 0.25 m；角差 0.0154153854842457 rad <= 0.25 rad。采于拒绝后 330.571445366 秒，非即时终态、非独立物理真值 |
| 最后 /cmd_vel | 两个观察窗口均没有样本；未知，不补零 |
| 补充静止窗口 | 仿真 5490.693–5505.057 s，跨度 14.3639999999996 s；395 有效样本，最大间隔 0.144000000000233 s |
| 最大速度 / 漂移 | 线速 2.91167575618666e-20 m/s、角速 1.0452642351425e-19 rad/s、最大漂移 1.0045281358844e-19 m；仅支持清理前静止 |
| 新鲜度 / clock | odom stamp 与接收时邻近 clock 最大差 0.417000000000371 仿真秒；clock 接收最大距离 0.138588958000582 墙钟秒。两个窗口无回跳记录，中间空白未知 |
| 成功后停止 | null：目标没有成功、没有最终零指令；补充静止不冒充成功后停车 |
| 清理 | FAILED：本轮资源已全部释放、nodes-after exit=0/列表为空，但 RViz exit=-6、launch 删除临时 SDF 报错 |


原始 [observations.jsonl](../../evidence/runs/p0-03-nq1hmjaf/runtime/observations.jsonl) 与 [补充观察](../../evidence/runs/p0-03-nq1hmjaf/runtime/observations-cleanup-20261022.jsonl) 分开保留；[最终离线计算](../../evidence/runs/p0-03-nq1hmjaf/runtime/evidence-summary-final-20261022.json) 含首末位姿、最大偏移、完整精确值、所有输入 SHA256 和两窗口计数。可复查脚本为 [analyze_smoke.py](../../evidence/runs/p0-03-nq1hmjaf/artifacts/analyze_smoke.py) 与 [最终评定包装脚本](../../evidence/runs/p0-03-nq1hmjaf/artifacts/analyze_smoke_final.py)，仅离线读取日志，不导入 ROS 或发布消息。

## 实际执行、原始记录与时间范围

进入现场时原 observe.py、240 秒观察文件、唯一目标日志与 dispatch/return 已经存在，原观察器已退出。原目标由此前终端操作发送；AI 继承它，没有发第二个目标来覆盖拒绝。已有 [客户端日志](../../evidence/runs/p0-03-nq1hmjaf/logs/navigate-goal.log) 明确 Goal was rejected，且没有 accepted UUID；[服务端启动日志](../../evidence/runs/p0-03-nq1hmjaf/logs/nav2-launch.log) 明确 Action server is inactive. Rejecting the goal。

AI 本次实际完成：只读节点/生命周期/Action status 复核，另起一个仅订阅的 15 秒补充 observer 并等待其自然退出，离线计算证据，按已核实归属的 PID/start_ticks 对本轮 Nav2→Gazebo 发送 SIGINT，整理记录与文档。之后仅在负责人明确确认事实并授权后修改记录合同，不再运行导航。

原采集结束单调时刻 6954.684752690，发送前 7283.356883222，CLI 返回 7285.176936098；CLI 窗口为 1.82005287599986 s，但窗口内无观测。补充采样为拒绝后的清理前时段，不据它补全缺失的动作期间数据。

补充里程计使用指南阈值：跨度>=3 仿真秒、>=5 个前进样本、最大间隔<=.5 s，线速<=.01 m/s、角速<=.03 rad/s、漂移<=.02 m。clock 按接收单调时间取最近记录，保留 stamp 差与接收间距；实际最大差小于 .5 仿真秒，没有放宽阈值。该窗口满足里程计静止条件，但不等于导航成功后的控制系统停车。三段 Twist 都未收到，不能把空消息当零。

最后 TF 使用清理前的新鲜记录，底层 odom 持续更新，TF 唯一 stamp 数 67，最后 TF 比节点 ROS now 早 0.0330000000003565 s；四元数有限且单位范数误差在 1e-6 内。没有用重复旧 TF 充样本。位置是 AMCL 链的估计，不是世界真值；RViz 人工移动观察没有独立记录，不代签。

## 首个失败原因及关闭异常

启动日志先记录 global_costmap 等待 base_link↔map TF 激活超时，lifecycle manager 中止 navigation bringup。后续虽然记录了 AMCL 初值和 map TF，实际 navigator 等节点仍 inactive，唯一目标被拒绝。Action server 存在不表示它 active；本次不擅自激活或调参，也不在同 run 重试。

清理按 Nav2 launcher 4079、后 Gazebo 3958 执行，发送前核对环境中 CMR_RUN/GZ_PARTITION、命令和 start_ticks；子进程已退出，nodes-after 查询成功且为空。保留 shell 和 ROS CLI daemon，未全局 kill。原 observer PID 12459 已不在，没有按旧 PID 发送信号；本次自持的补充 observer 13873 自然 exit=0。

关闭后 nav2-launch.log:2167–2168 显示 RViz exit=-6 与 launch 临时 SDF 删除失败，故最终 cleanup=FAILED，而不是正常清理通过。[最终清理评定](../../evidence/runs/p0-03-nq1hmjaf/runtime/cleanup-final-assessment-20261022.json) 覆盖先前只检查资源消失的 SUCCEEDED 判断；[初始元数据](../../evidence/runs/p0-03-nq1hmjaf/runtime/cleanup-evidence-20261022.json)、[事件日志](../../evidence/runs/p0-03-nq1hmjaf/logs/cleanup-20261022.jsonl) 与各版候选原样保留。首次 /proc 枚举权限错误发生在信号前，错误说明也保留，重试仅检查当前用户进程。

[session-ended.json](../../evidence/runs/p0-03-nq1hmjaf/runtime/session-ended.json) 已记录：boot_id 与起点相同，elapsed_monotonic_s=6771.903889646。其范围为包含人工调查与清理的手工会话，不是算法耗时。本轮开始未取得 /clock，所以 manifest/result 的 sim_start_s、result.sim_end_s 与 rtf 均为 null，未把后来时钟或其他窗口比例当会话 RTF。

## 负责人授权的最小合同修订与正式结果

真实组合为 startup=FAILED、action=REJECTED、harness=STARTUP_FAILED、cleanup=FAILED、action_success=false。旧 v1 validator 只允许 FAILED+NOT_SENT，会迫使记录掩盖已发送请求。先保存了 [具体冲突](../../evidence/runs/p0-03-nq1hmjaf/artifacts/result-contract-conflict-final.json) 和 [旧检查失败](../../evidence/runs/p0-03-nq1hmjaf/logs/result-proposed-final-check.json)，未伪造 READY 或 NOT_SENT。

负责人随后明确回复“确认事实，授权最小修订及回归”。按已展示的 [最小补丁](../../evidence/runs/p0-03-nq1hmjaf/artifacts/proposed-startup-rejected-contract.patch) 仅修改三个文件：tools/validate_run.py 的一条 startup/action 条件、experiments/schema.md 的对应规则、tests/test_run_schema.py 的七例回归。允许 FAILED+REJECTED/false，保留其他非法组合拒绝。JSON Schema、核心、所有导航配置均未改。

[修改前备份](../../evidence/runs/p0-03-nq1hmjaf/artifacts/contract-before-20261022.tgz)、[修改后快照](../../evidence/runs/p0-03-nq1hmjaf/artifacts/contract-after-20261022.tgz)、[授权与前指纹](../../evidence/runs/p0-03-nq1hmjaf/runtime/contract-revision-before.json) 均保留。准确验收目录为 runtime/contract-check-3we6en06：语法/三个 Schema 元校验通过，完整 [回归日志](../../evidence/runs/p0-03-nq1hmjaf/runtime/contract-check-3we6en06/regression.log) 和 [JUnit](../../evidence/runs/p0-03-nq1hmjaf/runtime/contract-check-3we6en06/regression.xml) 为 **122 项、0 failure/error/skipped**；[实际测试文件 SHA256](../../evidence/runs/p0-03-nq1hmjaf/runtime/contract-check-3we6en06/regression-summary.json) 绑定本次源码。没有依赖升级或仿真重测。

正式 [result.json](../../evidence/runs/p0-03-nq1hmjaf/result.json) 已按负责人确认的失败事实创建。运行参数取 runtime/controller_server.yaml 的实际 nav2_mppi_controller::MPPIController；误差字段保留清理前估计范围；stop_observed/intervention/delivery_success=null；独立碰撞评价未执行，coverage=none、evaluation_valid=false、所有碰撞/接触/间距测量 null。首个 primary_reason 仍为启动 TF 超时及 inactive 拒绝，关闭异常不覆盖它。

真实 CLI 未加 --allow-fixtures，结果 [VALID/0](../../evidence/runs/p0-03-nq1hmjaf/logs/run-final-check-after-contract.json)，record_origin=real、validation_scope=record_contract。它只证明字段和非空引用/hash 一致，**不改变本轮导航失败与清理失败结论**。较早 [INCOMPLETE/3](../../evidence/runs/p0-03-nq1hmjaf/logs/run-final-check.json) 和旧合同 INVALID 日志保留，不抹掉历史。

## 保护范围、交接和未完成项

[授权后保护复核](../../evidence/runs/p0-03-nq1hmjaf/runtime/final-protection-after-contract-20261022.json)：运行开始的 12 项保护记录中仅 validator 因明确授权改变；合同说明和测试的授权变更另有三文件备份/回归指纹。其余保护项、三份 JSON Schema、核心/CMake/package、三份 Nav2 YAML、make_baselines.py 及冻结输入均保持原字节。未 reset/clean、暂存、commit 或 push；原 .vscode 改动保留。

manifest 仍绑定运行开始时的源码和 source-at-start.tgz；事后记录合同修订另以快照与回归来源归档，没有重写 manifest 或假称使用新工具重新导航。manifest SHA256 为 `0e87af927931b13f259b24a056365b4355d6d0de5970148825593fe2e9fdf74e`；result SHA256 为 `bf5c8714200ecfb8c243b06b9734afe21fc687a4ec444f78205f89454ffc9e71`。

[数据流](../dataflow.md)、[冒烟说明](../navigation_smoke.md)、[P0-02 交接补充](p0-02-RaLo7r83-handoff-20261022.md) 已增量整理；P0-02 原草稿未改，官方通用场景与本 run 冻结场景均保留 draft/pending。负责人已确认本轮事实和合同修订，但个人推导、独立排故、RViz 观察熟练度没有代打勾。

下一轮仍需先解决启动时序、确认所需节点 active，并保证观察器在发目标前仍采样；另建 run，不覆盖本轮拒绝，不在此目录发第二目标。本次未完成成功导航、动作期间移动或成功后停止的通过验收，没有独立碰撞/实车/生产安全结论。

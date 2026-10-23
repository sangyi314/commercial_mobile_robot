# P0-04 MPPI / DWB 双基线实施与验收

用户工作日期：2026-10-24，Asia/Shanghai。工程结论：**通过**。个人理解与目视观察未代签。

实际项目 `/home/robot/commercial_mobile_robot`，HEAD `3137b0b51e761e360203fa0a9a163a48335541d2`。无项目 AGENTS.md；本地 sources 只读。源码、原配置、Schema、validator 与开始时已有证据已做前后指纹核对。

- MPPI：`/home/robot/commercial_mobile_robot/evidence/runs/p0-04-mppi-hhmvi_i0`
- DWB：`/home/robot/commercial_mobile_robot/evidence/runs/p0-04-dwb-76fqnpi5`
- 配对索引：`/home/robot/commercial_mobile_robot/evidence/env/p0-04-pair-q2MXrdh3/pair.json`

## 验收结果

| 检查项 | MPPI | DWB | 原始证据 |
|---|---|---|---|
| 实际插件 | nav2_mppi_controller::MPPIController | dwb_core::DWBLocalPlanner | runtime/controller_server.yaml、logs/runtime-plugin.out、logs/navigation.log；具体行号见 raw-evidence-locations.json |
| 发令前状态 | 所有保存的关键 lifecycle active | 所有保存的关键 lifecycle active | runtime/pre-goal.json、dispatch-readiness.json；Action ready、clock/scan/odom 原样样本和 pre-dispatch-input-freshness.json；不使用事后查询替代 |
| UUID | b95904295fb345dfa735767608d7848a | 392920a92f94406ea2b867643bba9363 | logs/action-client.jsonl 接受与终态行号见 raw-evidence-locations.json；observations 同 UUID status=4 |
| Action 终态 | SUCCEEDED | SUCCEEDED | logs/action-client.jsonl、runtime/goal-terminal.json；未使用 CLI exit=0 替代 |
| odom 净位移 m | 0.799916 | 0.792335 | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| odom 最大偏移 m | 0.799916 | 0.793098 | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 最终定位误差 m / rad | 0.237209 / 0.090957 | 0.241234 / 0.238600 | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 停止仿真时间窗 | 26.532–31.500 s | 26.532–31.500 s | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 停止时长 / 样本 | 4.968 s / 139 | 4.968 s / 139 | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 最大样本间隔 | 0.036 s | 0.036 s | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 最大线速度 / abs(wz) | 0 / 0 | 0 / 0 | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 最大两两位置漂移 m | 0 | 0 | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| odom 最大时间年龄 | 0.003000 s | 0.000000 s | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 最后 cmd_vel vx / wz | 0.0 / 0.0 | 0.0 / 0.0 | runtime/observations.jsonl；离线复算 independent-offline-evidence.json，原始行号见 raw-evidence-locations.json |
| 实际 goal checker | SimpleGoalChecker，stateful=true，0.25 m / 0.25 rad | 同左 | runtime/controller_server.yaml；末次新鲜 map→base 估计误差分别核对 |
| 无效/缺测/回跳 | 停止窗无无效样本、无时间回跳 | 同左 | independent-offline-evidence.json；stamp严格递增，clock同步前进；不把缺消息填零 |
| 清理 | SUCCEEDED | SUCCEEDED | runtime/cleanup-evidence.json、logs/session-events.jsonl、Gazebo stop 服务与实际 exit；owned groups为空；所有目标均先结束且停车窗口在关闭前 |
| 真实合同校验 | VALID / exit 0 | VALID / exit 0 | logs/run-final-check.json / .exit；未加 --allow-fixtures；VALID仅表示记录合同 |
| 独立碰撞评价 | 未做；collision=null | 未做；collision=null | result.json evaluation.coverage=none / evaluation_valid=false；Collision Monitor不是独立真值 |

## 公共条件与合理差异

相同：官方 world/map/robot 及模型依赖指纹；Ogre1 软件渲染；出生位姿 (-2,-0.5,0.01,0,0,0)；初值 pose/covariance（仅时间戳不同）；目标及空 behavior_tree；安装包版本；定位、规划、局部/全局代价地图、目标检查器、速度平滑器及近场保护的实际公共设置。两轮 Gazebo 同用进程专用 Boost 预加载。

运行限值语义映射：vx∈[-0.1,0.3] m/s、vy=0、abs(wz)≤1 rad/s；ax∈[-1,1] m/s²、ay=0、abs(az)≤2 rad/s²。运行值逐项一致，详见 final-runtime-semantic-comparison.json。

公共参数扫描的差异：`[{"node": "controller_server", "parameter": "controller_server.verbose", "mppi": false, "dwb": null}]`。`controller_server.verbose=false` 是 MPPI 参数处理器的日志开关，DWB 未声明；安装头文件 `/opt/ros/jazzy/include/nav2_mppi_controller/tools/parameters_handler.hpp:236–244` 显示它控制参数日志，属于合理差异，不是公共运动约束差异。

DWB 实际 trajectory generator：`dwb_plugins::StandardTrajectoryGenerator`。MPPI `regenerate_noises=False`；critic、轨迹采样/优化参数各自保留。50×0.05=2.5 s 与 DWB sim_time=2.5 s 不表示计算预算或算法行为相同。私有差异完整保存，不作排名。

run_id、分区、时钟 epoch、初值 stamp、UUID、回合路径和场景 revision 各自独立；它们不同符合冷启动回合语义。未发现本次已核对公共条件的实质不一致；这不等于所有隐藏因素都已证明相同。

## P0-03 交接与失败保留

已读 Codex chat「排查并修复 P0-03 导航失败」的实际完成回复和工具输出；成功 UUID 13148642f3b84392ad7d64ef7d4d734c 已完成功能验证。但交接明确没有正式 manifest/result，故不复用为本次完整 MPPI 回合。已解决 UDP 通信、定位先于导航和 launch 信号转发问题未重新排查。详见 p0-03-handoff.md。

本次较早的 MPPI / DWB 真实回合均导航成功，但 Gazebo stop 服务返回 data:true 后仍 SIGSEGV / exit=-11；原 result 继续保持 cleanup=FAILED。关闭诊断捕获 Sensors 线程退出的 TSD 析构地址 0xffffcc01cba0，并与关闭前 libboost_thread.so.1.83.0 映射对应。Gazebo 专用 LD_PRELOAD 使该库保持加载，独立关闭检查实际 exit=0；两方法均另建完整回合验证，不把关闭检查冒充导航回合。

本次准备错误也保留：p0-04-mppi-8w2ri1i8 在创建仿真前因将新增证据误判源码变化而停止；cnr5xdgs 错用了 validator 子命令；zkxon7rx 被 UTC Z 后缀规则拦下，两者均 NOT_SENT 且原 manifest 未重写。其 INVALID/2 诊断保留，不用新成功覆盖旧错误。

已知历史失败、未完成目录和本次全部尝试见 attempt-index.json；仅两个 engineering_accepted=true 项是最终基线。前后指纹核对只证明开始清单中已有文件未变，不能证明历史从未被删除。

## 解释与结论边界

odom 证明里程计运动；AMCL/TF 证明定位估计到达；预检 Gazebo 独立位姿只用于发令前定位合理性检查，未作全程独立物理到达或碰撞真值。仿真停止发生在已采集的有效停车窗口之后。

旧 docs/navigation_smoke.md 与 p0-03-nq1hmjaf 报告描述各自历史失败，不能当作当前 P0-03 全局状态；本次没有改写它们。早期停止服务成功但 exit=-11 说明服务应答不能替代实际清理证据。MPPI 第一版收尾脚本的控制台异常文本扫描曾把正常 signal_handler(SIGINT/SIGTERM) 描述也列入候选；清理失败仍由实际 Gazebo -11 独立成立，v2已收紧匹配，旧 result未覆写。

原始日志离线重算，不从 result.json 回读指标。分析器在独立临时目录的缺 odom、缺 cmd_vel、clock 回跳、停止间隙、无效数值、非零停止速度六种反例均拒绝；反例仅测试分析器，不作为真实运行证据。

保护核对：65 个源/工具/合同文件，变化 0；开始时 966 个历史证据文件，变化/缺失 0。未自动提交、push、reset或clean；未改原配置、核心、Schema、validator、历史输入和日志。

仅证明两种插件在本次官方静态短任务上的启动、导航、定位估计到达、连续停止和清理冒烟验证。不推导性能排名、成功率、实时性、独立无碰撞或实车安全结论。

## 放行与个人确认

工程证据放行：**满足**。个人理解与目视观察仍待本人确认。后续是 P0-05 单回合自动化，不是批量性能实验。

请本人确认：①文件名为何不能证明控制器身份；②运行参数、active、实际运动分别证明什么；③两种2.5秒预测长度为何不是相同预算；④CLI exit=0与REJECTED为何可同时出现；⑤公共输入应相同而回合身份应变化；⑥goal checker、零命令、停止观测与碰撞真值为何分开。未代签 RViz 目视移动或上述理解项。

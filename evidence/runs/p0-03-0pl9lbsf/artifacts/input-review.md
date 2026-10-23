# P0-03 本轮输入审查

整理日期：2026-10-24（Asia/Shanghai）。回合：p0-03-0pl9lbsf。

仅整理现有输入。未启动/停止 ROS、仿真或观察进程，未发初值/目标，未改源码、原配置、Schema 或 validator。

## 关键字段

- scenario_id 保持 official_static；revision=r2+p0-03-0pl9lbsf；status=draft。
- 目标 map：x=-1.63612 m，y=1.11395 m，yaw=0.0 rad；四元数模=1.0；behavior_tree 保持原文件。
- 原始初值按负责人“尚未记录，不要管”的补充保持 initial_pose=null；不伪造初值引用，不用出生参数或 AMCL 输出替代。
- source 取原始 repo-at-start：commit=cf4b05a1ba27f0f6e69026b0dda0438ee14e526a，dirty=false，snapshot_ref=null。
- clock_epoch=p0-03-0pl9lbsf:sim0；started_at_utc=2026-10-23T07:33:48.271176Z；sim_start_s=null。
- expected_plugin 是静态 MPPI 意图，本次未验证实际加载或生命周期。
- 三类主要 ArtifactRef 各一份；全部引用位于本 run 内且 hash 按实际字节。

## 公共约束

| 来源 | 值 |
|---|---|
| velocity_smoother.max_velocity / min_velocity | [0.3, 0, 1] / [-0.1, 0, -1]（m/s, m/s, rad/s） |
| velocity_smoother.max_accel / max_decel | [1, 0, 2] / [-1, 0, -2]（m/s², m/s², rad/s²） |
| controller_frequency | 20 Hz |
| general_goal_checker | SimpleGoalChecker，stateful=true，0.25 m / 0.25 rad |
| local/global robot_radius | 各 0.22 m |
| local/global footprint、footprint_padding | 本配置未显式设置；value=null |

完整来源键见 [common_constraints.json](common_constraints.json)。本记录不是两控制器公平调参完成的证据。

## 校验

| 检查 | 实际结果 |
|---|---|
| scenario | VALID，退出 0 |
| run | INCOMPLETE，退出 3；缺 result.json，未生成结果 |
| scenario --require-ready | NOT_READY，退出 4 |
| 世界变体 | 实际/官方来源 hash 一致；XML 比较仅渲染引擎变化 |
| 原输入、保护文件、Schema/validator/合同 | 整理前后 hash 未变 |

机器可读诊断、完整文件清单、限制及 manifest hash 见 [input-review.json](input-review.json)。
场景保持 draft，四项 verification 仍 pending，dependency_lock=null。缺初值、全局对齐和完整接口证据；上述校验不表示可以直接执行导航。
当前安装版本见 [environment-current.json](environment-current.json)，仅证明本次读取的当前环境。

## 新增文件

- manifest.json
- artifacts/scenario.yaml
- artifacts/common_constraints.json
- artifacts/environment-current.json
- artifacts/input-review.json
- artifacts/input-review.md
- artifacts/input-freeze.json

冻结仅覆盖 manifest 及其实际引用的现有输入。后续若补录初值或改变输入，应重新审查并形成新的输入版本；本快照不会自动吸收新内容。

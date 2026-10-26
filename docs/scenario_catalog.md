# 场景与输入目录

状态核对日期：2026-10-30。此页是当前文档的唯一场景/输入导航；不替代文件内容、冻结哈希或Runner预检。历史实验使用的输入保留在各Run与报告中。

## 长路线任务

任务版本p0_long_route_v1，来源[任务冻结文件](../experiments/goals/p0_long_route_task_v1.json)及[目标请求](../experiments/goals/p0_long_route_v1.json)：map起点(-2.0,-0.5,0)，目标(1.75,-0.5,0)，目标四元数(0,0,0,1)。公共约束与资源绑定以实际Profile为准。

## 当前工作树入口

| 用途 | Scenario | Profile | 状态与边界 |
|---|---|---|---|
| 静态单回合，MPPI/DWB | [static](../experiments/scenarios/p0_long_route_static_v1.yaml) | [static profile](../experiments/profiles/p0_long_route_static_v1.yaml) | 文件status=draft；现有Runner按该静态任务分支核验冻结基线和任务。本页不把字段改称ready；先走[Runner预检](guides/runner.md) |
| GUI静态观察 | [observe](../experiments/scenarios/p0_long_route_observe_v1.yaml) | [observe profile](../experiments/profiles/p0_long_route_observe_v1.yaml) | 文件status=draft；派生world含SceneBroadcaster；[观察流程](guides/gazebo-observation.md)。没有自动横穿道具 |
| 新布局S1横穿模板 | [crossing v2](../experiments/tool_layout_v2/scenarios/crossing.yaml) | [crossing profile v2](../experiments/tool_layout_v2/profiles/p0_06_crossing.yaml) | draft，review/probe=null，未真实探测；不能直接当正式导航输入 |
| 新布局S3已观测遮挡模板 | [occlusion v2](../experiments/tool_layout_v2/scenarios/observed_occlusion.yaml) | [occlusion profile v2](../experiments/tool_layout_v2/profiles/p0_06_observed_occlusion.yaml) | 同上；[动态准备流程](guides/dynamic-scenes.md) |
| 新布局通用probe模板 | [probe v2](../experiments/tool_layout_v2/scenarios/p0_06_probe.yaml) | [probe profile v2](../experiments/tool_layout_v2/profiles/p0_06_probe.yaml) | 仅准备入口，不是已运行结果 |
| 正常冷启动矩阵 | [normal matrix](../experiments/p0_smoke_matrix.yaml) | 矩阵各slot的冻结引用 | 20槽位；新代码必须新cohort，不能恢复旧源码指纹 |
| 受控故障矩阵 | [fault matrix](../experiments/p0_fault_matrix.yaml) | 矩阵各slot的冻结引用 | 独立6槽位；故障通过不计普通导航成功 |

动态模板的新旧SHA对应见[布局v2说明](../experiments/tool_layout_v2/README.md)与layout.json。当前动态记录只验证过MPPI；脚本位姿运动不是行人动力学、tracking或未知目标预测。

旧Probe候选已按依赖审计移出当前树；[实验输入说明](../experiments/README.md)提供逐文件分类与验证恢复方法。当前保留输入与冻结SHA不变。

## 历史正式输入与证据

旧[crossing](../experiments/scenarios/crossing.yaml)/[profile](../experiments/profiles/p0_06_crossing.yaml)及[observed_occlusion](../experiments/scenarios/observed_occlusion.yaml)/[profile](../experiments/profiles/p0_06_observed_occlusion.yaml)保存历史ready_for_recording声明和准备来源，但绑定了迁移前源码路径。当前工作树预检会拒绝；不能因旧字段是ready就认为新布局可执行，更不能改旧锁让其通过。

- 官方短静态双基线：[P0-04报告](verification/p0-04-p0-04-pair-q2MXrdh3.md)。
- 原动态四轮：[P0-06报告](verification/p0-06-kwdqt70u-live.md)与[P0-07评价](verification/p0-07-4v9sd2n4.md)。
- 长路线静态、两个probe、四轮动态和声明范围内几何评价：[联调报告](verification/long-route-v1-20261029.md)。
- 长路线P0-08：[批次报告](verification/p0-08-long-20261029-3cff94.md)。它已完成历史cohort；工具结构变化后不能以旧源码清单继续派发。
- 更早候选几何与当时状态：[历史场景目录](history/scenario-catalog-20261030.md)，不作为当前输入推荐。

S2/head_on未实现。几何评价选择指定Run的Concrete JSON Spec，[Catalog](../experiments/evaluations)仅作索引，不能直接传入--spec。新Run、新Spec、新分析目录必须独立，不自动选择“最新Run”。

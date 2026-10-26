# 单回合Runner

当前流程，适用tools布局v2。场景/任务与配对输入统一见[场景目录](../scenario_catalog.md)，不复用历史run_id。Runner是唯一生命周期管理者；不要另起Gazebo Server、手发第二Goal或删除锁绕过冲突。

## 静态预检

在Ubuntu Bash中执行。以下只读--check不创建Run或进程：

~~~bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
source tools/stages/p0_03_env.bash
CMR_SCENARIO="$PWD/experiments/scenarios/p0_long_route_static_v1.yaml"
CMR_PROFILE="$PWD/experiments/profiles/p0_long_route_static_v1.yaml"
CMR_RUN_ID="static-mppi-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B tools/run_trial.py \
  --scenario "$CMR_SCENARIO" --profile "$CMR_PROFILE" \
  --method mppi --run-id "$CMR_RUN_ID" --check
~~~

读取预检输出与原因。通过只说明启动前输入检查成功，不代表导航、录包或停车已通过。DWB使用同一静态配对输入，改method且使用新的run_id；动态版本的限制见场景目录。

## 执行与关闭

以下会启动仿真、录包并发送原有Goal；本次文档任务没有执行：

~~~bash
python3 -B tools/run_trial.py \
  --scenario "$CMR_SCENARIO" --profile "$CMR_PROFILE" \
  --method mppi --run-id "$CMR_RUN_ID"
~~~

退出0表示Runner各门通过；2为启动前拒绝；3为已创建但未通过；130为中断后的有限收尾。Ctrl+C交由所属Runner处理；不要给它套短于工作和清理预算的外部timeout。SIGKILL/断电不能保证finally。

每轮保存早期manifest、输入/源码包、events、Action UUID/终态、实际参数、观测、MCAP、检查结果与cleanup/result。行为、合同、录包和清理分别核对；VALID、Action成功或进程退出均不能单独替代完整通过。实际预算、IPC、退出及清理策略见[Runner合同](../contracts/runner-contract.md)，归档包结构见[tools说明](../../tools/README.md)。

## 关闭后的检查

~~~bash
CMR_RUN="/home/robot/commercial_mobile_robot/evidence/runs/$CMR_RUN_ID"
python3 -B tools/validate_run.py run "$CMR_RUN"
~~~

bag检查worker的新路径是tools/runner/inspect_bag.py；动态检查器是tools/dynamic/inspect_dynamic_recording.py。它们由Runner归档调用。需要独立复查时读取各自--help，选择全新--output，不回放bag或覆盖旧检查文件。

GUI操作见[观察指南](gazebo-observation.md)，批次见[批次指南](batch-experiments.md)，几何分析见[评价指南](geometry-evaluation.md)。旧短任务命令与原验收记录保存在[历史指南](../history/p0-05-runner-guide-20261030.md)和[P0-05报告](../verification/p0-05-bmzezluh.md)。

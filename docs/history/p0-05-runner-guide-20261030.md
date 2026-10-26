# 当前入口：长路线 p0_long_route_v1

map 起点 (-2.0,-0.5,0)，目标 (1.75,-0.5,0)，四元数 (0,0,0,1)。本次实施、实际 run_id、命令顺序、图表、失败保留与验收结论见 [长路线迁移交付](goal_migration_long_route_v1.md)。

静态日常入口：`/home/robot/commercial_mobile_robot/experiments/scenarios/p0_long_route_static_v1.yaml` 与 `/home/robot/commercial_mobile_robot/experiments/profiles/p0_long_route_static_v1.yaml`。动态入口 crossing/observed_occlusion 已切换新目标、新轨迹、新 world 和新 probe 证明；动态仍只使用 MPPI。07 使用各具体 run_id 的 JSON spec，p0_07_*.yaml 只是索引。08 仅同步并静态预检，本次没有重新执行故障矩阵。

下文为保留的旧版本说明/历史命令。旧 runner_profile.yaml 与 p0_05_official_static.yaml 继续作为旧任务兼容入口；旧目标结果不能算作新长路线验证。重新采集必须新 run_id，重新分析必须新 evaluation_id，禁止覆盖。

---

# 单回合 runner 使用说明

工作区固定 `/home/robot/commercial_mobile_robot`，仿真专用。先在 Ubuntu 终端加载：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/commercial_mobile_robot
```

只读预检（不创建子进程、节点或回合目录）：

```bash
python3 tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-05-check-mppi-20261025-review --check
```

启动一个正常 MPPI 回合，使用当次生成的真实新编号：

```bash
python3 tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id "p0-05-mppi-$(date -u +%Y%m%dT%H%M%SZ)-$RANDOM"
```

DWB 使用同样的场景和 profile，仅改方法及编号前缀：

```bash
python3 tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method dwb --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id "p0-05-dwb-$(date -u +%Y%m%dT%H%M%SZ)-$RANDOM"
```

受控失败只在明确理解测试意图时调用：

```bash
python3 tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id "p0-05-noinitial-$(date -u +%Y%m%dT%H%M%SZ)-$RANDOM" --fault withhold-initial-pose --ack-test-fault
```

正常通过退出0；输入拒绝2；已创建但未通过3；用户中断完成有限收尾130。受控缺初值应退出3，同时结果合同VALID/0。不要给整个runner套短于其清理预算的外部timeout。Ctrl+C请求有限收尾；SIGKILL/断电不能保证finally。

每轮自动保存早期manifest、输入/源码快照、events.jsonl、Action sidecar、observations、实际参数、MCAP、行为和bag核查、清理与result。文件已存在会拒绝覆盖；不手动删除锁文件绕过另一个运行中的runner。

源码/API责任与时间预算见 runner_contract.md。本机不显示RViz；Ogre1仍使用已登录图形会话的真实DISPLAY/XAUTHORITY。DDS使用现有loopback UDP profile，Gazebo子进程单独预加载已验证Boost线程库；不改系统shell或库。

`inspect_bag.py`只读SequentialReader遍历已关闭的MCAP，输出文件必须不存在；不会replay。`summarize_runs.py`枚举所有匹配目录，缺文件/坏合同/fixture/真实失败分别保留，不能仅凭VALID或Action成功判runner通过。

本次实际回合、验收命令及哪些步骤已运行，以 docs/verification/p0-05-bmzezluh.md 为准；本说明中的动态编号命令不复用既有编号。个人理解/RViz目视、独立碰撞评价、性能统计与实车安全均不由工具代签。


## 2026-10-25 实际验收入口

MPPI正常：/home/robot/commercial_mobile_robot/evidence/runs/p0-05-mppi-20261025-06；DWB正常：/home/robot/commercial_mobile_robot/evidence/runs/p0-05-dwb-20261025-01，均runner退出0。

缺初值：/home/robot/commercial_mobile_robot/evidence/runs/p0-05-noinitial-20261025-01，进入FAULT_ACTIVATED后约10秒退出3，未发布初值/目标，cleanup成功、合同VALID/0。它是预期失败案例，不是正常导航成功。

完整实际命令、200个测试、原始行号、所有失败和保护指纹见[工程报告](verification/p0-05-bmzezluh.md)。开发证据：/home/robot/commercial_mobile_robot/evidence/env/p0-05-dev-bmzezluh。

可再次执行的DWB只读预检：

~~~bash
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method dwb --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-05-check-dwb-20261025-final --check
~~~

已实际执行的只读MCAP检查及汇总（输出已存在，原样再次执行会拒绝覆盖）：

~~~bash
python3 -B tools/inspect_bag.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-05-mppi-20261025-06 --output /home/robot/commercial_mobile_robot/evidence/env/p0-05-dev-bmzezluh/p0-05-mppi-20261025-06-bag-independent.json
python3 -B tools/summarize_runs.py --root /home/robot/commercial_mobile_robot/evidence/runs --prefix p0-05- --output /home/robot/commercial_mobile_robot/evidence/env/p0-05-dev-bmzezluh/p0-05-index.json
~~~

原仓库没有CI workflow；分层测试实际在此Ubuntu运行，不能声称远程CI已执行。既有run_id拒绝重用，不删除历史目录绕过保护。

## P0-06 增量（第6步审查门）

原静态入口保持。新动态输入目前均draft，生产--check应拒绝。v2 probe/正式准备、代码差异、已执行测试和下一阶段停点见 [P0-06使用说明](p0_06_usage.md) 与 [第1～6步报告](verification/p0-06-a3179_6v-step1-6.md)。本次没有启动任何P0-06真实回合，未放行第7步。

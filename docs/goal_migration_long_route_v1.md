# 长路线 v1 实施与 06 → 07 实际联调交付

实施标识：20261029；现有 Ubuntu 工程 `/home/robot/commercial_mobile_robot`，起始 HEAD `7e38fb1`。Mac 仅执行 SSH、宿主采样与既有有界负载包装器。Ubuntu 与 Mac 墙钟日历存在偏差，未改系统时间；原始墙钟照实保留，轨迹评价使用仿真源纳秒、epoch，跨机负载使用独立单调时钟往返边界。

## 1. 先保护现场

祖先与仓内未发现额外 AGENTS.md；Mac 镜像 AGENTS.md 要求 sources 只读，已遵守。实际 Git 修改清单在 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/git-status-before.txt`。未发现运行中的 Nav2/Gazebo 回合，两个项目锁检查可独占获取。没有删除锁、全局杀进程、reset/clean、第二工程、提交、推送或实物操作。

可恢复源码/配置/文档备份：`/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/workspace-before.tgz`；已有未提交差异：`/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/dirty-before.patch`。任务前源码哈希 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/source-before.json`；历史证据哈希 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/historical-before.json`，最终保护结果 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/historical-protection-final.json`。原有 08 四种 profile 除目标及冻结任务引用外逐字段相等，故障策略完全相等，证明 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/p0-08-preservation.json`。

## 2. 冻结新任务，再切换入口

新任务 `p0_long_route_v1`：map 起点 `(-2.0,-0.5,0.0)`，目标 `(1.75,-0.5,0.0)`，四元数 `(0,0,0,1)`。`experiments/goals/p0_long_route_v1.json` 是原 NavigateToPose Action 请求格式，含 pose/header/pose 与 behavior_tree；不是 ScenarioSpec pose。

新增 `experiments/goals/p0_long_route_task_v1.json` 明确冻结版本、起终点及请求/初始请求/约束/机器人/地图输入哈希。`tools/task_contract.py` 只接受已知版本的确定起终点，核对实际请求、四元数、spawn、资源和哈希。原 P0-04 成功记录只证明控制器/资源出处，新任务首次运行不要求先有新任务成功记录。

旧 `experiments/runner_profile.yaml`、`experiments/scenarios/p0_05_official_static.yaml`、P0-04 输入和旧记录保留。静态长路线入口为 `experiments/profiles/p0_long_route_static_v1.yaml` 与 `experiments/scenarios/p0_long_route_static_v1.yaml`。动态入口仍为 crossing/observed_occlusion，但 revision 已变为 `p0-06-long-route-v1-r1-reviewed`，由 draft 经新 preparation、ready_for_probe 与真实 probe 证明后晋级 ready_for_recording。静态 v1 的 draft 字段不代替下列真实验收记录。

## 3. 地图、占用范围与静态单目标导航

原地图、静态柱、机器人及 MPPI/DWB 参数保留。实际地图起点/目标中心到非自由栅格最近距离约 0.539/0.626 m，SDF collision 采样包络在起终点自由；源审查 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/map-review-v2.json` 与 `map-review-v2.png`。归档 SDF 推导的保守 3D 球投影半径为 0.272594781861 m，07 从未改用 Nav2 0.22 m。

启动时保存实际 global/local costmap。全局地图严格检查远端目标 footprint；滚动局部地图检查当前机器人 footprint，避免要求远端目标位于当前局部窗口。阈值未放宽。运行时另存 `/plan` 真实发布路径到 runtime/plans.jsonl。

实际路径、停车空间复查：`/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/static-path-review-v2.json`；轨迹叠图 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/static-paths-v2.png`。各 run 的 runtime/global-costmap.yaml、local-costmap.yaml、nav-readiness.json、plans.jsonl、behavior.json、MCAP 和 goal-dispatch.json 可逐项复查。map/AMCL 栅格路径检查不等于独立物理碰撞检查。

|静态 run_id|实际终态|末端位置误差 m|yaw误差 rad|连续停止 s|回合结论|
|---|---|---:|---:|---:|---|
|p0-long-static-mppi-20261029-a1|SUCCEEDED|0.231826|0.044505|4.968|通过|
|p0-long-static-dwb-20261029-a1|SUCCEEDED|0.265829|0.177186|4.968|失败保留|
|p0-long-static-dwb-20261029-a2|SUCCEEDED|0.242219|0.139503|4.932|通过|

DWB a1 的最早失败为 OBSERVE_STOP 到达检查：终态附近误差约 0.237 m，随后定位漂移到 0.266 m，超过原 0.25 m。没有改采样策略、目标、控制器或阈值；一次独立冷启动 a2 复核通过。a1 的 result、bag、失败账本仍在，不能由 Action SUCCEEDED 改称成功回合。

## 4. 根据真实路径设计动态交互，再执行新 probe

S1：静态 MPPI 实测在接受后约 6.98 s 到 map x=-0.55（y≈-0.596）。圆柱改为 x=-0.55，沿 Y 从 -1.0 到 -0.05，8 s 移动、2 s 保持结点、2 s hold；速度 0.11875 m/s，接受后延迟 1 s。两条正式记录通过实际路径交点与障碍物先到的检查；不宣称同时接触。

S3：圆柱 x=-0.55，Y=-0.3 → 0.28 → 0.8，结点时间 0/2/8/10/16/18 s，hold 2 s；最大段速度约 0.0967 m/s。接受后延迟 20 s，利用长路线终点侧的新观察方向及静态柱形成遮挡，采集覆盖终态之后的完整运动。静止 probe 与正式导航各自检查其实际实体/位姿/扫描，不能用配置轨迹证明遮挡。

派生 SDF 只调整 scripted_obstacle 初态，原静态模型与依赖保留。命令位姿、服务成功与实际 Pose_V 分开。20 ms 调度、50 ms 原始间隔上限、0.02 m 轨迹误差、0.05 m 量程等原界限保留。scene window 45 s，原 runner 运行预算/到达/停止/新鲜度阈值均未放宽。

新 probe：`p0-long-crossing-probe-20261029-a1`、`p0-long-occlusion-probe-20261029-a1`。均为 Action NOT_SENT，startup READY / harness COMPLETED / cleanup SUCCEEDED。两位置接触正例/旧位置负例、两次暂停、实际快照、visual、原始 scan、时钟对齐、冷初态和真值隔离均通过；个人目视未签署。

准备输入 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/crossing-prep-r1/` 与 `observed_occlusion-prep-r1/`；正式晋级证明 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/crossing-ready-proof-r1.json`、`observed_occlusion-ready-proof-r1.json`。没有套用旧目标 ready 状态。

## 5. 四条正式采集与 07 完整评价

每条记录的范围先由唯一目标接受时间及冻结运动确定：开始=接受；结束=max(终态+5 s, release+最后结点+hold)。运行器在原停止观察后继续检查实际 Pose_V 源时间覆盖结束边界及额外 50 ms 支持，不通过缩短范围解决缺口。

|新 run_id|UUID|位置/yaw误差|停止 s|07 源范围 s|覆盖|线性模型最小间距 m|
|---|---|---|---:|---|---|---:|
|p0-long-crossing-low-20261029-a1|1abcf0728f274a8b84c107ebed66a8d5|0.231709/0.012401|4.968|9.603–36.404|full|0.141817179|
|p0-long-crossing-higher-20261029-a1|8f79b016ef444bb3ae72ddd36d3e5557|0.218274/0.033968|4.968|14.808–39.938|full|0.163081450|
|p0-long-occlusion-low-20261029-a1|d917151276d54ccbb608d46d2bdb9401|0.236827/0.032454|4.968|10.533–50.533|full|0.026608753|
|p0-long-occlusion-higher-20261029-a1|1a55fcd2df2a49c1bef60ce3cde4ff6e|0.199765/0.062678|4.968|15.381–55.381|full|0.026987589|

完整证据表 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/formal-evidence-table.json` 包含实际请求、UUID、epoch、实体 ID、源间隔、边界支持、原始路径与 spec 哈希；负载原始数据和进程归属在 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/host/`。
- crossing 宿主运动窗口 CPU：low 23.982%，higher 42.580%，增加 0.186 ≥ 0.10；higher 沿用两条 nice19、每条最多120秒的既有负载。
- occlusion 宿主运动窗口 CPU：low 25.425%，higher 44.327%，增加 0.189 ≥ 0.10；higher 沿用两条 nice19、每条最多120秒的既有负载。

每条新记录由 prepare_geometry_spec.py 从自身原始文件重新生成具体 JSON，未复制旧 spec 改 run_id。两个 p0_07_*.yaml 仅为索引，含可识别的新旧任务版本。07 算法和 evaluation_contract.md 无需修改，严格复用原始间隔、源时间、实体、几何包络、无外推规则。

以下是本次实际执行的命令链（输入/输出已存在，生成和绘图命令会拒绝覆盖；再次分析须选择新的 evaluation_id；重新采集须选择新 run_id）：

```bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
source /home/robot/commercial_mobile_robot/install/p0-06-a3179_6v/setup.bash
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_long_route_static_v1.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_long_route_static_v1.yaml --method mppi --run-id p0-long-static-mppi-20261029-a1
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_long_route_static_v1.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_long_route_static_v1.yaml --method dwb --run-id p0-long-static-dwb-20261029-a2
```

日常先 source 环境，按新 run_id 预检并运行上面的长路线静态 profile。动态正式 MPPI 入口如下（对应 profile 已携带长路线 base/version/hash）：

```bash
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/crossing.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_crossing.yaml --method mppi --run-id p0-long-crossing-low-20261029-a1
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/crossing.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_crossing.yaml --method mppi --run-id p0-long-crossing-higher-20261029-a1
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/observed_occlusion.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_observed_occlusion.yaml --method mppi --run-id p0-long-occlusion-low-20261029-a1
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/observed_occlusion.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_observed_occlusion.yaml --method mppi --run-id p0-long-occlusion-higher-20261029-a1
```

上述是 Ubuntu runner；本次实际两种负载经 Mac 的原工具 p0_06_host_trial.py 调用同一命令，精确 argv/runner.log/metadata 在 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/host/host-*-a1/`。higher 不能用无宿主包装的单条 Ubuntu 命令冒充。

```bash
python3 -B tools/validate_run.py run /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-low-20261029-a1
python3 -B tools/prepare_geometry_spec.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-low-20261029-a1 --output /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-crossing-low-20261029-a1.json
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-low-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-crossing-low-20261029-a1.json --check
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-low-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-crossing-low-20261029-a1.json --output-dir /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-crossing-low-20261029-a1/long-route-v1-20261029-e1
python3 -B tools/plot_geometry.py --report /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-crossing-low-20261029-a1/long-route-v1-20261029-e1/report.json --output /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-crossing-low-20261029-a1/long-route-v1-20261029-e1/geometry.png
python3 -B tools/validate_run.py run /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-higher-20261029-a1
python3 -B tools/prepare_geometry_spec.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-higher-20261029-a1 --output /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-crossing-higher-20261029-a1.json
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-higher-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-crossing-higher-20261029-a1.json --check
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-crossing-higher-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-crossing-higher-20261029-a1.json --output-dir /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-crossing-higher-20261029-a1/long-route-v1-20261029-e1
python3 -B tools/plot_geometry.py --report /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-crossing-higher-20261029-a1/long-route-v1-20261029-e1/report.json --output /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-crossing-higher-20261029-a1/long-route-v1-20261029-e1/geometry.png
python3 -B tools/validate_run.py run /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-low-20261029-a1
python3 -B tools/prepare_geometry_spec.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-low-20261029-a1 --output /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-occlusion-low-20261029-a1.json
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-low-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-occlusion-low-20261029-a1.json --check
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-low-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-occlusion-low-20261029-a1.json --output-dir /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-occlusion-low-20261029-a1/long-route-v1-20261029-e1
python3 -B tools/plot_geometry.py --report /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-occlusion-low-20261029-a1/long-route-v1-20261029-e1/report.json --output /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-occlusion-low-20261029-a1/long-route-v1-20261029-e1/geometry.png
python3 -B tools/validate_run.py run /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-higher-20261029-a1
python3 -B tools/prepare_geometry_spec.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-higher-20261029-a1 --output /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-occlusion-higher-20261029-a1.json
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-higher-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-occlusion-higher-20261029-a1.json --check
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-long-occlusion-higher-20261029-a1 --spec /home/robot/commercial_mobile_robot/experiments/evaluations/p0-long-occlusion-higher-20261029-a1.json --output-dir /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-occlusion-higher-20261029-a1/long-route-v1-20261029-e1
python3 -B tools/plot_geometry.py --report /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-occlusion-higher-20261029-a1/long-route-v1-20261029-e1/report.json --output /home/robot/commercial_mobile_robot/evidence/metrics/p0-long-occlusion-higher-20261029-a1/long-route-v1-20261029-e1/geometry.png
```

以上四条链全部 strict/dynamic/prepare/check/evaluate/plot 退出 0；还逐项检查 full、行为、边界、实际请求、原始数据和清理，未凭单个退出码验收。每个输出包含 spec.json/input.json/sources.json/intervals.json/report.json/geometry.png，原 run 未回写。

S3 图与原始光束支持：
- `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/p0-long-occlusion-low-20261029-a1-visibility.json` 与同名 PNG；原 run 的 runtime/gazebo-raw.jsonl 和 bag/bag_0.mcap。JSON 含同一 entity ID、源纳秒、bag message index、实测光束量程及静态前景交点，明确 visible → occluded → visible；partial/unknown 未当作遮挡。
- `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/p0-long-occlusion-higher-20261029-a1-visibility.json` 与同名 PNG；原 run 的 runtime/gazebo-raw.jsonl 和 bag/bag_0.mcap。JSON 含同一 entity ID、源纳秒、bag message index、实测光束量程及静态前景交点，明确 visible → occluded → visible；partial/unknown 未当作遮挡。

## 6. 旧四条记录兼容回归

|旧 run_id|新分析目录|覆盖|线性模型最小间距 m|
|---|---|---|---:|
|p0-06-crossing-kwdqt70u-c11|/home/robot/commercial_mobile_robot/evidence/metrics/p0-06-crossing-kwdqt70u-c11/long-route-v1-compat-20261029-e1|full|-0.144711322|
|p0-06-crossing-kwdqt70u-c12|/home/robot/commercial_mobile_robot/evidence/metrics/p0-06-crossing-kwdqt70u-c12/long-route-v1-compat-20261029-e1|full|-0.133461043|
|p0-06-occlusion-kwdqt70u-o5|/home/robot/commercial_mobile_robot/evidence/metrics/p0-06-occlusion-kwdqt70u-o5/long-route-v1-compat-20261029-e1|full|0.924402769|
|p0-06-occlusion-kwdqt70u-o6|/home/robot/commercial_mobile_robot/evidence/metrics/p0-06-occlusion-kwdqt70u-o6/long-route-v1-compat-20261029-e1|full|0.926518800|

旧 S1 的负间距仍表示声明模型的包络相交，没有将它改为物理碰撞或修改历史结论。旧原始文件、既有分析及原 spec 保留；本次旧 spec 从原记录另行生成到环境审计目录。

## 7. 08 后续入口与测试

08 static/normal/controller-deactivate/pause-gazebo/short-deadline 仅同步任务与相关哈希；保留现有故障预算、策略、arrival_policy、raw_gazebo 和未提交功能。两矩阵所有 slot 指向新目标；本次只执行：

```bash
python3 -B tools/run_smoke_matrix.py --matrix /home/robot/commercial_mobile_robot/experiments/p0_smoke_matrix.yaml --batch-dir /home/robot/commercial_mobile_robot/evidence/batches/long-route-smoke-precheck --limit 1 --check
python3 -B tools/run_smoke_matrix.py --matrix /home/robot/commercial_mobile_robot/experiments/p0_fault_matrix.yaml --batch-dir /home/robot/commercial_mobile_robot/evidence/batches/long-route-fault-precheck --limit 1 --check
```

相关单测首次 159 项通过；最终正确 source ROS 并隔离 domain72 后，完整 tests 为 **413 passed**，日志 `/home/robot/commercial_mobile_robot/evidence/env/goal-long-route-v1-20261029/tests-final-ros.log`。覆盖 geometry_metrics/alignment/inputs/cli、dynamic_contract/runner/recording、任务加载和既有 08 测试。新增 tests/test_long_route_task.py 验证请求/起点/版本/哈希不一致、旧准备误用及长路线结束边界。没有改旧测试断言。

首轮回归暴露旧 yaw 浮点尾差，改为 1e-9 有界比较；资源 schema 补齐 package 字段。扩大录包测试时首次没有 source ROS，六例因 rosbag2_py 缺失失败；保留 tests-final.log，在正确环境全套通过，未通过改测试绕过。

## 8. 验收结论与边界

**已实现**：新版本文件和入口、任务冻结与新旧兼容、实际路径证据、完整范围边界。**已静态验证**：目标/请求/资源哈希、负例测试、08 矩阵预检。**已集成验证**：MPPI/DWB 静态成功回合、两个新 probe、四条新动态合格记录、四条 07 full 评价、旧四条兼容回归。

**待验证/未签署**：08 故障矩阵的长路线实际重新验收、个人理解与目视确认、实物/全场景安全。新 S3 模型余量只有约 2.7 cm，误差界未建立，不能解释为物理安全余量。所有 collision 继续 null。

**回答：新目标下，06 已产生四条合格正式记录；07 已用这四条记录按原合同完整运行成功。** 此结论限定于本文声明的仿真、对象对、固定范围和模型。

最终保护核对：24,177 个任务前历史证据文件哈希全部相同、无缺失；源码清单相对于开始时实际工作区：新增17、修改23、192个既有文件保持不变，无删除。逐文件理由见 [文件清单](goal_migration_long_route_v1_files.md)，机器可读清单与精确工作区差异在环境审计目录。

图表QA：初版静态地图图使用固定20m extent，目检发现错误。v2按实际图片尺寸、map origin与resolution重画，unknown按非自由栅格计入复查。初版图/JSON保留为诊断，不作为最终地图证据；实际Nav2地图和07世界位姿评价未修改。

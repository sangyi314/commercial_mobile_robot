# P0-06 使用说明（第7～11步工程验证后）

真实工程记录与边界见 docs/verification/p0-06-kwdqt70u-live.md。当前 crossing 和 observed_occlusion 已冻结为 ready_for_recording；旧 probe/draft 文件保留，不能当当前正式输入。P0-05静态入口、v1合同保留。个人目视/理解仍待确认，collision=null。

## 环境与唯一生命周期

在 Ubuntu 执行，不在Mac镜像执行ROS。原runner负责项目锁、至多一个目标、期限、归属进程、录包与清理。先完成前轮关闭检查；禁止手发第二目标和全局pkill。

```bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
source /home/robot/commercial_mobile_robot/install/p0-06-a3179_6v/setup.bash
source /home/robot/commercial_mobile_robot/tools/p0_03_env.bash
```

原生构建实际完成，当前build-04日志和两项CTest在 evidence/env/p0-06-live-kwdqt70u。修改源码或任一受审依赖后，既有probe准备指纹失效，必须新revision重新探测；不能直接套旧ready标签。

## 已执行的只读预检

以下命令实际退出0，不生成回合目录。再次执行将核对当前输入：

```bash
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/crossing.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_crossing.yaml --run-id p0-06-crossing-review-kwdqt70u-c9 --check
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/observed_occlusion.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_observed_occlusion.yaml --run-id p0-06-occlusion-review-kwdqt70u-o5 --check
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-06-static-mppi-review-kwdqt70u --check
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method dwb --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-06-static-dwb-review-kwdqt70u --check
```

## 实际执行的单回合命令

以下为历史执行命令，不是覆盖重跑许可。同名输出/回合已存在会拒绝。宿主包装器仅加入采样和本任务两条 nice19、最多120秒负载，调用原Ubuntu runner。low表示不另加负载；higher必须经实际CPU窗口复算达到预设差值才认可。

在本次Mac实际执行：

```bash
python3 -B /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host_trial_kwdqt70u.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/crossing.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_crossing.yaml --run-id p0-06-crossing-kwdqt70u-c11 --load low --output /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host-kwdqt70u/crossing-c11
```

```bash
python3 -B /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host_trial_kwdqt70u.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/crossing.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_crossing.yaml --run-id p0-06-crossing-kwdqt70u-c12 --load higher --output /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host-kwdqt70u/crossing-c12
```

```bash
python3 -B /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host_trial_kwdqt70u.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/observed_occlusion.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_observed_occlusion.yaml --run-id p0-06-occlusion-kwdqt70u-o5 --load low --output /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host-kwdqt70u/occlusion-o5
```

```bash
python3 -B /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host_trial_kwdqt70u.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/observed_occlusion.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_observed_occlusion.yaml --run-id p0-06-occlusion-kwdqt70u-o6 --load higher --output /Users/manman/.codex/.chatgpt-projects/g-p-6a9f976d7b648191ac95a3c6d217d849/deliverables/P0-06_host-kwdqt70u/occlusion-o6
```

无宿主负载包装时，Ubuntu正常入口仍是同一 run_trial.py；上述四轮包装器的 runner.log 保存了真实启动结果。每轮完整收尾，再校验bag与sidecar。新实验需使用唯一新ID及新输出目录，不能复用本页历史ID。

## 已执行的关闭后检查

```bash
python3 -B tools/validate_run.py run /home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11
python3 -B tools/inspect_bag.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11 --output /home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u/recomputed-bag-c11.json
python3 -B tools/inspect_dynamic_recording.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11 --output /home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u/recomputed-dynamic-c11.json
python3 -B tools/summarize_runs.py --root /home/robot/commercial_mobile_robot/evidence/runs --prefix p0-06- --output /home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u/index-final.json
```

带output的历史命令拒绝覆盖，应为新的复查选择新的输出文件。VALID只说明记录合同；Action成功、normal_pass、动态有效、bag内容和cleanup分别判断。纯逻辑/隔离ROS合成测试不是导航基线。

## 探测与停止条件

当前各自准备目录为 evidence/env/p0-06-live-kwdqt70u/crossing-r10-preparation 和 observed_occlusion-final08-preparation。probe模式固定不发导航目标，使用PROBE_ARMED，包含冷启动初态、两个位置局部接触正负例、运动、两次不同相位暂停/恢复。历史final08四probe详见报告；本页不把准备目录当正式记录。

资源/实体身份变化、时钟回跳、数据缺失/乱序、真值隔离失败、轨迹/间隔超限、缺扫描支持、Action未知、bag边界缺失或清理失败即保留失败并停止。不得用未知补零，不得放宽阈值重试。原官方symbol重名SDF诊断仍255，实际加载另行验证；不得改成静态检查通过。

物理外参[-0.064,0,0.15]来自实际实体树及扫描，保留真实3D姿态。源时间对齐不平移轨迹。S1当前是有实际路线与先后时刻的交汇，圆柱明显先经过，不代表同时近距对抗；S3必须同一实体先见/静态遮挡/恢复，partial与unknown不计遮挡。完整碰撞评价、实车安全和个人理解不由本工具签署。

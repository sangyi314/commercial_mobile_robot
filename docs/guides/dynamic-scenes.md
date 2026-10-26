# 动态场景准备与检查

当前输入/版本状态只在[场景目录](../scenario_catalog.md)维护。旧ready声明绑定旧源码；新布局模板仍draft、review/probe=null，不应直接执行正式导航。动态模块不创建第二套Runner生命周期。

## 准备次序

1. 阅读[P0-06设计](../design/p0-06-dynamic-scenes.md)、[新布局输入说明](../../experiments/tool_layout_v2/README.md)，核对实际场景、模型、轨迹、任务、二进制和完整依赖锁。原候选坐标不等于实测证据。
2. 显式工程审查后，用阶段准备入口生成新的独立probe输入。下面只是操作模板，会写准备目录；本次没有执行：
   
~~~bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
CMR_PREP="$PWD/experiments/tool-layout-probe-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B tools/stages/prepare_dynamic_probe.py \
  --scenario experiments/tool_layout_v2/scenarios/crossing.yaml \
  --profile experiments/tool_layout_v2/profiles/p0_06_crossing.yaml \
  --output-dir "$CMR_PREP" --as-probe --ack-review
~~~

3. 准备结果仅许可probe，不证明真实实体、源时间对齐、扫描、暂停或真值隔离。先以原Runner对生成的probe.yaml/profile.yaml做--check，再按已有合同开展单独授权的probe、记录与审查；通过证据门后才能形成新的正式输入。不得手改status或沿用旧probe指纹。
4. 新输入必须独立版本、run_id和输出目录；源码/模型/依赖变化使旧准备指纹失效。按[Runner指南](runner.md)统一归属、录包和清理。普通Ubuntu执行不构成higher宿主负载实验。

## 宿主入口与记录

Mac宿主程序的新路径为tools/stages/p0_06_host_trial.py，公共实现为tools.dynamic.host_trial。它按原SSH关系调用Ubuntu tools/run_trial.py。使用包含完整tools包的实际Mac检出路径，不能把Ubuntu的/home路径当作Mac本地路径；参数以该入口--help为准。本次未运行宿主负载，也未改变原采样/负载机制。

实际Pose_V、命令位姿和服务应答分别记录。scan在MCAP中；真值sidecar不接入Nav2。动态检查器位于tools/dynamic/inspect_dynamic_recording.py，由Runner归档调用，独立输出不得覆盖原报告。动态记录有效不等于几何无相交，物理collision仍未知。

原短任务四轮、低/较高负载与V/O/V证据见[P0-06报告](../verification/p0-06-kwdqt70u-live.md)；新任务既有实测见[长路线报告](../verification/long-route-v1-20261029.md)。详细已执行命令另存[历史指南](../history/p0-06-guide-20261030.md)，不能原样覆盖重跑。

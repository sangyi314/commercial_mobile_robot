# Gazebo 观察入口与空白场景修复

> 工具布局v2（当前入口）：根目录8个常用CLI保持路径；阶段入口改为tools/stages/，自动worker和实现移入功能子包。完整迁移表、归档布局和测试结果见[tools/README.md](../../tools/README.md)。下文旧日期、冻结输入与验收命令保留历史原文；旧动态源码锁不能直接用于新布局，新模板在experiments/tool_layout_v2且仍为draft，真实集成未验证。

适用仓库：/home/robot/commercial_mobile_robot。保留 Ogre1、LIBGL_ALWAYS_SOFTWARE=1 和原有 preload。观察入口复用原 run_trial.main；不创建第二套 Server、Nav2、录包或清理生命周期。

## 修复范围

原冻结静态 world 显式配置 Physics/UserCommands/Sensors/Imu，却未配置 SceneBroadcaster。Server 能运行导航不代表 GUI 能获得场景。旧 readiness 只证明 GUI 初始化请求和所属 X11 窗口存在，不能证明 Entity Tree 或像素已显示。

新增 src/cmr_sim/worlds/p0_long_route_observe_v1.sdf：从原冻结 world 派生，仅追加一个 gz::sim::systems::SceneBroadcaster。新增 experiments/{profiles,scenarios}/p0_long_route_observe_v1.yaml 指向派生 world 及其新 SHA256。原静态和动态输入、资源哈希、目标、初始位姿、Nav2/MPPI 参数和原 Runner 保持不变。已有动态 world 含 SceneBroadcaster，没有重复添加。

## 在 Ubuntu 桌面终端启动

DISPLAY 和 XAUTHORITY 使用当前桌面会话实际值。以下命令不暂停 Gazebo；等待时间使用墙钟，最大120秒，且不能延长原 work 总预算。

```bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
SCENARIO=/home/robot/commercial_mobile_robot/experiments/scenarios/p0_long_route_observe_v1.yaml
PROFILE=/home/robot/commercial_mobile_robot/experiments/profiles/p0_long_route_observe_v1.yaml
python3 -B tools/run_trial_observe.py \
  --scenario "$SCENARIO" --profile "$PROFILE" --method mppi \
  --run-id gui-check-only --gazebo-gui --wait-for-operator --check
```

预检成功后：

```bash
RUN_ID="gui-observe-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B tools/run_trial_observe.py \
  --scenario "$SCENARIO" --profile "$PROFILE" --method mppi \
  --run-id "$RUN_ID" --gazebo-gui --wait-for-operator --operator-timeout 120
```

终端出现 “Server scene verified; GUI window open; navigation ready” 后，目视确认机器人、地面、障碍物，在空行按 Enter。默认相机可能被围墙遮挡，可在 GUI 中聚焦 turtlebot3_waffle。Enter 前不写 dispatch-intent、不发送 Goal；输入后沿用原目标和期限。测试中的自动 PTY Enter 不代表用户目视或安全签署。

不需要观察时，保持原静态输入：

```bash
RUN_ID="gui-off-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B tools/run_trial_observe.py \
  --scenario experiments/scenarios/p0_long_route_static_v1.yaml \
  --profile experiments/profiles/p0_long_route_static_v1.yaml \
  --method mppi --run-id "$RUN_ID" --no-gazebo-gui
```

仅传 --gazebo-gui、不传 --wait-for-operator 时，原 Runner 在场景检查后自动发 Goal。--no-gazebo-gui 直接委托原 main，不安装 GUI 适配。不能把 --wait-for-operator 与关闭 GUI 同时使用。

## readiness 的证据边界

1. GUI 存活：进程启动和持续退出监测。
2. window_ready：初始化日志、有效窗口尺寸、窗口 PID 归属、IsViewable；不是场景成功。
3. server_scene_verified：同回合 GZ_PARTITION 下实际请求 /world/default/scene/info 成功；包含 ground_plane、turtlebot3_world、turtlebot3_waffle 及可视记录（递归检查嵌套障碍物）；clock/stats 连续推进，Pose_V 包含机器人 ID。
4. gui_received_entities / visual_confirmed：自动报告保持 null。独立探针收到场景不等于 GUI 收到场景，也不等于像素正确；需对应回合的截图或人工观察。

ready 表示“允许开始观察”，不表示自动证明 GUI 已渲染成功。runtime/gui-scene-proof.json 保存原始场景和更新样本；runtime/gui-status.json 保存分层状态。提示明确要求目视检查。

探针在原 ProcessSupervisor 下启动，8秒总期限，服务响应2秒、更新样本最多4秒；所有进程仍由原生命周期归属和清理。Transport 13 同步请求在订阅 Python 回调之前完成，finally 取消订阅。GUI_READY_TIMEOUT、GUI_SCENE_TIMEOUT、GUI_SCENE_UNAVAILABLE、GUI_EARLY_EXIT、OPERATOR_WAIT_TIMEOUT 均拒绝继续派发。错误退出会保留原失败状态，不改写为成功。

## 验证与历史记录

本次证据根目录：/home/robot/commercial_mobile_robot/evidence/env/gui-scene-fix-20261029。
详细回合表、最小差异、保护基线检查与日志索引见该目录的 report.md、repair.patch、protection-audit.json、integration-summary.json。

针对 GUI/探针/长路线合同的测试：tests-04.log，75 passed。包括错误分区、缺少模型/可视记录、服务超时、时钟不推进、机器人位姿缺失、订阅失败，以及原入口委托、输入等待、窗口归属和期限。

旧 evidence/env/runner-gui-20261029 与旧回合全部保留；其中 ready=True 只按旧门槛成立，不能作为场景已显示的证据。原先完整460项测试属于旧入口验证，本次没有重跑或宣称新增全量460项验证。

新派生 world 具有新资源哈希，属于独立观察输入，不能与原冻结 P0-05/P0-06 基准混记。GUI 增加渲染负载；观察回合不替代无 GUI 基准。collision=null 仍是未知，本修复不构成碰撞或 P0-07 阶段验收。没有 commit 或 push。

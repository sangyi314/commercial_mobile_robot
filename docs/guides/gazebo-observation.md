# Gazebo观察与人工确认

适用tools布局v2。观察入口复用tools.runner.trial的同一Runner实现；关闭GUI直接委托原生命周期。输入配对和历史/当前状态统一见[场景目录](../scenario_catalog.md)。

## 预检与观察

在Ubuntu桌面Bash中使用真实DISPLAY/XAUTHORITY，保留既有Ogre1、LIBGL_ALWAYS_SOFTWARE和Gazebo专用preload。以下预检不启动GUI：

~~~bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
CMR_SCENARIO="$PWD/experiments/scenarios/p0_long_route_observe_v1.yaml"
CMR_PROFILE="$PWD/experiments/profiles/p0_long_route_observe_v1.yaml"
CMR_RUN_ID="gui-observe-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B tools/run_trial_observe.py \
  --scenario "$CMR_SCENARIO" --profile "$CMR_PROFILE" \
  --method mppi --run-id "$CMR_RUN_ID" \
  --gazebo-gui --wait-for-operator --check
~~~

通过预检后，以下会启动真实仿真；本次未执行：

~~~bash
python3 -B tools/run_trial_observe.py \
  --scenario "$CMR_SCENARIO" --profile "$CMR_PROFILE" \
  --method mppi --run-id "$CMR_RUN_ID" \
  --gazebo-gui --wait-for-operator --operator-timeout 120
~~~

等待提示后，目视确认机器人、地面和场景实体，在空行按Enter。Enter前不写dispatch-intent或发Goal。默认相机可能被墙遮挡，可聚焦机器人。静态观察场景没有自动生成的横穿道具。仅--gazebo-gui而不等待人工时，Runner在场景检查后会自动发Goal。

关闭GUI可用--no-gazebo-gui并选择场景目录中的静态配对输入；不能同时--wait-for-operator。人工等待默认60秒、最大120秒，受原work预算限制；SSH人工操作须有TTY。观察负载不与无GUI基准混记。

## readiness与日志

进程存活、所属可见窗口、同partition Server的真实scene/pose/clock证据分别检查。独立探针成功不证明GUI收到实体或像素正确，gui_received_entities/visual_confirmed仍需人工/对应记录核实。

runtime/gui-status.json和runtime/gui-scene-proof.json保存分层状态。GUI_READY_TIMEOUT、GUI_SCENE_TIMEOUT、GUI_SCENE_UNAVAILABLE、GUI_EARLY_EXIT、OPERATOR_WAIT_TIMEOUT均保留失败原因，不能改成“已显示”。

## 空白场景排查

先查原Runner管理的唯一Server与GUI是否处于同一GZ_PARTITION，再检查scene/info实际响应、机器人/障碍物实体与后续更新，不能只凭topic存在。确认SceneBroadcaster现状后再诊断3D Scene插件、Ogre1/Mesa/UTM渲染；不要先切Ogre2或重复添加插件。

历史空白场景原因、派生world、75项测试与真实修复记录见[原观察指南快照](../history/gazebo-observation-guide-20261030.md)。该记录有自己的源码版本；新工具布局仅完成离线兼容验证，真实GUI/Runner集成待验证。

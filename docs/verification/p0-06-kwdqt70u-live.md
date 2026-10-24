# P0-06 第7～11步工程验收

结论：本次限定工程范围通过。当前冻结版本完成 S1 横穿、S3 已观测遮挡各低/较高负载一次，共四个独立冷启动正式回合；原始录包、独立实际位姿、导航行为及清理分别验证。具备 P0-07 真实输入接入条件，但不等于完整碰撞评价、安全证明或个人签署。

用户于本对话授权继续第7～11步及必要最小修复，替代原第6步停止要求。工程审查由 Codex 完成；个人理解与 RViz 目视观察保持待本人确认。报告日期采用客户端 2026-10-27；宿主/虚拟机墙钟差异未被人为校正。

## 起点与保护

实际执行主机为 Ubuntu 24.04 ARM64 cmr-ubuntu，工作区 `/home/robot/commercial_mobile_robot`，续作起点 Git HEAD `0062f76245ca71c1806548457b50448a3db8a8c6`。Mac 仅运行宿主负载采样/有界负载和 SSH 包装器，未用镜像替代机器人仓库。Ubuntu 适用祖先/仓内 AGENTS 检查见 before.json；Mac 镜像 sources 保持只读。
开发验收目录：`/home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u`。修改前 `before.json`、可恢复 `source-before.tgz`、逐次修复前 tgz 和所有差异均保留。原 P0-04/P0-05 五个真实记录再次通过严格 validator；详见 legacy-validation.json。保护核对覆盖核心、三份基线、v1 合同、validator、旧主 runner 与 DDS 环境；必要的 nav_goal.py 录包边界确认扩展单独列明。首次审计 final-audit.json 因 HEAD 改变退出1并保留；补充审查见 git-drift-impact-review.json，最终内容与清理核对见 final-audit-02.json；不能据此证明初始快照之前从未删除历史。

## 当前四轮正式记录

检查项|S1 低 c11|S1 较高 c12|S3 低 o5|S3 较高 o6
---|---|---|---|---
真实回合|`/home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11`|`/home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c12`|`/home/robot/commercial_mobile_robot/evidence/runs/p0-06-occlusion-kwdqt70u-o5`|`/home/robot/commercial_mobile_robot/evidence/runs/p0-06-occlusion-kwdqt70u-o6`
实际控制器|nav2_mppi_controller::MPPIController|nav2_mppi_controller::MPPIController|nav2_mppi_controller::MPPIController|nav2_mppi_controller::MPPIController
Action/runner/合同/bag/动态/清理|SUCCEEDED / PASS / VALID / 有效 / 有效 / SUCCEEDED|SUCCEEDED / PASS / VALID / 有效 / 有效 / SUCCEEDED|SUCCEEDED / PASS / VALID / 有效 / 有效 / SUCCEEDED|SUCCEEDED / PASS / VALID / 有效 / 有效 / SUCCEEDED
唯一 UUID|0b6fc939ea5b4a1885b6022b4952736b|a57fd2ae1e8648df886e97d2b4b31023|a4ecd30a413a4189b7cf7b6bd4e56c1c|534ff1ed2b1441708c0ecc52eb0f142e
odom 最大位移 m|0.837819|0.806371|0.762212|0.740494
独立 Gazebo 实际位移 m|0.815834|0.792414|0.743003|0.719860
定位估计到达误差 m / rad|0.241702 / 0.039570|0.240913 / 0.046375|0.235477 / 0.171265|0.248339 / 0.165436
停止窗口 s / 样本数|4.968 / 139|4.968 / 139|4.968 / 139|4.968 / 139
停止最大间隔 s|0.036000|0.036000|0.036000|0.036000
停止最大线速 / 角速 / 漂移|0.000000 / 0.000000 / 0.000000|0.000000 / 0.000000 / 0.000000|0.000000 / 0.000000 / 0.000000|0.000000 / 0.000000 / 0.000000
轨迹最大误差 mm|3.984|4.426|2.900|2.900
实际宿主 CPU 忙碌均值|20.6%|41.0%|22.8%|43.3%
未做全程碰撞评价|collision=null|collision=null|collision=null|collision=null

实际 goal checker 的位置/归一化角度限均为 0.25 m / 0.25 rad，保留原 stateful 设置。到达值来自成功后的新鲜 map→base 定位估计，不是物理真值。停止窗口要求连续推进≥3仿真秒、≥5样本、间隔≤0.5秒、线速≤0.01m/s、角速≤0.03rad/s、漂移≤0.02m；最终命令 abs(vx)、abs(wz)≤0.001，窗口开始后保存的三段速度链无新非零命令。缺消息未按零处理。

### 原始证据入口（每轮均可复查）

**p0-06-crossing-kwdqt70u-c11**
- `/home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11/logs/action-client.jsonl` 第 2 / 1899 行：同一 UUID 接受与 SUCCEEDED，源时刻 11.295000→30.171000 s。
- `runtime/observations.jsonl`：运动行 [488, 2526]；到达第 3067 行、stamp=41.076000s；停止行 [2613, 2849]、源窗口 31.680000→36.648000s。
- `runtime/gazebo-raw.jsonl`：实际机器人运动行 [4510, 12037]；圆柱轨迹行 [4912, 10530]，源窗口 ns=[12295000000, 26395000000]；`runtime/motion-commands.jsonl` 第 1 行为绑定事件。
- 完整复算与原始 SHA256：`/home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u/independent-reviews/p0-06-crossing-kwdqt70u-c11.json`；bag 原始 `/scan` 在该轮 `bag/*.mcap`，逐帧消息索引、时间、实际 Pose_V 插值括号及波束几何见复算文件 example_frames 和 dynamic-report.json。
- 就绪/订阅隔离：`runtime/dynamic-readiness.json`、`runtime/all-parameters.json`、`logs/events.jsonl`；清理：`runtime/cleanup.json` 的 PID/start_ticks/归属 token、信号、退出码与 remaining=[]，不是只看结果字段。

实际横穿复算：`{"xy": [-2.070981289104253, -0.14], "robot_source_ns": [26949000000, 26967000000], "actor_source_ns": [13863000000, 13881000000], "robot_intersection_estimated_ns": 26949801087, "actor_intersection_estimated_ns": 13864342758, "ahead_of_robot": true, "simultaneous_contact_claim": false}`。

**p0-06-crossing-kwdqt70u-c12**
- `/home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c12/logs/action-client.jsonl` 第 2 / 1890 行：同一 UUID 接受与 SUCCEEDED，源时刻 9.081000→27.858000 s。
- `runtime/observations.jsonl`：运动行 [378, 2407]；到达第 2978 行、stamp=39.420000s；停止行 [2491, 2727]、源窗口 29.340000→34.308000s。
- `runtime/gazebo-raw.jsonl`：实际机器人运动行 [3629, 11114]；圆柱轨迹行 [4030, 9642]，源窗口 ns=[10081000000, 24181000000]；`runtime/motion-commands.jsonl` 第 1 行为绑定事件。
- 完整复算与原始 SHA256：`/home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u/independent-reviews/p0-06-crossing-kwdqt70u-c12.json`；bag 原始 `/scan` 在该轮 `bag/*.mcap`，逐帧消息索引、时间、实际 Pose_V 插值括号及波束几何见复算文件 example_frames 和 dynamic-report.json。
- 就绪/订阅隔离：`runtime/dynamic-readiness.json`、`runtime/all-parameters.json`、`logs/events.jsonl`；清理：`runtime/cleanup.json` 的 PID/start_ticks/归属 token、信号、退出码与 remaining=[]，不是只看结果字段。

实际横穿复算：`{"xy": [-2.1016850052982496, -0.14], "robot_source_ns": [23229000000, 23247000000], "actor_source_ns": [11427000000, 11445000000], "robot_intersection_estimated_ns": 23229618483, "actor_intersection_estimated_ns": 11443401398, "ahead_of_robot": true, "simultaneous_contact_claim": false}`。

**p0-06-occlusion-kwdqt70u-o5**
- `/home/robot/commercial_mobile_robot/evidence/runs/p0-06-occlusion-kwdqt70u-o5/logs/action-client.jsonl` 第 2 / 828 行：同一 UUID 接受与 SUCCEEDED，源时刻 12.435000→20.595000 s。
- `runtime/observations.jsonl`：运动行 [515, 1425]；到达第 2636 行、stamp=45.648000s；停止行 [1509, 1745]、源窗口 22.068000→27.036000s。
- `runtime/gazebo-raw.jsonl`：实际机器人运动行 [4967, 8216]；圆柱轨迹行 [5764, 13739]，源窗口 ns=[14435000000, 34435000000]；`runtime/motion-commands.jsonl` 第 1 行为绑定事件。
- 完整复算与原始 SHA256：`/home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u/independent-reviews/p0-06-occlusion-kwdqt70u-o5.json`；bag 原始 `/scan` 在该轮 `bag/*.mcap`，逐帧消息索引、时间、实际 Pose_V 插值括号及波束几何见复算文件 example_frames 和 dynamic-report.json。
- 就绪/订阅隔离：`runtime/dynamic-readiness.json`、`runtime/all-parameters.json`、`logs/events.jsonl`；清理：`runtime/cleanup.json` 的 PID/start_ticks/归属 token、信号、退出码与 remaining=[]，不是只看结果字段。

合格连续观测段：visible 15.000→17.601s / 14帧; occluded 18.600→20.202s / 9帧; visible 21.801→26.601s / 25帧; visible 27.000→34.401s / 38帧。partial=11，unknown=2，未归入遮挡。

**p0-06-occlusion-kwdqt70u-o6**
- `/home/robot/commercial_mobile_robot/evidence/runs/p0-06-occlusion-kwdqt70u-o6/logs/action-client.jsonl` 第 2 / 792 行：同一 UUID 接受与 SUCCEEDED，源时刻 9.702000→17.496000 s。
- `runtime/observations.jsonl`：运动行 [410, 1282]；到达第 2523 行、stamp=43.200000s；停止行 [1364, 1599]、源窗口 18.936000→23.904000s。
- `runtime/gazebo-raw.jsonl`：实际机器人运动行 [3873, 6979]；圆柱轨迹行 [4674, 12640]，源窗口 ns=[11702000000, 31702000000]；`runtime/motion-commands.jsonl` 第 1 行为绑定事件。
- 完整复算与原始 SHA256：`/home/robot/commercial_mobile_robot/evidence/env/p0-06-live-kwdqt70u/independent-reviews/p0-06-occlusion-kwdqt70u-o6.json`；bag 原始 `/scan` 在该轮 `bag/*.mcap`，逐帧消息索引、时间、实际 Pose_V 插值括号及波束几何见复算文件 example_frames 和 dynamic-report.json。
- 就绪/订阅隔离：`runtime/dynamic-readiness.json`、`runtime/all-parameters.json`、`logs/events.jsonl`；清理：`runtime/cleanup.json` 的 PID/start_ticks/归属 token、信号、退出码与 remaining=[]，不是只看结果字段。

合格连续观测段：visible 11.802→14.601s / 15帧; occluded 16.002→17.400s / 8帧; visible 19.401→22.200s / 15帧; visible 22.602→31.602s / 46帧。partial=15，unknown=1，未归入遮挡。

## 场景晋级、数据流与边界

当前冻结入口为 formal-input-freeze-release.json、ready-proof-crossing-r10.json、ready-proof-observed_occlusion-final08.json。两场景分别使用自己的 S1 r10 / S3 final08 low/higher probe；这些真实探测的 Action=NOT_SENT，不能伪造 GOAL_ACCEPTED。各轮冷启动实体初态、visual、collision、原始激光、实际姿态、两次不同相位暂停与恢复、进程退出均有独立原始记录。暂停证据要求新接收的源 Pose_V/stats，重复 scene 缓存不能通过。接触自检在两个位置有正例和旧位置负例，只评价圆柱局部设施，不扩展为导航无碰撞。

正式触发绑定本轮唯一接受 UUID；同一触发重复不重置。常驻 Gazebo Transport 客户端按整数仿真纳秒推进，暂停期间不推进相位；单请求在途，名义20ms周期、最大50ms间隔，0.5墙钟秒请求期限。请求 pose、服务 Boolean 和实际 Pose_V 分开保存。最大轨迹误差限20mm，激光表面匹配限50mm，均未因失败放宽。原始 world/map/robot、目标、初值、MPPI参数和公共约束继承 P0-04/P0-05，四轮 controller_server 实际参数相同，见 parameter-comparison-final.json。

安装接口为 Gazebo Sim8 / Transport13 / Msgs10 / Sensors8 的已有 set_pose、clock、stats、pose/info、scene/info 与局部 contact；不是反复 CLI+sleep 驱动。原 P0-05 run_trial.py 的锁、期限、worker、rosbag2 和进程监督保持唯一生命周期。宿主包装器仅采样负载并调用同一 runner，不重建 ROS 生命周期。

实际物理激光外参为 top-model→sensor xyz=[-0.064,0,0.15]，不是 ROS base_scan TF 的0.121高度。保留实际机器人俯仰和四元数，三维旋转与有限高度圆柱射线复算；不把姿态压平。360束、5Hz，实际 time_increment=scan_time=0，并结合安装源接口确定瞬时扫描语义。map/world采用经柱体和栅格核实的名义对应，栅格地标最大差约0.057m，不宣称像素级严格相等；AMCL不作为独立真值。

p0_06_geometry-r02.json 的 status 仍保留首次校准时 real_probe_01_derived_pending_full_facilities 字样，属于历史注记；实际设施准备与晋级结论由后续独立真实 probe 和带指纹的 ready-proof 决定，没有修改旧文件消除表面不一致。

ROS/Gazebo先按相同源stamp有界关联（接收差≤100ms），保留原始接收最近匹配及每1墙钟秒窗口中位数锚定≤25ms；无轨迹时间平移或拟合。缺失、非有限、实体变更、乱序、回跳及对齐失败都不能补零。CPU跨主机时钟只用独立SSH往返时间估计并裁剪不确定边界，与轨迹源时间对齐分离。

S3标签要求同一实体先见—同一静态柱前景遮挡—恢复，连续扫描，阶段各≥1秒/5帧。partial、无效值、缺消息、超距、实体消失保留 partial/unknown，不能当遮挡。原始真值/未来轨迹/离线标签只进入 sidecar 与评价，实际 Nav2 订阅端点核查 truth_leak_detected=false；没有真值ROS发布者。

S1已有实际机器人路线与圆柱路线交点及先后时刻，但圆柱明显早于机器人经过交点；这是路径交汇和时序证据，不是同时近距离相遇或高难度避障证明。不能从四轮成功推导成功率、控制器排名、实时性或实车安全。

宿主低负载含系统自然负载，不代表空闲；较高负载使用本任务两条 nice=19、每条≤120秒、可归属的计算进程。实际运动窗口 CPU 差值见 formal-load-contrast-release.json；两场景均须增加≥0.10才承认不同条件。原始入口 host-evidence-10（S3）/host-evidence-12（S1）、load-formal-*.json，保留 Mac/Ubuntu CPU、Gazebo RTF、PID、CPU用时和退出证据。未更改虚拟机资源配额。

## SDF 诊断与修复依据

原 P0-04 世界及全部派生世界 gz sdf -k 仍退出255，stderr相同：Non-unique names detected in <link name='symbol'>。具体为官方 turtlebot3_world/ros_symbol 的 symbol link 中15组 collision/visual 共享名称；两份官方模型版本均可复现。新圆柱单体检查退出0。原始诊断、资源SHA256见 sdf-recheck.json 和 native-final.json。没有改 /opt/ros 官方资产，也没有把静态检查改写为通过。新世界能实际加载的结论来自各自真实探测及正式记录，不能替代此项静态诊断。

最小修复及其证据如下（每次失败原回合保留，使用新revision/run_id）：

- 接触传感器 topic 归入 contact 子元素、排空删除应答后确认实体不存在；真实两位置接触与旧位置负例复核。
- 完整3D姿态/扫描外参、暂停重复源stamp微小稳定差按首锚限幅、冻结分析窗口及 partial 过渡段语义；不外推、不把 unknown 当遮挡。
- 恢复时 stats/clock 到达竞态：有界恢复并按实际源时间复位调度；atomic-pause 将 quiesce/排空/pause 合成单一有界操作，保持50ms限。见 resume-race-fix*、atomic-pause-fix*。
- 有相同源stamp但回调排队的时钟关联，增加未平移接收锚；反例检出持续±60ms偏移。见 clock-source-join*、source-join-anchor*。
- 原 nav_goal.py 增加录制边界的有界可靠发布确认与新鲜clock推进，再关闭 recorder；MCAP内容检查仍独立。见 marker-drain-fix.diff 和 marker-drain-ros.xml。
- c8 中轨迹在13.1仿真秒完成，但父进程在较晚 Action 结束时先检查当前时间造成误判。改为先校验实际最后请求的实体/结束源时刻/终点，并仍要求实际完成≤原30秒窗口；未完成者仍超时。见 completion-observation-fix.diff/basis.json 和18例回归。
- c5 Action虽SUCCEEDED，但成功后定位误差0.252487m超过0.25m，因此原回合仍失败。未放宽到达限或改变 goal checker；r09改圆柱运动路径并恢复原静态目标，先做几何余量与两个负载probe，再正式采集。c1/c3等失败目标/轨迹版本完整保留，r08未加载候选不算真实回合。

c10 最初5个成功后新鲜定位样本在限内，随后停车位移使最终误差0.253107m；没有选择有利早样本改写失败。安装库隔离复现验证stateful位置锁存语义（goal-checker-reproduction.json），但不冒充该轮内部锁存日志。r10只反转圆柱方向并清至右侧，基于原始路径偏移提出几何假设，先核对地图/静态柱/出生余量再重新做两负载probe。任务、基线、阈值均未改；新回合只能证明该版本实际结果，不能保证任意导航永不超限。

## 实际测试、回归及文件差异

最终非ROS测试：`{'tests': 278, 'failures': 0, 'errors': 0, 'skipped': 0}`；隔离域72 ROS/Action/录包测试：`{'tests': 26, 'failures': 0, 'errors': 0, 'skipped': 0}`。命令、真实退出码、JUnit、stdout在 verification-commands-final.json 与 final-*.log/xml/exit。C++当前 build-04.exit=0、ctest-04.exit=0（2项），二进制身份保存在每轮 artifacts；没有把编译或合成测试当真实场景验收。旧五轮合同复查、静态MPPI/DWB只读入口、v1/v2反例和两个ready门均检查。
覆盖轨迹边界、两种恢复顺序、暂停/回跳、重复触发、实体变化、缺失/乱序、源时间偏移、无原始扫描不能误判可见/遮挡、按时完成被迟观察与真正逾时区分、Action拒绝/取消/未知终态及录包边界。失败测试日志未覆盖，完整命令尝试索引见 development-attempts-final-02.json。

职责：scene_client.cpp/timeline.hpp 为原生时间与Transport；dynamic_geometry.py 为3D实际姿态/射线；inspect_dynamic_recording.py 为关闭后的原始证据复算；runner_dynamic.py 为原runner有限扩展；nav_goal.py仅扩展录制边界确认；p0_06_host_trial.py为宿主采样/有界负载；场景/轨迹/profile/资源锁为冻结输入；tests为针对性反例和兼容性。

相对本次续作起点修改文件：

- `experiments/motions/crossing.yaml`
- `experiments/p0_06_dependencies.json`
- `experiments/p0_06_native_dependencies.json`
- `experiments/profiles/p0_06_crossing.yaml`
- `experiments/profiles/p0_06_observed_occlusion.yaml`
- `experiments/profiles/p0_06_probe.yaml`
- `experiments/scenarios/crossing.yaml`
- `experiments/scenarios/observed_occlusion.yaml`
- `experiments/scenarios/p0_06_probe.yaml`
- `experiments/schemas/dynamic_report.schema.json`
- `src/cmr_sim/include/cmr_sim/timeline.hpp`
- `src/cmr_sim/src/scene_client.cpp`
- `src/cmr_sim/test/scene_timing_test.cpp`
- `tests/test_dynamic_contract.py`
- `tests/test_dynamic_recording.py`
- `tests/test_dynamic_ros_contract.py`
- `tests/test_dynamic_runner.py`
- `tools/dynamic_geometry.py`
- `tools/inspect_dynamic_recording.py`
- `tools/nav_goal.py`
- `tools/prepare_dynamic_probe.py`
- `tools/runner_dynamic.py`

相对本次续作起点新增文件：

- `experiments/motions/crossing_r10.yaml`
- `experiments/p0_06_dependencies-r10.json`
- `experiments/profiles/p0_06_crossing_r10_probe.yaml`
- `experiments/scenarios/p0_06_crossing_r10_probe.yaml`
- `src/cmr_sim/worlds/p0_06_crossing_r10.sdf`
- `experiments/goals/p0_06_crossing_r06.json`
- `experiments/motions/crossing_r06.yaml`
- `experiments/motions/crossing_r07.yaml`
- `experiments/motions/crossing_r08.yaml`
- `experiments/motions/crossing_r09.yaml`
- `experiments/p0_06_dependencies-r06.json`
- `experiments/p0_06_dependencies-r07.json`
- `experiments/p0_06_dependencies-r08.json`
- `experiments/p0_06_dependencies-r09.json`
- `experiments/p0_06_geometry-r02.json`
- `experiments/profiles/p0_06_crossing_r06_probe.yaml`
- `experiments/profiles/p0_06_crossing_r07_probe.yaml`
- `experiments/profiles/p0_06_crossing_r08_probe.yaml`
- `experiments/profiles/p0_06_crossing_r09_probe.yaml`
- `experiments/scenarios/p0_06_crossing_r06_probe.yaml`
- `experiments/scenarios/p0_06_crossing_r07_probe.yaml`
- `experiments/scenarios/p0_06_crossing_r08_probe.yaml`
- `experiments/scenarios/p0_06_crossing_r09_probe.yaml`
- `src/cmr_sim/worlds/p0_06_crossing_r06.sdf`
- `src/cmr_sim/worlds/p0_06_crossing_r07.sdf`
- `src/cmr_sim/worlds/p0_06_crossing_r08.sdf`
- `src/cmr_sim/worlds/p0_06_crossing_r09.sdf`
- `tools/p0_06_host_trial.py`

本报告及更新的使用说明/设计/场景目录一并属于交付。本任务可审查逐行差异在 task-incremental-review.diff；包含并行元数据的完整快照差异在 incremental-review-02.diff，包含未跟踪源码；git-diff.txt仅是Git能显示的部分。源码修改前副本位于 source-before.tgz 与各修复前存档。

c10 的 MCAP 有实际消息、RECORDING_START/END 和覆盖范围；其 bag-check=false 来自统一行为验收的 arrival=false，并非已证明文件损坏。harness=RECORDER_FAILED 是原框架汇总分类，最早阻塞见 failures.json 的 OBSERVE_STOP。报告按首因解释，不改历史状态。个别可选节点参数查询不完整时保持未知，不能由公共配置hash代替全部运行参数证据。

## 所有真实尝试与失败保留

汇总实际发现 47 个 P0-06 回合，分类 `{'real': 47, 'fixture': 0, 'incomplete_or_missing': 0, 'invalid': 0, 'validator_error': 0}`。以下不按最新目录推断结果，也不把旧失败重算成功后改写；每轮原始 result/exit/bag/dynamic/cleanup 保留。

回合|Action|harness|cleanup|原首因
---|---|---|---|---
`p0-06-crossing-kwdqt70u-c1`|CANCELED|NAV_TIMEOUT|FAILED|{"code": "GOAL_DEADLINE", "phase": "WAIT_RESULT", "detail": "Terminal not received before deadline", "monotonic_s": 160156.835887243}
`p0-06-crossing-kwdqt70u-c10`|SUCCEEDED|RECORDER_FAILED|SUCCEEDED|{"code": "RuntimeError", "phase": "OBSERVE_STOP", "detail": "behavioral gate failed {'action': True, 'movement': True, 'arrival': False, 'commands': True, 'stop': True}", "monotonic_s": 170316.632381837}
`p0-06-crossing-kwdqt70u-c11`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-crossing-kwdqt70u-c12`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-crossing-kwdqt70u-c3`|ABORTED|INTERNAL_ERROR|FAILED|{"code": "ACTION_ABORTED", "phase": "WAIT_RESULT", "detail": "ABORTED", "monotonic_s": 161888.522631277}
`p0-06-crossing-kwdqt70u-c5`|SUCCEEDED|RECORDER_FAILED|SUCCEEDED|{"code": "RuntimeError", "phase": "OBSERVE_STOP", "detail": "behavioral gate failed {'action': True, 'movement': True, 'arrival': False, 'commands': True, 'stop': True}", "monotonic_s": 166774.986419315}
`p0-06-crossing-kwdqt70u-c7`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-crossing-kwdqt70u-c8`|SUCCEEDED|RECORDER_FAILED|FAILED|{"code": "TimeoutError", "phase": "WAIT_RESULT", "detail": "SCENE_WINDOW_EXCEEDED", "monotonic_s": 167998.431162148}
`p0-06-crossing-kwdqt70u-c9`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-final05-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-final06-low`|NOT_SENT|RECORDER_FAILED|SUCCEEDED|{"code": "BAG_INVALID", "phase": "INSPECT_BAG", "detail": "['missing recording interval markers']", "monotonic_s": 166023.285341207}
`p0-06-crossing-probe-kwdqt70u-final07-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-final07-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-final08-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-final08-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-r06-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-r06-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-r07-higher`|NOT_SENT|INTERNAL_ERROR|FAILED|{"code": "MOTION_GAP", "phase": "NAV_READY", "detail": "MOTION_GAP: {'code': 'MOTION_GAP'}", "monotonic_s": 162970.779981251}
`p0-06-crossing-probe-kwdqt70u-r07-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-r09-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-r09-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-r10-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-r10-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-resume03-low`|NOT_SENT|INTERNAL_ERROR|FAILED|{"code": "MOTION_REQUEST_GAP", "phase": "NAV_READY", "detail": "MOTION_REQUEST_GAP: {'code': 'MOTION_REQUEST_GAP'}", "monotonic_s": 165105.523591436}
`p0-06-crossing-probe-kwdqt70u-resume04-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-crossing-probe-kwdqt70u-resume04-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-occlusion-kwdqt70u-o1`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-occlusion-kwdqt70u-o2`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-occlusion-kwdqt70u-o3`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-occlusion-kwdqt70u-o4`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-occlusion-kwdqt70u-o5`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-occlusion-kwdqt70u-o6`|SUCCEEDED|COMPLETED|SUCCEEDED|
`p0-06-occlusion-probe-kwdqt70u-01`|NOT_SENT|INTERNAL_ERROR|SUCCEEDED|{"code": "DYNAMIC_RECORDING_INVALID", "phase": "INSPECT_BAG", "detail": "['VISIBILITY_NOT_ESTABLISHED']", "monotonic_s": 157783.857625903}
`p0-06-occlusion-probe-kwdqt70u-02`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-occlusion-probe-kwdqt70u-03`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-occlusion-probe-kwdqt70u-final07-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-occlusion-probe-kwdqt70u-final07-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-occlusion-probe-kwdqt70u-final08-higher`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-occlusion-probe-kwdqt70u-final08-low`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-occlusion-probe-kwdqt70u-resume04-low`|NOT_SENT|INTERNAL_ERROR|SUCCEEDED|{"code": "DYNAMIC_RECORDING_INVALID", "phase": "INSPECT_BAG", "detail": "['CLOCK_ALIGNMENT_FAILED']", "monotonic_s": 165545.878808271}
`p0-06-probe-kwdqt70u-01`|NOT_SENT|INTERNAL_ERROR|FAILED|{"code": "PAUSED_CLOCK_ADVANCED", "phase": "NAV_READY", "detail": "PAUSED_CLOCK_ADVANCED: {'code': 'PAUSED_CLOCK_ADVANCED'}", "monotonic_s": 151061.949363197}
`p0-06-probe-kwdqt70u-02`|NOT_SENT|INTERNAL_ERROR|SUCCEEDED|{"code": "DYNAMIC_RECORDING_INVALID", "phase": "INSPECT_BAG", "detail": "['no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'no extrapolation', 'PROBE_FACILITY_EVIDENCE_INCOMPLETE']", "monotonic_s": 155370.630626169}
`p0-06-probe-kwdqt70u-03`|NOT_SENT|STARTUP_FAILED|SUCCEEDED|{"code": "RuntimeError", "phase": "NAV_READY", "detail": "nav_ready: RuntimeError: goal footprint not free 253", "monotonic_s": 155716.123724583}
`p0-06-probe-kwdqt70u-04`|NOT_SENT|INTERNAL_ERROR|SUCCEEDED|{"code": "DYNAMIC_RECORDING_INVALID", "phase": "INSPECT_BAG", "detail": "['TRUTH_CONFLICTING_DUPLICATE']", "monotonic_s": 157357.729469949}
`p0-06-probe-kwdqt70u-05`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-probe-kwdqt70u-06`|NOT_SENT|COMPLETED|SUCCEEDED|
`p0-06-probe-kwdqt70u-07`|NOT_SENT|COMPLETED|SUCCEEDED|

回合准确绝对路径及所有诊断在 index-final.json；每行基目录均为 `/home/robot/commercial_mobile_robot/evidence/runs/`。旧已通过的 O1/O2/O3/O4/C7 可作额外历史证据，但本次四格验收只使用上表当前统一冻结版本。无结果/中断/INVALID按实际汇总独立保留，不补造终态。

## 并行 Git 变化审查

本任务没有执行 commit、push、reset、clean。执行期间 HEAD 从 `0062f76245ca71c1806548457b50448a3db8a8c6` 变为 `5ce66527148d2de8a45d1e0a987c10cb2ef0095c`，reflog/commit 原始输出保存在 external-git-change-review.json。不能仅凭Git作者字段推断实际操作人。首次最终审计因“HEAD必须不变”断言退出1，原日志和JSON原样保留；本次没有把 head_unchanged=false 改成true。

补充检查确认：2297个起点历史文件内容一致，保护文件未改变；当前两场景全部冻结依赖与场景/profile/准备证据指纹仍匹配，四轮关闭后合同、bag与原始动态复算通过。并行提交把 evidence 从当前Git索引移出，但磁盘文件未因此消失。P0-07在当前工作区可接入；复制或重新clone时必须另行携带证据包，不能认为只clone当前HEAD就包含真实记录。

初始source_hashes仅覆盖该次所选源码范围；`.gitignore`、`.vscode`设置、AI_USAGE.md、LICENSE、THIRD_PARTY.md在起点Git树已经存在，不是本任务新增文件。它们从本任务新增清单与 task-incremental-review.diff 中排除；完整快照差异仍单独保留，`.gitignore`的并行变化不回退。实际归属与校验入口为 git-drift-impact-review.json。

首次尝试启动o5曾因自动审批用量限制而未执行；随后恢复执行时验证目录不存在、项目锁空闲，再创建了真实o5。没有虚构中断回合。该工具阻塞单列在 execution-interruption-review.json。

## 操作与下一阶段

已验证的完整命令见更新后的 docs/p0_06_usage.md；启动命令会创建新回合并拒绝同名目录，不能用原验收ID覆盖重跑。四轮正式命令实际执行；只读 --check 不创建回合、不发目标。

P0-07 可以接入四轮原始 MCAP、实际 Pose_V、源时间/epoch、实体映射、触发事件、profile/资源锁和负载证据，建立独立评价。尚不能宣称完整碰撞真值、安全距离评价已完成：本次 collision=null，运动学移动的离散接触/穿透边界及完整机器人碰撞几何评价仍属下一阶段。已确认物理激光外参只适用于本次资源身份。

仍待本人确认：亲自解释轨迹/时间合同和失败边界；RViz/Gazebo目视观察；是否接受S1较大领先时间的场景难度。工程事实与个人签署分开，不代签。若希望更紧迫的同时交汇，应另立场景revision、重新独立probe和冻结后真实运行，不能事后改标签。

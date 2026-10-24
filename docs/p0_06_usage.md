# P0-06 使用说明（目前只到第6步审查门）

工作区 /home/robot/commercial_mobile_robot，Ubuntu ARM64。当前三份动态场景均draft，不能执行。原P0-05静态入口保留。本文件后面的真实命令仅供本人审查确认后使用，本次未执行。

## 已执行的编译和离线检查

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/commercial_mobile_robot
colcon --log-base log/p0-06-a3179_6v-11 build --packages-select cmr_sim --build-base build/p0-06-a3179_6v --install-base install/p0-06-a3179_6v
source /home/robot/commercial_mobile_robot/install/p0-06-a3179_6v/setup.bash
ctest --test-dir /home/robot/commercial_mobile_robot/build/p0-06-a3179_6v/cmr_sim --output-on-failure
~~~

加载ROS/本次安装环境是运行原生程序的前提；没有此环境时新增protobuf测试曾找不到libgz-utils2，失败已保留。不要改系统LD_LIBRARY_PATH或全局预加载。Gazebo仍单独使用P0-04/05验证的Boost修正。

~~~bash
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method mppi --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-06-static-check-mppi-20261026 --check
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_05_official_static.yaml --method dwb --profile /home/robot/commercial_mobile_robot/experiments/runner_profile.yaml --run-id p0-06-static-check-dwb-20261026 --check
~~~

以上退出0。新crossing/observed_occlusion/probe draft的--check实际退出2，未生成目录。v2 --require-ready只承认ready_for_recording，ready_for_probe不是正式导航许可。

## 本人确认后，第7步的顺序与命令（本次未运行）

先审查工程报告及代码差异，尤其两派生world与原world的原生SDF检查均255且stderr相同。该继承诊断未改成通过；未完成审查不执行以下命令。主方案仍保留原静态资产，先做不发目标的真实加载/实体探测；若实际加载失败，保留新回合再评审最小修复。

1. 只读确认无外来owner、工作区和输入指纹未变化；先加载上面的ROS/安装环境。
2. --ack-review意味着本人已审查候选坐标、余量、协议、继承SDF诊断及代码；不是对物理/激光的签名。工具创建一个全新准备目录，结构审查和资源hash完成才生成ready_for_probe。物理与外参仍pending。
3. 对准备目录执行--check，确认mode=probe/sends_goal=false、两个C++角色、资源/二进制、期限/清理计划。
4. 只在本人批准后执行同一参数去掉--check。该命令启动仿真、发布定位初值、激活Nav2、录包和有限道具探测，但不生成/发送导航目标。另一轮必须等上一轮清理完毕并使用新编号。

~~~bash
python3 -B tools/prepare_dynamic_probe.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_06_probe.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_probe.yaml --output-dir /home/robot/commercial_mobile_robot/evidence/env/p0-06-probe-preparation-20261026-review01 --ack-review
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/evidence/env/p0-06-probe-preparation-20261026-review01/probe.yaml --method mppi --profile /home/robot/commercial_mobile_robot/evidence/env/p0-06-probe-preparation-20261026-review01/profile.yaml --run-id p0-06-probe-20261026-review01 --check
python3 -B tools/run_trial.py --scenario /home/robot/commercial_mobile_robot/evidence/env/p0-06-probe-preparation-20261026-review01/probe.yaml --method mppi --profile /home/robot/commercial_mobile_robot/evidence/env/p0-06-probe-preparation-20261026-review01/profile.yaml --run-id p0-06-probe-20261026-review01
~~~

S3应使用它自己的不发目标probe，不能拿S1证据冒充。下面只生成S3 probe输入，不自动运行：

~~~bash
python3 -B tools/prepare_dynamic_probe.py --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/observed_occlusion.yaml --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_06_observed_occlusion.yaml --output-dir /home/robot/commercial_mobile_robot/evidence/env/p0-06-occlusion-preparation-20261026-review01 --ack-review --as-probe
~~~

原始资源和旧profile不改写；输出目录/文件已存在则拒绝。probe是固定完整白名单：接口和初态、机器人静止、两个位置接触正负例（第二位置的负例在旧位置）、轨迹及两次计划暂停。不是任意流程脚本。

## 第7步停止条件

接口或类型/实体ID冲突、没有独立新鲜Pose_V、clock不能对齐、初态不符、机器人移动、录包/捕获器不就绪、服务失败、暂停时有在途请求未排空、原生加载错误，均保留该轮并停止。不要手发目标、重发STARTUP或用CLI循环驱动。

目前laser_extrinsic、world_map_alignment、scan_timing_semantics均未冻结。首轮probe可以收集原始证据但可能因这些缺口或暂停缺少实际样本而非零；不能去掉检查获得成功。只收到相同scene缓存回复不能证明暂停位姿不变；缺少新鲜实际pose应标未核实。确认原始数据后生成新revision/新回合，不补写旧证据。

S1候选需实际路线相交且障碍先于机器人经过交点；S3须在移动观察者下获得可见→完整遮挡→恢复，固定机位候选不能代替正式证据。当前几何分析限定已核实的平面外参，非平面truth明确无效而不静默压平。

## 关闭后离线检查与汇总（未来命令）

~~~bash
python3 -B tools/validate_run.py run /home/robot/commercial_mobile_robot/evidence/runs/p0-06-probe-20261026-review01
python3 -B tools/inspect_bag.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-06-probe-20261026-review01 --output /home/robot/commercial_mobile_robot/evidence/env/p0-06-probe-preparation-20261026-review01/bag-review.json
python3 -B tools/inspect_dynamic_recording.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-06-probe-20261026-review01 --output /home/robot/commercial_mobile_robot/evidence/env/p0-06-probe-preparation-20261026-review01/dynamic-review.json
~~~

这些路径目前没有真实回合。检查器不会replay；输出必须新建。VALID只说明记录合同；normal_pass/probe_pass/动态有效性/清理/碰撞评价分别判定。collision保持null。

~~~bash
python3 -B tools/summarize_runs.py --root /home/robot/commercial_mobile_robot/evidence/runs --prefix p0-06- --output /home/robot/commercial_mobile_robot/evidence/env/p0-06-dev-a3179_6v/current-run-index.json
~~~

此只读汇总本次执行时分母为0。当前不执行第8～11步：宿主负载、真实暂停/冷启动验证、正式输入冻结、S1/S3各至少两轮正式记录、个人目视/理解签署均待审查确认。

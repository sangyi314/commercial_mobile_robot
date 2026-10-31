# P1-02 构建、协议测试与证据复核

2026-10-31。运行对象为真实ROS Action协议上的合成配送任务，不是实际机器人导航。先读[Action合同](../contracts/p1-action-v1.md)与[模块导读](../design/p1-action-design.md)。测试使用原Mission FSM/SQLite/Runtime；不启动Gazebo、Bridge或Nav2栈。

## 1. 干净终端与新输出目录

Mac执行ssh cmr-ubuntu，进入Ubuntu后开启bash --noprofile --norc，再逐条执行：

```bash
cd /home/robot/commercial_mobile_robot
git status --short
git rev-parse HEAD
CMR_P102_RUN=$(mktemp -d /home/robot/commercial_mobile_robot/evidence/env/p1-02-review-XXXXXX)
echo "$CMR_P102_RUN"
```

保存打印目录。已有P1-01取消修复是合法未提交改动，不reset。后续失败保留原日志，重跑使用新输出目录；不要清理未知进程。

## 2. OFF纯核心

```bash
cmake -S src/cmr_mission -B "$CMR_P102_RUN/pure-build" -DCMR_MISSION_BUILD_ROS=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$CMR_P102_RUN/pure-build" --parallel 2
ctest --test-dir "$CMR_P102_RUN/pure-build" -N
ctest --test-dir "$CMR_P102_RUN/pure-build" --output-on-failure --no-tests=error
ldd "$CMR_P102_RUN/pure-build/mission_probe"
```

只创建编译/SQLite合成测试产物，不创建ROS节点；预期17组，链接不包含rcl/rmw/Nav2。保留P1-01原16组，新增共用ActionCorrelation边界一组。测试数据库在各构建目录内，不重复利用已执行过的DB目录。

## 3. ON构建与安装来源

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base "$CMR_P102_RUN/build-log" build --base-paths src --packages-up-to cmr_mission --build-base "$CMR_P102_RUN/ros-build" --install-base "$CMR_P102_RUN/ros-install" --cmake-args -DCMR_MISSION_BUILD_ROS=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source "$CMR_P102_RUN/ros-install/local_setup.bash"
ros2 pkg prefix cmr_mission
ros2 pkg prefix cmr_interfaces
```

两个prefix均须来自本轮ros-install。只构建这两个工作区包及已有系统依赖，不启动任何导航进程。核心/适配目标分离；普通colcon核心测试不会默认启动协议会话。

```bash
colcon --log-base "$CMR_P102_RUN/test-log" test --packages-select cmr_mission --build-base "$CMR_P102_RUN/ros-build" --install-base "$CMR_P102_RUN/ros-install" --event-handlers console_direct+
colcon test-result --test-result-base "$CMR_P102_RUN/ros-build" --verbose
```

预期17组核心/关联测试；不把这17组称为真实ROS协议矩阵。

## 4. 明确隔离及零副作用预检

```bash
export ROS_DOMAIN_ID=72
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 -B tools/mission/run_protocol_tests.py --help
python3 -B tools/mission/run_protocol_tests.py --suite experiments/mission/action-suite-v1.yaml --install-prefix "$CMR_P102_RUN/ros-install" --output-dir "$CMR_P102_RUN/protocol-01" --check
test ! -e "$CMR_P102_RUN/protocol-01"
```

预检只读suite、二进制和输出路径，不导入rclpy、不创建节点/DB/目录、不启动daemon。若prefix错误或输出已存在应拒绝。ROS_LOCALHOST_ONLY在本机Jazzy显示deprecated但明确honored，日志保留；没有把domain编号当完整安全隔离。

## 5. 完整协议矩阵与单案定位

```bash
python3 -B tools/mission/run_protocol_tests.py --suite experiments/mission/action-suite-v1.yaml --install-prefix "$CMR_P102_RUN/ros-install" --output-dir "$CMR_P102_RUN/protocol-01"
```

每案新namespace /cmr_test/p102_<随机ID>_Axx、独立Mission进程、Fake、数据库、日志；监督器先检查图和类型，只回收本轮Popen进程组。逐案失败即停止后续，结果中未执行不填通过。A17默认3客户端、20seed×200业务请求；不是只在内存随机调用FSM。

只复查晚接受或取消可用新目录：

```bash
python3 -B tools/mission/run_protocol_tests.py --suite experiments/mission/action-suite-v1.yaml --install-prefix "$CMR_P102_RUN/ros-install" --output-dir "$CMR_P102_RUN/late-accept-01" --cases A06,A08
```

A11的非法重复/错误UUID/乱序是实际适配器共用纯关联边界注入，另有ROS fixture错身份；不把它称为非法线协议验证。A19的错误类型属于实际发现图预检。A20预期正式模式exit2且没有任务DB，这种拒绝用例通过不表示正式执行能力已实现。

## 6. 真实输入重放与独立审计

```bash
python3 -B tools/mission/run_protocol_tests.py --suite experiments/mission/action-suite-v1.yaml --install-prefix "$CMR_P102_RUN/ros-install" --output-dir "$CMR_P102_RUN/replay-01" --cases A17 --replay-input "$CMR_P102_RUN/protocol-01/A17/replay-input.jsonl" --replay-limit 200
python3 -B tools/mission/audit_protocol.py "$CMR_P102_RUN/protocol-01" "$CMR_P102_RUN/protocol-01-audit.json"
```

重放例子取保存输入的前200条；省略limit可重放全文件。ROS调度不保证逐帧相同，保存的Mission BUSINESS_RX序号与输入command_id关联，审计报告恢复实际接收顺序。

每案trace.json保存请求/响应/身份/DB快照，server*.log独立保存Goal/Cancel UUID和活动峰值，mission*.log保存Native Send调用与owner事件，cleanup.json区分计划SIGKILL和正常退出。result分startup/protocol/mission/fault_activation/harness/cleanup/evidence。审计比较Native Send数与独立服务端请求数，不要求“INTENT数=实际Send数”：readiness拒绝存在意图但从未调用Send，必须明确区分。

## 7. 业务服务与Fixture读法

接口定义：src/cmr_interfaces/srv/MissionCommand.srv。operation为submit/query/cancel/handover。Submit业务字段依S1规则，测试站点a→b、map/stations版本固定。Query返回持久状态/版本/身份；detail是诊断用Record v1编码，不作为外部长期业务序列化合同。Cancel/Handover的command_id作用域是mission_id；相同ID不同操作/expected版本返回COMMAND_CONFLICT。重复已终态任务可返回ALREADY_TERMINAL，不重执行。

SyntheticEvent仅在test_mode=true的隔离配置存在。它提供Stop/Arrival/Resolve等外部事实，不会因Server SUCCEEDED自动触发。server control不负责“证明停车”。正常两段流程的服务调用及具体参数均在A01 trace.json中，不需要凭空填写真实站点坐标。

## 8. Sanitizer与已知边界

本轮已对核心17组运行ASan/UBSan，并对13个实际ROS重点用例运行instrumented节点/Fake。依赖包未全部重新instrument，因此不声称覆盖整个ROS中间件的内存正确性。

TSan已实际尝试，两组测试在测试体前报unexpected memory mapping；同编译器最小空程序exit66并报相同错误。原始失败保留，TSan不计通过；替代检查为单线程executor、mutex/atomic边界、确定性并发/退出与ASan/UBSan，不等同TSan证明。

正常关闭有1600ms协议收尾窗口，关键Port命令只入队、不等网络。SQLite等OS不可中断I/O不提供硬实时保证。SIGKILL/Server失联可留下未知外部Goal，重启继续暂停，关闭Client或Fake清理不是实际停车。

P1-03交接：保留NavigationAdapter和Mission语义，以真实ExecutionPort替换合成授权/停止入口；本轮不实现Gate，不宣称Gazebo/实车/物理停止/v0.2发布。

# P1-02 构建、协议测试与证据复核

当前 A01–A20 通过 DeliveryCommand 运行 Mission Node、ExecutionPort、Gate 和 Observer。Fake NavigateToPose 与隔离 sink 提供合成 Action、定位和运动数据。2026-10-31 的原始矩阵与证据继续见[历史验收记录](../verification/p1-02-action-20261031.md)。

## 1. 干净终端与新输出目录

Mac执行ssh cmr-ubuntu，进入Ubuntu后开启bash --noprofile --norc，再逐条执行：

```bash
cd /home/robot/commercial_mobile_robot
git status --short
git rev-parse HEAD
CMR_P102_RUN=$(mktemp -d /home/robot/commercial_mobile_robot/evidence/env/p1-02-review-XXXXXX)
echo "$CMR_P102_RUN"
```

保存打印目录。保留工作区已有改动。后续失败保留原日志，重跑使用新输出目录；不要清理未知进程。

## 2. OFF纯核心

```bash
cmake -S src/cmr_mission -B "$CMR_P102_RUN/pure-build" -DCMR_MISSION_BUILD_ROS=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$CMR_P102_RUN/pure-build" --parallel 2
ctest --test-dir "$CMR_P102_RUN/pure-build" -N
ctest --test-dir "$CMR_P102_RUN/pure-build" --output-on-failure --no-tests=error
ldd "$CMR_P102_RUN/pure-build/mission_probe"
```

只创建编译/SQLite合成测试产物，不创建ROS节点；用 ctest -N 核对当前源码登记的测试，ldd 输出中应无 rcl/rmw/Nav2。测试数据库在各构建目录内，不重复利用已执行过的DB目录。

## 3. ON构建与安装来源

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base "$CMR_P102_RUN/build-log" build --base-paths src --packages-up-to cmr_mission --build-base "$CMR_P102_RUN/ros-build" --install-base "$CMR_P102_RUN/ros-install" --cmake-args -DCMR_MISSION_BUILD_ROS=ON -DCMR_EXECUTION_BUILD_ROS=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source "$CMR_P102_RUN/ros-install/local_setup.bash"
ros2 pkg prefix cmr_mission
ros2 pkg prefix cmr_interfaces
ros2 pkg prefix cmr_execution
```

Mission、Interfaces 及 Execution 的 prefix 均须来自本轮 ros-install。构建只处理 cmr_mission 及其工作区依赖。核心/适配目标分离；普通colcon核心测试不会默认启动协议会话。

```bash
colcon --log-base "$CMR_P102_RUN/test-log" test --packages-select cmr_mission --build-base "$CMR_P102_RUN/ros-build" --install-base "$CMR_P102_RUN/ros-install" --event-handlers console_direct+
colcon test-result --test-result-base "$CMR_P102_RUN/ros-build" --verbose
```

CTest 覆盖纯核心、Action 关联及 ExecutionPort 容量与生命周期。下节协议矩阵另起独立 ROS 进程。

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

每案使用独立 /cmr_test 命名空间、Mission、Gate、Observer、Fake、数据库和日志。监督器先检查图和类型，只回收本轮子进程组。A17 默认使用三个客户端发送 20 个 seed 各 200 条真实 ROS 业务请求。

只复查晚接受或取消可用新目录：

```bash
python3 -B tools/mission/run_protocol_tests.py --suite experiments/mission/action-suite-v1.yaml --install-prefix "$CMR_P102_RUN/ros-install" --output-dir "$CMR_P102_RUN/late-accept-01" --cases A06,A08
```

A03 检查缺少 Action Server 时配送准入拒绝，适配器发送前 readiness 超时由 CTest 独立覆盖。A11 运行 ActionCorrelation 错身份和乱序用例，并验证 ROS 状态查询。A14 通过私有源时钟测试暂停和回退。A16/A18 用真实 SQLite 写锁建立 owner 阻塞，分别检查过载和等待中的提交遇到关闭。A19 验证错误图配置和输出目录拒绝。A20 检查域、Action、执行命名空间和时钟配置拒绝，以及 ROS 对未声明覆盖项的处理。

## 6. 真实输入重放与独立审计

```bash
python3 -B tools/mission/run_protocol_tests.py --suite experiments/mission/action-suite-v1.yaml --install-prefix "$CMR_P102_RUN/ros-install" --output-dir "$CMR_P102_RUN/replay-01" --cases A17 --replay-input "$CMR_P102_RUN/protocol-01/A17/replay-input.jsonl" --replay-limit 200
python3 -B tools/mission/audit_protocol.py "$CMR_P102_RUN/protocol-01" "$CMR_P102_RUN/protocol-01-audit.json"
```

重放例子取保存输入的前200条；省略limit可重放全文件。ROS调度不保证逐帧相同，保存的 Mission DELIVERY_RX 序号与输入 command_id 关联，审计报告恢复实际接收顺序。

每案trace.json保存请求/响应/身份/DB快照，server*.log独立保存Goal/Cancel UUID和活动峰值，mission*.log保存Native Send调用与owner事件，cleanup.json区分计划SIGKILL和正常退出。result分startup/protocol/mission/fault_activation/harness/cleanup/evidence。审计比较Native Send数与独立服务端请求数，不要求“INTENT数=实际Send数”：readiness拒绝存在意图但从未调用Send，必须明确区分。

## 7. 当前业务服务与停止证据

业务写请求使用 DeliveryCommand。MissionCommand 提供状态查询，写请求返回 DELIVERY_API_REQUIRED。交接携带 handover_id 和 handover_entry_version，重复请求按 command_id 和业务内容判定。

Fake Server 的结果和取消回调进入原 NavigationAdapter。Gate 和 Observer 分别提供授权及停止证据，DeliveryMonitor 从合成传感器和 TF 检查到达。正常两段流程的实际服务参数见当轮 A01 trace.json。

## 8. 测试范围

协议矩阵使用合成输入，验证 Action 关联、持久化、授权、停止证据和业务状态。历史 ASan/UBSan 与 TSan 结果见 P1-02 验收记录。正常关闭有 1600ms 协议收尾窗口。SIGKILL 或 Server 失联可以留下未知外部 Goal，重启保留 PAUSED 和活动槽。

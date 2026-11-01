# cmr_execution

独立 C++17 运动授权核心、ROS Gate 和只读停止观察器。Gate 仅接纳合法租约与新鲜候选；不启动 Nav2/Gazebo，不发送 Action Goal，不写业务数据库。

- 核心：include/cmr_execution/gate.hpp、stop.hpp。
- ROS：src/gate_node.cpp、stop_observer.cpp。
- 可选隔离夹具：candidate_relay.cpp，保持值与stamp，SIGUSR1停流、SIGUSR2恢复；无授权能力。正式r20链为Collision Monitor直连Gate，不运行relay；G05短暂暂停本轮CM进程测试停流。
- 派生导航：launch/navigation_gate.launch.py，明确仿真时间、无 docking、无 composition/respawn、手动生命周期。
- 真实 Mission 适配位于 cmr_mission，不复制状态机。

独立 OFF 构建：cmake -S src/cmr_execution -B <新目录> -DCMR_EXECUTION_BUILD_ROS=OFF；随后 cmake --build 与 ctest。
ON 构建、运行边界、故障工具及教学详见 docs/guides/execution-gate.md；验收见 docs/verification/p1-03-un63b3ef.md。

控制状态不等于物理停止。消费者 ACK、独立下游 watchdog、实车安全未证明，禁止在运动仿真中通过杀 Gate 冒充 owner 失效测试。

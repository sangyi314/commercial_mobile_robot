# P2-01 TF 权属

| ROS 边 | P1 与 P2 shadow | P2 接管候选 |
|---|---|---|
| map → odom | AMCL，`tf_broadcast=true` | AMCL |
| odom → base_footprint | Gazebo DiffDrive 经 execution bridge 转发 /tf | robot_localization |
| base_footprint → base_link | robot_state_publisher /tf_static | 同左 |
| base_link → base_scan、imu_link | robot_state_publisher /tf_static | 同左 |

Shadow 配置必须 publish_tf=false。接管先在新运行的派生 bridge 中移除 DiffDrive 到 /tf 的 GZ_TO_ROS 路由，raw /odom 保留。
原 P1 bridge 和 Gazebo 速度路由独立保留。运行验证按实际 TF 消息边和 GID 计数，不能以全局 /tf 发布者数量代替每条边的权属。

回退先由原 Runner 结束会话并回收 EKF，再使用原 P1 profile 开新会话。
两种模式分别记录 bag、运行 Publisher/GID 和配置。现阶段本表是静态绑定，运行结果以证据目录为准。

## 执行切换和回退

1. 完整停止当前 Runner 会话，核查其 cleanup 记录和命名空间进程已退出。
2. 由相同源码、配置与校准模型的 shadow 通过报告和 P1 完整回归通过报告构造有哈希的资格输入。通过 Runner 的 `--check` 检查新派生 profile。挂接入口在启动 EKF 前观察动态边，拒绝已存在的 odom → base_footprint 及未预期的 /tf 发布者。
3. 使用新的 run_id 启动 Runner。派生 bridge 保留 /odom 和原 Gate 路由，移除 DiffDrive 动态 TF 的 ROS 转发。再次验证完整 TF 权属、时间合同、参数、输出 bag 和原 P1 回归。
4. 回退时按第 1 步停止接管会话。得到 EKF 已退出记录后，用原版 P1 profile 和新 run_id 启动，不附加 `--p2-profile`，检查 bridge 独占该动态边并重复 P1 受影响场景。

本轮 shadow 的边/GID 原始观测见 `tf-shadow-normal-observation.json`。当 RMW 初始发现只提供 GID、节点名未知时，记录保留该未知状态。正式接管受原 P1 完整回归门约束，不能用 shadow 成功、单独启动 EKF、或默认 P1 启动替代切换和回退的实测记录。

三轮真 Gazebo 的私有 TF 双源注入通过。正常 shadow 的边/GID 核验见 `tf-shadow-normal-proof.json`，含 15 个权属明确的试验会话。使用实际失败资格报告的 replacement 负向预检在 `verification-current/replacement-negative/preflight.log` 中返回拒绝。此拒绝是门控验证，正式切换未执行。

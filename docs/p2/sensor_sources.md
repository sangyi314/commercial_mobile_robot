# P2-01 传感器来源

安装资产 `/opt/ros/jazzy/share/nav2_minimal_tb3_sim`。SDF、URDF、bridge 和 Nav2 的配置指纹见本次 `r0/sensor-config.sha256`。

| 输入 | 已核配置与来源 | 配置频率 | 当前运行实测 |
|---|---|---:|---|
| /scan | GPU LiDAR → ros_gz bridge，base_scan，360 束。LaserScan 桥接的 time_increment/scan_time 由审查记录为零 | 5 Hz | 待新采集 |
| /odom | DiffDrive 读取左右轮 JointPosition 并积分，桥接到 odom / base_footprint。轮径半径 0.033 m、轮距 0.287 m | 30 Hz | 待新采集 |
| /imu | 模拟 IMU → bridge，imu_link。SDF 的角速度逐轴高斯标准差为 0.0002 rad/s | 200 Hz | 待新采集 |
| /joint_states | Gazebo JointStatePublisher → bridge | 30 Hz | 待新采集 |
| P1 私有 body_odom | Gazebo OdometryPublisher 读 world pose，用于原 P1 停车辅助 | 50 Hz | P2 的 EKF 输入排除 |

历史两袋的 Header 样本频率约为 scan 5 Hz、odom 27.7778 Hz，未录 IMU。历史 scan/odom offered QoS 为 reliable/volatile。运行图、到达频率、延迟、丢样和协方差待实测。
Header stamp、录制时间、适配器接收 ROS 时间和本地 steady 时间各自记录。历史 wheel 协方差零为未知，需要独立静止和动态残差建立模型。
IMU 仅使用经核外参的角速度 z，姿态和线加速度不参与融合。

GPU LiDAR 的 Gazebo 源码在一次 Update 中渲染并发布整帧，使用单个采样时刻。该静态检查须结合本地版本及旋转扫描的误差评价，当前 timing_mode 为 UNKNOWN。

- [Gazebo DiffDrive](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.cc)
- [Gazebo GPU LiDAR](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/GpuLidarSensor.cc)

## 本轮 Gazebo 运行证据

实施基线发现原 SDF 的 `imu_link` 没有与车体连接。原始桥接可收到 200 Hz 数据，实际车体旋转时陀螺仪 z 仍接近静止噪声。P2 Runner 使用派生 SDF，把同一个传感器挂到 `base_link`，安装位姿取自现有 URDF，ROS frame 继续为 `imu_link`。派生模型同时把 DiffDrive 目标频率设为 100 Hz，以满足 20 ms 内测量时刻 TF 可用的要求。Gazebo 时间步下实际约 83.33 Hz，仍由轮关节反馈积分。

| 话题 | 仿真采样频率 | steady 接收频率范围 | 接收 ROS 年龄 p99 | offered QoS |
|---|---:|---:|---:|---|
| /scan | 5 Hz | 4.97–4.99 Hz | 3–6 ms | reliable / volatile |
| /odom | 83.33 Hz | 82.90–83.16 Hz | 0 ms（仿真时钟量化） | reliable / volatile |
| /imu | 200 Hz | 198.98–199.59 Hz | 0 ms（仿真时钟量化） | reliable / volatile |

表来自三个 60 s 静止采样窗口。0 ms 表示接收回调所见 ROS clock 量化年龄，墙钟传输延迟未由此推断。分组、样本数、异常计数和 QoS 原始记录在本轮 `sensor-frequency-static-window.json` 和 `sensor-audit-shadow/audit.json`。

新模型和独立 Gazebo 参考的校准位于 `calibration-fast-tf-model/`。轮速 vx 方差约 3.0644e-5 (m/s)²，陀螺仪 wz 方差约 4.7032e-5 (rad/s)²。真值通过 Gazebo transport 单独采集。P1 既有独立车体停止校验沿用原链，EKF 的输入只有私有 wheel_odom 和 imu_validated。

激光采样的实测诊断位于 `scan-timing/calibration-observation-02/`。原始字段 `time_increment` 和 `scan_time` 均为零。本机 Gazebo 模型上，静止与双向转动的平面距离中位数约 4–5 mm。人为添加 200 ms 逐束时序时，双向转动残差中位数约 47–60 mm。该观察支持所测 Gazebo 模型是整帧采样，原始 bag 审计仍保留 UNKNOWN 缺省，其他来源须独立资格化。

实际 covariance 效果见 `verification-current/gyro-covariance-binding.json`。所测 raw IMU 角速度 covariance 可用，适配器保留经过外参旋转的角速度 covariance，并使用测得偏置。上一段列出的陀螺仪动态残差方差是未知 covariance 时的备用模型，当前 Gazebo raw IMU 不走该分支。raw orientation covariance 为全零未知，适配输出的 orientation 和 linear_acceleration 均显式标为不可用。

本轮只对已哈希绑定的 Gazebo GPU LiDAR 模型冻结 `INSTANTANEOUS_MODEL`。`scan-timing/qualification.json` 引用静止、双向旋转和时序敏感性对照。采用该时序的 MCAP 复审在 `sensor-audit-shadow-instantaneous/`，572/572 条 scan 的测量时刻 TF 完整。硬件数据和其他模型仍从 UNKNOWN 开始。

`verification-current/wheel-joint-observation.json` 对比了同袋的轮关节位置差分与原始 odom，含 1,729 个已匹配间隔。动态 vx 相关系数 0.9963，wz 相关系数 0.9998。这与 DiffDrive 正在积分关节反馈的源码核查相符。

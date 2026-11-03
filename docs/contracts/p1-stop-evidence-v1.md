# P1 Stop Evidence v1

2026-11-01。独立只读观察，纯判定器不创建ROS节点或发布运动。报告不等于真实底盘认证。

| 层 | 证据 | 限制 |
|---|---|---|
| A | Gate当前控制序号、失效原因、禁止非零 | 仅本地决策 |
| B | 外部订阅最终出口的连续零序列，及Gate输出序号/本地时间 | 不推断Bridge收到；最终Twist剥离源stamp，接收时间另记 |
| C | 明确标Bridge接收/Gazebo总线可见/消费者处理的实际边界 | 没有ACK则消费者处理UNKNOWN；不以B/D反推 |
| D | 独立odom/可用世界pose在推进ROS时间内低速且低漂移的完整窗口 | 不证明实车制动、永久静止或无碰撞 |

初始参数沿用已读P0 stop参考：窗口3s、至少5样本、最大间隔0.5s、线速度≤0.01m/s、角速度≤0.03rad/s、位置漂移≤0.02m、方向漂移≤0.03rad、最终输出各分量≤0.001。作为本轮待冻结参数，运行报告给实际频率、资格与版本；不将旧配置当新系统实测。

窗口绑定run/gate/owner/mission/operation/attempt/lease/clock_epoch及运动generation。Begin/Revoke/Reset/clock变化后重建窗口，不拼接旧事实。严格递增源时间、接收新鲜、无长间隙、无重复/倒序、无跨epoch，所有速度及位姿有限。各点相对窗口首点的位移/角度均检查，不能只比较首尾或相信零速度字段。最终出口不连续零则D不合格。clock不进展或回退即UNKNOWN；恢复后从新generation收集，旧报告失效。

观察器保存原始接收时间与源时间及证据引用。Gate对报告接收新鲜度使用自己的steady时钟；ROS来源年龄只同epoch计算。跨进程绝不直接相减绝对steady时间；失效→外部持续零用同一观察者时钟测量，运动停止延迟/停距只在同ROS epoch内计算。UNKNOWN用null/明确原因，不填0。

合成sink反馈必须标synthetic=true，只证明协议和判定逻辑。Gazebo D需核实DiffDrive由运动状态产生odom；Gate自身不得把发零当STOPPED。A+D及目标已终态可满足Mission取消收尾，C消费者ACK未知仍单列。缺D或Goal未决继续占槽；用户点击不能代替证据。

反例必须覆盖缺样本、重复/倒序、clock暂停/回退、长间隙、旧operation/gate/generation、过期、零速度但漂移、非零最终输出。手算合格序列与独立oracle验证，不能只自调用内部函数。


## P1-04 停止证据修复 v1（2026-11-03，本人批准仅P1仿真Oracle辅助门控）

G03反例证明DiffDrive轮关节积分的odom不能单独支撑车体静止。正式simulation的StopReport必须为`reason=WINDOW_PROVEN_BODY_POSE`：原轮式窗全部通过后，以该精确from/to为窗口边界，再核对独立body反馈。BodyWindow使用包围边界的原始位姿样本，逐对计算平移速度/角速度，同时检查反馈twist、漂移、样本/接收间隔、GID与绑定身份；缺失、冲突、过期、重复/回退、跨generation均不合格。Gate与正式ExecutionPort拒绝旧轮式WINDOW_PROVEN。sink模式仍明确synthetic，只验证协议。所有原3s/0.01mps/0.03radps/0.02m/0.03rad/0.5s/200ms阈值不变。

数据源复用Gazebo Sim 8.15.0已安装OdometryPublisher插件：在派生model上读取worldPose，发布私有`<run>/body_odom`；官方Bridge仅GZ_TO_ROS。其odom_frame=body_world、child=body_base，私有TF不桥接到/tf。它不是轮关节积分；插件twist含滚动均值，因此额外直接计算位姿差分，不只信twist。此源仅供独立停止观察器，不供Nav2定位、目标、地图或速度规划，不证明实车能力。P0原SDF、Bridge、轮式odom和导航TF不变。

开源研究：Gazebo官方gz-sim tag gz-sim8_8.15.0，commit446a44335a45b704b4d36dabcc5508ee34eeb3d8，Apache-2.0；阅读DiffDrive.cc、OdometryPublisher.cc、实际integration test/world。直接复用安装插件，未复制上游系统。ros_gz tag1.0.24 commit5912bfe7976a79d2cc9f373b2d40a8a3a3e33da2，Apache-2.0，复用官方Odometry转换/Bridge。Pose_V→TFMessage转换不保留集合header给各pose，故不拿缺时间的桥接PoseV做停止源。原始源码、hash和缺失路径/403调查记录保存在evidence/env/p1-04-stopfix-ds6qdezr/upstream。CMR双流判定器为参考数据语义后独立实现；无额外框架。

本扩展已获本人明确授权，见`docs/adr/p1-oracle-stop-20261103.md`；记录必须标Oracle辅助，不属于sensor-only E2。

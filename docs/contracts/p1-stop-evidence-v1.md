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

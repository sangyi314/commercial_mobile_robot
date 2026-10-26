# 本轮导航数据流与证据边界

2026-10-22，来源 `p0-03-nq1hmjaf` 的配置、节点/端点检查和冻结输入。本文件记录本轮看见的链路，不代表整体运行成功；[本轮报告](../verification/p0-03-nq1hmjaf.md) 保留 inactive 与拒绝事实。

| 数据 / 链路 | 本轮来源与限界 |
|---|---|
| Gazebo → ros_gz_bridge → /clock | 已有 clock 观测推进；仅两个采样窗口，空白段未知。epoch=p0-03-nq1hmjaf:sim0，会话开始 /clock 未取得，sim_start_s=null |
| Gazebo 激光 → /scan | 类型 sensor_msgs/msg/LaserScan，frame=base_scan；AMCL、local_costmap、Collision Monitor 有订阅记录。非碰撞真值 |
| Gazebo DiffDrive → /odom 与 odom→base_footprint | 新鲜里程计用于位移/静止检查；不能从缺消息补零 |
| robot_state_publisher → 固定机器人 TF | 连接 base_footprint/base_link/base_scan；本轮未重新发布或伪造 TF |
| AMCL → map→odom；与 odom 链合成 map→base | 初值是 AMCL 输入，非出生位置独立真值；后续 map 定位估计不能证明全局地图与物理世界完整对齐 |
| /navigate_to_pose → bt_navigator | 原始唯一目标被 inactive server 拒绝，无 accepted UUID；Action server 存在不表示 ready |
| controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel → bridge/DiffDrive | 本轮静态意图与端点类型为 Twist；FollowPath 插件运行参数是 nav2_mppi_controller::MPPIController。planner/navigator/smoother/monitor inactive，不能说速度链实际跑通 |
| /cmd_vel 额外发布者 | 已有 endpoint 记录还包含 docking_server；不宣称最终速度仅有一个来源或已完成全链隔离 |
| 只读 observer | 订阅 clock/odom/三段 Twist/Action status，读取 TF。原窗口过早结束；补充 15 秒数据只支持清理前静止；无实际 /cmd_vel 样本 |

输入见冻结 manifest/scenario；来源与两种窗口见 runtime/evidence-summary-20261022.json。命令接收单调时间不是控制器生成时间，不能计算可靠端到端延迟。未执行独立碰撞评价、真实硬件或安全停车认证，不凭 Collision Monitor 没有触发填写 collision=false。核心、导航配置、JSON Schema 和冻结输入未改；记录 validator、合同说明与回归测试仅按负责人明确授权作最小修订，122 项回归通过。真实失败 result 为 VALID/0，不代表速度链实际成功运行。

# ADR: P1 仿真停车资格允许 Oracle 辅助门控

2026-11-03，用户已明确决定：“允许仅用于仿真停车资格，明确标为 Oracle 辅助门控”。

背景：G03实际消费的轮式停稳窗口被独立车体记录否定。总纲要求真值只给独立评价器；若其结果影响任务放槽/后续授权，需要明确扩展。本轮已向本人说明该具体数据流，得到上述授权。

决定：仅在P1受控Gazebo实验中，独立停车评价器使用真实worldPose派生的私有body_odom，轮式与车体窗口均合格才发布StopReport；原Gate与真实Mission使用该资格判定。记录`oracle_stop_assisted=true`、`sensor_only_E2=false`。该值不进入Nav2定位/规划/速度计算，不用于算法感知/预测可靠性结论，不扩张为实车能力。

实现参考Gazebo8.15.0官方OdometryPublisher，commit446a44335a45b704b4d36dabcc5508ee34eeb3d8，复用安装组件；双源判定独立实现。所有原阈值不变。缺失、冲突、陈旧、时间跳变继续STOP_UNKNOWN。后续P2/E2传感器实验必须另审查并去除此Oracle辅助边界，不能自动沿用P1结果。

原始决定和测试：evidence/env/p1-04-stopfix-ds6qdezr/truth-boundary-decision.json。G01本人交接当前不可执行，保持待本人实测，不由AI代确认。

# 当前入口：长路线 p0_long_route_v1

map 起点 (-2.0,-0.5,0)，目标 (1.75,-0.5,0)，四元数 (0,0,0,1)。本次实施、实际 run_id、命令顺序、图表、失败保留与验收结论见 [长路线迁移交付](goal_migration_long_route_v1.md)。

静态日常入口：`/home/robot/commercial_mobile_robot/experiments/scenarios/p0_long_route_static_v1.yaml` 与 `/home/robot/commercial_mobile_robot/experiments/profiles/p0_long_route_static_v1.yaml`。动态入口 crossing/observed_occlusion 已切换新目标、新轨迹、新 world 和新 probe 证明；动态仍只使用 MPPI。07 使用各具体 run_id 的 JSON spec，p0_07_*.yaml 只是索引。08 仅同步并静态预检，本次没有重新执行故障矩阵。

下文为保留的旧版本说明/历史命令。旧 runner_profile.yaml 与 p0_05_official_static.yaml 继续作为旧任务兼容入口；旧目标结果不能算作新长路线验证。重新采集必须新 run_id，重新分析必须新 evaluation_id，禁止覆盖。

---

# 当前已验证场景（2026-10-27）

S1使用crossing_r10：圆柱中心从(-2.3,-0.14,0.3)至(-1.4,-0.14,0.3)，6.1仿真秒匀速，8.1秒末结点后保持6秒；实际导航任务继承原短目标。S3保留实际独立准备的observed_occlusion。两场景分别有r10/final08低/较高负载probe和四轮正式原始证据。历史候选保持可追溯，旧候选描述不能覆盖当前冻结输入。S1实际交汇提前较多，只证明路线与先后关系；S3有连续扫描支持V/O/V。个人目视仍待确认。

详细入口：`docs/verification/p0-06-kwdqt70u-live.md`。原设计快照如下，其中“未运行”“待确认外参”等只描述当时状态。

---

# 场景目录（离线候选，未实测）
S0继续用P0-05静态入口；S1=crossing，S2=head_on不实现，S3=observed_occlusion。
官方冻结world嵌套turtlebot3_world→include ros_symbol→link symbol均无pose，按SDF默认identity；九柱collision局部中心x/y∈{-1.1,0,1.1}，z=.25，半径.15，高.5。model.config实际版本选择仍待第7步实体树核对。mesh墙不用于本次确定遮挡标签。
新增圆柱r=.10/h=.60，visual/collision同中心；static表示运动学道具，移位后collision是否跟随未知。
S1候选：机器人原出生(-2,-.5)，沿原P0-04短目标；圆柱(-2.5,0,.3)向(-1.5,0,.3)横移，候选相交点(-2,0)。不得在真实路线未核实前宣布横穿成立。
S3候选：圆柱x=-.6，y从-.3经.28到.8，柱体(-1.1,0)作遮挡；以原出生固定观察点初算中段被挡。导航中观察者会移动，因此固定机位几何不能作为正式遮挡证明。
模型资源闭包和静态几何解析结果见experiments/p0_06_dependencies.json；完整原件和基world不修改。不新增静态柱墙、不生成新地图。
三段各至少1秒、5帧；visible/recovered至少3束符合实际前表面，occluded要求全部目标射线先交同一静态柱且range支持前景。FOV外/掉线/消失/NaN皆unknown。
坐标、目标、误差、激光世界高度、map↔world对应和目视理解待本人及真实probe确认；本目录不构成ready放行证据。

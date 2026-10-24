# P0-07 独立几何评价合同 v1

本合同由 AI 完成工程审查；用户已授权步骤1～11，个人数学、手算和目视签署仍为 null。

输入只取关闭的 real 回合中 Gazebo Pose_V 的独立实际位姿及源整数纳秒，保留原始行号、seq、pose索引、entity ID、epoch、顶层world关系和资源SHA256。实际模型pose、link局部pose、设定pose、服务应答、odom和AMCL分开；后四者不作为评价位置。按原validator验记录，不能凭VALID宣称几何正确。

正式对象仅为 turtlebot3_waffle 与 scripted_obstacle；不包含mesh墙、地面或所有静态物体。机器人圆心选model原点（offset=[0,0,0]），半径由归档SDF全部collision推导：每个形状用3D包围球 rho，取 max(||link平移||+||collision平移||+rho)。box rho=半尺寸对角线，cylinder rho=hypot(radius,length/2)，sphere rho=radius。其水平投影为保守圆包络，不是最小外接圆、不用Nav2 inflation/robot_radius。每个原始Pose_V还以实际link局部姿态重算包含性；不能验证包含性则拒绝。该验证仅覆盖观测源时刻，不替连续真实运动背书。actor取已核验竖直圆柱的水平半径，验证实际link/collision中心及姿态。所有collision均纳入，包括轮子、脚轮、激光和相机；visual mesh不扩展物理collision。

正式统一范围规则在计算距离之前固定：start=唯一目标接受源时刻；end=max(Action终态源时刻+5秒, release+冻结轨迹最后结点+hold)。目标UUID/arm事件必须相同，ROS/GZ对应须有交接证据；原始GZ源时间保持，不平移。全部事件/资源hash写入每轮spec，结束缺失不缩短范围。相同场景各负载使用同规则。P0-06未来轨迹仅用于预先定义观察范围，不进入实际pose或距离。

同epoch/frame/entity验证；数值、bool、非有限、逆序、回跳、冲突重复严格拒绝。相同stamp且内容完全相同可合并并保留原始索引，不能增加持续时间。只选固定请求范围及两端最多max_gap支持区；范围外消息数量记录。不跨50ms原始间隔，不外推，不用插值掩盖原始缺口。两个序列原始断点并集形成线性子段。单点只能说明瞬时测量，不能说明非零时间区间完整。

分别输出：两对象都有直接原始样本的最小值；允许有界对齐的断点最小值；分段线性圆心模型的区间最小值。后两项不是直接测得的连续轨迹。最近时刻不是首次接触时刻。物体由set_pose跳变/保持，偏移旋转亦可产生曲线，线性段只是一种明确模型。

数值域：坐标绝对值≤1e6m，半径0～1e6m，整数时间0～9e15ns。几何函数不以小速度阈值抹去非零相对运动；采用缩放相对位移与端点导数符号处理下溢。数值比较epsilon=max(1e-12m,64*ulp(当前坐标/半径尺度))，不等于物理精度。位置/半径/时间/段内插值的独立物理误差界未经证明，均保持null；误差感知物理结论unknown。nominal geometric_contact仅指所声明的圆/线性模型，边界带为null。

coverage按每个对象对的有效仿真时间长度计算。full仅指声明对象对和固定区间完整，不含全世界。缺口保留无效区间和原因；partial保留可信局部最小值/重叠，不能声称全请求范围最小。无数据的测量为null，不补0/Infinity。

collision始终null。正式回合contact_sensor_event未评价则null。复用probe的测试球/圆柱自检单独声明实体、几何、时间/高度及contact ID；2D包络相交不等于3D接触。真实接触深度与实际pose分别复算，不用命令pose填真值。

只读预检不创建目录。输出排他创建，原run之外，不允许覆盖、符号链接逃逸或输出侵入输入。成功/失败均另存分析，绝不回写旧manifest/result/bag/sidecar。CLI 0=完整有效模型评价（包括重叠），2=格式/身份非法，3=无可用输入，4=有效局部但不完整，1=意外内部失败。fixture必须显式开关，永不混入real列表。

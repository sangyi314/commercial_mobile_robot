# cmr_core 接口合同（P0-01 审查草稿）

日期：2026-10-20（Asia/Shanghai）。依据：当前 `src/cmr_core/include/cmr_core/core.hpp`（90 行）。本文描述现有 API；新增测试尚未验收，人工推导与审查待完成。库为 ROS-independent、header-only、C++17；本步不修改生产头文件、构建声明或导航功能。

## 类型、单位与错误语义

| 接口 | 输入与单位 | 输出与现有行为 |
|---|---|---|
| `Pose2{x,y,yaw}` | x/y：m；yaw：rad；右手平面，正 yaw 逆时针 | 普通数据结构，自身不校验；无 frame、时间戳、协方差或硬件反馈 |
| `Twist2{v,w}` | 车体前向 v：m/s；偏航 w：rad/s | 不包含横移；有限 v/w 不保证轮速可行 |
| `propagate(p,u,dt)` | 一个时间步内 v/w 恒定；dt：s，有限且 ≥0；位姿和控制各字段有限 | 返回解析常速圆弧终点；yaw 使用 remainder 归一到闭区间 [-π,π]，±π 均可能出现。非法输入抛 `std::invalid_argument`；计算输出非有限抛 `std::overflow_error` |
| `wheel_rates(u,radius,track)` | 轮径半径 r、左右轮距 b：m，均正且有限；u 有限 | `std::array<double,2>` 顺序为 `{left,right}`，单位 rad/s。非法模型或控制抛 `std::invalid_argument`。极端有限输入可能发生算术溢出，直接 API 当前没有单独的输出异常保证 |
| `wheel_speed_ok(u,radius,track,max_abs_rate)` | 同上；轮速绝对值上限为正有限 rad/s | 两轮均有限且各自绝对值 ≤上限时 true；非法输入、非有限结果或任一超限时 false。只检查，不限幅，不检查轮加速度 |
| `Command{value,seq,created,ttl}` | value 为 Twist2；seq 为 uint64；created 为 steady_clock 时间点；ttl 为 milliseconds | 本进程命令数据；无会话 ID、来源授权、ROS frame 或底盘状态 |
| `CommandGate(max_ttl)` | 正 milliseconds | ≤0 抛 `std::invalid_argument`；有状态、单写者；没有线程保护 |
| `accept(c,now)` | 同一进程 steady 时间域，合理且可表示的时间差 | 仅在所有条件满足时返回 true 并更新 last_seq；否则 false，不消耗序号 |

## 位姿传播与独立预期

连续模型为 `dx/dt=v*cos(yaw)`、`dy/dt=v*sin(yaw)`、`dyaw/dt=w`。对 w≠0，用半径 R=v/w 和圆心几何求端点：Δx=R(sin(yaw+w*dt)-sin(yaw))，Δy=R(cos(yaw)-cos(yaw+w*dt))。w=0 时为直线。当前实现用等价的中点角/sinc 写法避免近零相减损失；测试使用手算端点及独立几何界，不复制该实现作为唯一 oracle。

- 原点、v=1、w=1、dt=π/2：端点 (1,1,π/2)；单步 Euler 会给出 (π/2,0,π/2)，不能表达这个常速圆弧端点。
- w=-1：端点 (1,-1,-π/2)；v=-1、w=1：(-1,-1,π/2)。纯旋转不改变位置。
- dt=0：x/y 不变；yaw 仍归一。未归一 yaw 不要求逐位不变，应比较角度等价。跨 ±π 用角差比较，不能比较原始 yaw 差值。
- 模型没有电机加速、一阶响应、轮滑、制动距离、碰撞或执行延迟；积分正确不等于实车模型准确。

## 差速轮速

`left=(v-b*w/2)/r`，`right=(v+b*w/2)/r`。v=.5、w=2、r=.1、b=.4 得 `{1,9}` rad/s，因此上限 8 时拒绝；分别限制 v 和 w 不能替代两轮约束。精确可表示边界 v=1、w=0、r=.5、b=1 得 `{2,2}`，上限 2 接受，上限为 2 的前一可表示数时拒绝；负轮速同样按绝对值检查。

`wheel_rates` 在极端有限输入下可能返回 Inf：例如最大有限 v 除以 .5，或 track*w 的中间乘积溢出。这是当前局限，不把 Inf 固定为未来必须保留的成功结果。直接调用者应限定物理模型范围并检查输出。`wheel_speed_ok` 必须对此返回 false；本步不改变 API 为抛溢出异常。如需强化直接 API，应另行审查最小补丁与调用方。

## TTL、序号与职责边界

接受区间为 `[created,created+ttl)`，含创建时刻，不含到期时刻；ttl 必须 >0 且 ≤max_ttl，now 不早于 created。首条 seq=0 合法；已有接受记录后必须严格大于 last_seq。拒绝（含非法高序号）不更新 last_seq；最大 uint64 被接受后不能回绕至 0，当前无会话重置协议。

100ms TTL 的 99ms 接受，100ms、101ms 拒绝。分别使用新 gate，避免重复序号掩盖过期判断；拒绝后只修复时间或数值，同 seq 应可接受。测试时钟均为合成 steady_clock 时间点，不 sleep，不读取 ROS clock。

合同限于可表示的合理持续时间，不承诺任意 time_point::min/max 相减安全，也不将外部/ROS 时间戳直接转为 steady 时间。不同机器单调时钟原点不能互比；重启、跨主机和会话边界留给适配层。

`accept` 只在调用时检查，不存储有效命令、不定时回调、不输出零速度；没有新消息时不会在 TTL 到期自动停车。P1 的周期监督/输出撤销和 P2 的独立底盘 watchdog 尚未实现或验证。此合同不构成生产级安全性或实车放行结论。

对应案例与源码定位见 [测试设计](../design/core-test-design.md)；本阶段证据见 [审查报告](../verification/p0-01-gfGjKp8O-draft.md)。

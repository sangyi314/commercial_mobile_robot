# 核心测试设计（P0-01 审查草稿）

2026-10-20（Asia/Shanghai）：逐项核对当前源码后的增量版本。旧版本 56 行简版仅有五组基础检查，不能按旧文档声称已有全部边界。现保留原有检查，在原 `straight / arc / near_zero / wheels / gate` 内补缺；CMake 五组注册保持原样。以下是**草稿覆盖索引，新增案例尚未编译或运行，不是通过记录**。

接口依据见 [core_contract.md](core_contract.md)。本轮沿用真实步骤 2–3 编号 `p0-01-gfGjKp8O`；基准指纹、日志与审查停点见 [本步报告](verification/p0-01-gfGjKp8O-draft.md)。

## 案例对照

源码路径统一为 `src/cmr_core/test/core_test.cpp`；位置列是含 ID 的首个断言/标签行，可按 ID 搜索整段（共享 helper 与循环会展开多个检查）。所有行号对应本次草稿，运行日志将输出 `[check] ID ...`，CTest 通过案例的详细日志应查看 verbose 输出或 LastTest.log，不能仅看 5/5 汇总。

| ID | 分组 | 修改前覆盖 / 本次动作 | 独立预期与隔离方法 | 当前源码位置 |
|---|---|---|---|---|
| P01 | straight | 已有端点，补航向/范围 | (1,2,π/2)，v=.5、dt=2 → (1,3,π/2) | `core_test.cpp:55` |
| P02 | straight | 新增 | 倒车 v=-.5、dt=2：(1,2,0) → (0,2,0) | `core_test.cpp:57` |
| P03 | straight | 新增 | dt=0，x/y 精确不变；4π+.25 与 .25 角度等价且归一 | `core_test.cpp:59` |
| P04 | straight | 新增 | dt=-.1 精确按 invalid_argument 类型检查 | `core_test.cpp:63` |
| P05 | straight | 新增 3 输入 | dt=NaN/+Inf/-Inf，各抛 invalid_argument | `core_test.cpp:67` |
| P06 | straight | 新增 5×3 输入 | x/y/yaw/v/w 每字段分别为 NaN/+Inf/-Inf，其余合法，各抛 invalid_argument | `core_test.cpp:81` |
| P07 | straight | 新增 3 输入 | max*v 时间乘积、位置求和及角度乘积有限输入溢出，各抛 overflow_error | `core_test.cpp:87` |
| A01 | arc | 保留，补角差/范围 | 单位正四分之一圆：(1,1,π/2) | `core_test.cpp:96` |
| A02 | arc | 新增 | 负转向：(1,-1,-π/2) | `core_test.cpp:100` |
| A03 | arc | 新增 | 倒车圆弧：(-1,-1,π/2) | `core_test.cpp:102` |
| A04 | arc | 新增 | (2,-3,.25) 纯转两秒 → (2,-3,1.25) | `core_test.cpp:104` |
| A05 | arc | 新增 | 整圆回原点；正反方向跨 ±π 用角差，yaw 属 [-π,π] | `core_test.cpp:106` |
| A06 | arc | 新增 | 非零位姿 (.8,.6) 前进后以 (-.8,-.6) 和相同正 dt 返回；P04 仍拒绝负 dt | `core_test.cpp:114` |
| N01 | near_zero | 已有 +1e-12 宽松极限；补 0/负向与性质 | w=0、±1e-12，有限、近直线；非零转向符号正确，航向积分正确 | `core_test.cpp:118` |
| N02 | near_zero | 新增正负两侧 | dt=1，w=±2e-6*(1±1e-3)，小角几何界及连续性，不复制 sinc | `core_test.cpp:144` |
| W01 | wheels | 保留 | {1,9} 左右顺序；limit=8 拒绝；已有直行可行例保留 | `core_test.cpp:153` |
| W02 | wheels | 新增 | 原地旋转 v=0,w=2,r=.1,b=.4 → {-4,4} | `core_test.cpp:158` |
| W03 | wheels | 新增 | 倒车 v=-.5,w=0 → {-5,-5} | `core_test.cpp:160` |
| W04 | wheels | 新增正负边界 | v=±1,r=.5,b=1 得绝对值 2；limit=2 接受，nextafter(2,0) 拒绝 | `core_test.cpp:162` |
| W05 | wheels | 已有 radius=0 bool 检查；补齐 | r/b 分别 0、负、NaN/+Inf/-Inf；直接 API 抛 invalid_argument，checker=false | `core_test.cpp:156` |
| W06 | wheels | 新增 | limit 分别 0、负、NaN/+Inf/-Inf，其他输入合法，false | `core_test.cpp:180` |
| W07 | wheels | 新增 | v/w 分别 NaN/+Inf/-Inf：直接 API 抛 invalid_argument，checker=false；有限除法/中间乘积溢出 checker=false | `core_test.cpp:185` |
| G01 | gate | 新增 | max_ttl=0ms/-1ms 构造抛 invalid_argument | `core_test.cpp:212` |
| G02 | gate | 新增 | 新 gate 首条 seq=0 且 now=created 接受 | `core_test.cpp:215` |
| G03 | gate | 保留已有场景；补隔离与序号复用 | 99/100/101ms 各新 gate；100ms TTL 接受/拒绝/拒绝，到期拒绝后更新时间，同序号接受 | `core_test.cpp:200` |
| G04 | gate | 已有重复拒绝；补大小顺序 | 合法 seq=10 接受，10/9 拒绝，11 接受；所有时间和数值合法 | `core_test.cpp:201` |
| G05 | gate | 已有未来拒绝；补独立修复 | now<created 拒绝；只修复 created，同 seq 接受 | `core_test.cpp:204` |
| G06 | gate | 新增独立 3 场景 | ttl=0/-1/251ms，max=250ms；各拒绝后只修复 ttl，同 seq 接受 | `core_test.cpp:240` |
| G07 | gate | 新增独立边界 | ttl=max=250ms；创建时刻接受，到期新 gate 拒绝 | `core_test.cpp:248` |
| G08 | gate | 已有 v=NaN；补 2×3 独立输入 | v/w 分别 NaN/+Inf/-Inf；分别新 gate，恢复合法数值后同序号接受 | `core_test.cpp:207` |
| G09 | gate | 新增 | 先接受 seq=1，再拒绝非法 ttl 的 seq=100，合法 seq=2 仍接受 | `core_test.cpp:262` |
| G10 | gate | 新增 | 首条 uint64 最大值接受；同值和回绕到 0 拒绝，其他字段合法 | `core_test.cpp:267` |

## Oracle、误差与失败行为

- 大角端点来自单位圆、圆心/半径和手算端点；A06 是附加的可逆性质，不作为唯一参考实现。P03 明确验证零时间的位置不变、角度等价，而不是要求未归一 yaw 逐位不变。
- 位置默认绝对容差 `1e-10 m`（沿用已有端点容差），轮速用同一数值绝对容差 `1e-10 rad/s`；航向角差容差 `1e-12 rad`。精确轮速边界和零时间位置用零容差。`near` 显式要求比较量及容差 finite，且容差非负；位姿还检查 yaw 归一范围。
- 原点、yaw=0、v=dt=1 时，独立积分几何给出 `0≤1-x≤w²/6`、`0≤sign(w)*y≤abs(w)/2`；非零 w 的 y 符号必须正确。取绝对舍入余量 `16*double epsilon`（约 `3.55e-15`）。连续性界来自 `|dx/dw|≤|w|/3`、`|dy/dw|≤1/2`：比较分支两侧结果的变化不超过导数界乘输入差再加余量。这里单位路径使数值可直接比较，不复制被测 sinc 分支。
- `check` 不依赖 assert/NDEBUG，失败抛出并由 main 返回 1；成功逐条打印 ID。`expect_throws<Expected>` 只将指定异常类型算成功，其余标准异常、非标准异常或未抛异常均失败。运行时错误也会返回非零。
- 非有限集合逐项为 quiet_NaN、+Inf、-Inf；逐字段替换，其余参数合法。W07 不断言 wheel_rates 必须返回 Inf，只要求 wheel_speed_ok 不放行溢出。
- TTL/数值/序号用例分别隔离；G03/G05/G06/G08 还验证拒绝后只修复非法字段、同 seq 可接受。所有时间是 steady_clock 合成的可表示小时间差，不 sleep，不使用 ROS clock。

## 验收证据合同（待人工审查后执行）

1. 本阶段仅静态审阅与差异；新增源码的 hash 是草稿身份，不能标为已测试 hash。既有 before-ctest 日志只绑定修改前源码。
2. 审查确认后，按步骤 7→8→9→10 依次执行独立 Release、Debug ASan/UBSan、Jazzy/ament、纯 CMake 与 ament 两种安装消费者。沿用本轮编号创建唯一后缀目录，不删除旧结果，不加载根旧 install/setup.bash。
3. 保持五组时，`ctest -N` 必须发现五组，实际执行用 `--no-tests=error`，完整保存配置/编译/测试退出码、日志与 JUnit。启用 pipefail，不能让 tee 掩盖失败。不得开启 fast-math。
4. 新验收记录必须保存被测试核心头文件、测试、CMake 的 SHA256、Git HEAD/dirty、环境和真实构建参数；测试前后源码一致，之后变更必须重新验收。不用历史日志补齐新版本。
5. 消费者只链接安装导出目标 `cmr_core::cmr_core`，默认请求 C++11、编译期确认实际 C++17 并执行；不得通过源码 include 路径绕过导出错误。Jazzy/ament 仅包级构建测试，不启动 ROS 图。
6. 发现合同违例时保留最小失败用例、原始日志、影响范围和建议最小补丁，不改 oracle 或擅自扩大生产源码修改范围。目前未运行新增测试，不能下运行时违例结论。

保留历史边界：旧 command_policy 不恢复、不测试；不新建产品空包；ROS 消息集成、Nav2、Gazebo、录包、硬件运动、主动过期停车、多线程与跨主机时间均未执行或未覆盖。P0-01 总验收与人工理解仍待完成。

# P0-01 增量测试与证据合同：审查草稿

日期：2026-10-20（Asia/Shanghai）。**停点：步骤 6，待负责人审查；P0-01 未完成验收。** 本报告仅列后续待验收项，不是第 12 步完成报告。

## 现场与基准复核

- 实际工作路径 / Git 根：`/home/robot/commercial_mobile_robot`，Ubuntu Linux aarch64。
- HEAD：`062eed30e1d59adf586d0743f1c825110180f0a7`；修改前 dirty：仅未跟踪 `evidence/env/p0-01-gfGjKp8O/`。当前工作树含本次测试/文档草稿与新证据，未 commit/push。
- 既有步骤 2–3 唯一编号：`p0-01-gfGjKp8O`。本次核对后沿用，不另建仓库或混用历史 section4 编号。
- 证据目录：[evidence/env/p0-01-gfGjKp8O](../../evidence/env/p0-01-gfGjKp8O/)；修改前构建：`build/p0-01-gfGjKp8O-before`。
- 实际复核：备份中全部 7 个普通文件字节与修改前文件一致；8 个 selected 和 69 个 tracked SHA256 均一致；HEAD 一致；基准 CMakeCache 指向当前核心目录，Release、BUILD_TESTING=ON、禁用 ament 查找、GNU 13.3.0，flags 为 -O3 -DNDEBUG，无 fast-math。
- 现有基准为 5/5、JUnit tests=5/failures=0；本次仅读取和交叉核对，没有重新执行。该结果只属于修改前简版，绝不绑定新增测试 hash。新增运行项均未执行。

基准原始证据：[备份](../../evidence/env/p0-01-gfGjKp8O/core-and-docs-before.tgz)、[原重点指纹](../../evidence/env/p0-01-gfGjKp8O/selected-sha256-before.json)、[原 tracked 指纹](../../evidence/env/p0-01-gfGjKp8O/tracked-sha256-before.json)、[配置](../../evidence/env/p0-01-gfGjKp8O/before-configure.log)、[构建](../../evidence/env/p0-01-gfGjKp8O/before-build.log)、[发现 5 组](../../evidence/env/p0-01-gfGjKp8O/before-test-list.log)、[CTest 5/5](../../evidence/env/p0-01-gfGjKp8O/before-ctest.log)、[JUnit](../../evidence/env/p0-01-gfGjKp8O/before-ctest.xml)。

本次附加的证据：[基准复核记录](../../evidence/env/p0-01-gfGjKp8O/draft-baseline-review.json)、[草稿前补充备份](../../evidence/env/p0-01-gfGjKp8O/draft-before.tgz)、[静态核对记录](../../evidence/env/p0-01-gfGjKp8O/draft-static-review.json)、[草稿指纹](../../evidence/env/p0-01-gfGjKp8O/draft-sha256.json)、[完整审查差异](../../evidence/env/p0-01-gfGjKp8O/draft-changes.patch)、[当前 Git 状态](../../evidence/env/p0-01-gfGjKp8O/draft-git-status.txt)。原基准文件不覆盖。

## 当前源码与历史描述的更正

修改前 `core_test.cpp` 56 行：只有朝北端点、正四分之一圆、+1e-12 极限、组合轮速/部分 bool 检查和少量 gate 场景。`docs/core_test_design.md` 与旧完成报告所称扩展边界不能从该源码得到支持。此次不推测成因，不抹去历史报告；完成报告只追加本日期复核附记。

## 修改文件与案例证据

| 文件 | 实际改动 |
|---|---|
| `src/cmr_core/test/core_test.cpp` | 原五组内增量保留旧检查并补全部 32 个清单 ID；finite 比较、角差、指定异常 helper、逐检查日志；Release 不依赖 assert |
| `docs/core_contract.md` | 新增现有 API 单位、异常、左右轮顺序、TTL/序号与现有局限草稿 |
| `docs/core_test_design.md` | 按 ID 对照原覆盖/缺口、独立 oracle、源码锚点和待执行证据合同 |
| `docs/section4_completion_report.md` | 仅追加带日期复核更正与本报告链接 |
| `docs/ownership.md` | 仅追加实际 AI 工作与尚未人工审查状态 |
| `AI_USAGE.md` | 仅追加问题、AI 范围、实际基准复核、未执行项和局限 |
| `docs/verification/p0-01-gfGjKp8O-draft.md` | 本草稿报告 |

| 类别 | 本阶段证据 | 后续执行证据 |
|---|---|---|
| P01–P07 / straight | [测试设计 ID 索引](../design/core-test-design.md)，测试源代码与完整差异 | 待新增 Release/sanitizer 逐 ID 日志 |
| A01–A06 / arc | 手算正负四分之一圆、纯转、跨 π、正 dt 逆运动 | 待运行 |
| N01–N02 / near_zero | 独立几何误差界、正负转向、分支两侧连续性 | 待运行 |
| W01–W07 / wheels | 手算轮速、精确边界、非法参数、溢出 fail-closed | 待运行 |
| G01–G10 / gate | 独立 gate、逐字段拒绝与同序号修复、最大序号局限 | 待运行 |

生产头文件、CMakeLists、package.xml、三份 nav2 YAML、tools/make_baselines.py 的指纹须与原 selected-sha256 对照；本步只读，未修改。五组 CMake 注册保持原样，没有创建 ROS 节点/消息、恢复旧 command_policy 或开发导航功能。

## 草稿身份（尚未被新测试执行）

```text
core.hpp SHA256: 637194346f676c125c14d1359770496373cc5c4653cc91324d10d7cb049f30f3
core_test.cpp 草稿 SHA256: 411bd138852d83a84281d5944dcb885ede36995257dc802dbd8202ebcdcaaf9b
CMakeLists.txt SHA256: 83a587a5abc4d79b3f22d76979f1991e0f57d31bd8862d2a0480314d4c8f8f1e
```

这些 hash 标识审查版本；后续每次验收须另记录实际编译执行的源码 hash、参数、日志、退出码和测试前后是否变化。不能给草稿 hash 配旧 5/5 日志。

## 验收状态与待执行顺序

| 层级 / 项目 | 本阶段状态 | 审查后要求 |
|---|---|---|
| 静态检查：源码/文档/范围 | AI 已核对；详见静态记录，负责人未审查 | 人工核对 oracle、异常与实际差异，不能代替运行 |
| 修改前单元基准 | 既有记录 5/5，指纹已复核；本次未重跑 | 不用于新增案例通过结论 |
| 步骤 7：新增独立 Release | 未执行，待审查 | 五组发现与实际运行、--no-tests=error、详细案例日志、JUnit、hash |
| 步骤 8：Debug ASan/UBSan | 未执行，待步骤 7 | 保存配置/编译/运行及 sanitizer 原始日志 |
| 步骤 9：Jazzy/ament 包级集成 | 未执行，待步骤 8 | 仅 cmr_core，独立编号 build/install/log，实际测试结果非零条目 |
| 步骤 10：独立安装消费者 | 未执行，待步骤 9 | 使用新安装目标、确认 C++17 传递并运行 |
| 步骤 10：ament 安装消费者 | 未执行 | 同上，绑定新 ament 安装与源码 |
| ROS 图与消息集成 | 未执行，本步不要求 | 后续阶段 |
| Nav2 / Gazebo / 录包 / 真实底盘 | 均未执行 | 后续阶段，不从核心测试推断 |

本阶段未创建后续 release/sanitizer/colcon/consumer 构建或安装目录。后续继续沿用 `p0-01-gfGjKp8O` 的唯一后缀，保留旧 before 产物，按步骤顺序实际执行。

## 局限、人工审查与下一停点

wheel_rates 极端有限输入可能产生 Inf，checker 必须拒绝；本步未更改直接 API。时间差仅覆盖合理可表示范围，不承诺任意 min/max 差值；序号到最大值无回绕会话恢复；门控单写者、无主动 TTL 停车、无鉴权/底盘反馈。有限用例不覆盖无限浮点域，也不支持生产安全性结论。

目前没有新增运行失败日志，因为新增测试未执行。若审查后发现实现违例，须先保存最小失败用例、原始日志、影响范围与建议最小补丁；不改 oracle 迁就实现，不擅改受保护生产文件。

负责人需审查：①单位圆 (1,1,π/2) 与 Euler 反例；②轮速 {1,9}、上限 8 拒绝；③100ms TTL 的 99/100ms 独立判断与拒绝不耗序号；④小角界/舍入容差、异常类型及 dt=0 的角度等价；⑤无自动停车和真实硬件验证的边界。能解释/能推导/能修改/能排故均待本人确认，没有代打勾。

**请先审查完整差异和案例预期。未经本轮人工审查，不执行步骤 7–10，不填写第 12 步通过结论，不进入 P0-02 完成状态。**

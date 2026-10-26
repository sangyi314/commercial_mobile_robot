# CMR 文档导航

本目录区分当前操作说明、接口合同、设计依据与历史实测记录。源码和冻结输入决定程序行为；指南不替代预检，验收报告只对其声明的版本、Run 和范围有效。

当前开发主线已转向 P1。P0-09仅完成README文档收尾，其余复现、验收与发布工作暂缓；没有v0.1正式发布。P1-01配送任务状态机是下一项开发工作，本次未开发P1代码。

## 从这里开始

| 任务 | 权威入口 |
|---|---|
| 选择场景、配对输入、判断当前与历史版本 | [场景目录](scenario_catalog.md) |
| 构建核心、运行分层测试、环境边界 | [构建与测试](guides/build-and-tests.md) |
| 单回合预检、执行与收尾 | [Runner指南](guides/runner.md) |
| GUI观察与人工确认 | [Gazebo观察](guides/gazebo-observation.md) |
| 动态输入准备与版本迁移 | [动态场景](guides/dynamic-scenes.md) |
| 指定Run生成Spec、预检与几何评价 | [几何评价](guides/geometry-evaluation.md) |
| 批次预检、派发与离线汇总 | [批次实验](guides/batch-experiments.md) |
| 实验输入职责、历史精简与恢复 | [experiments说明](../experiments/README.md) |
| Python入口、worker及源码归档布局 | [tools说明](../tools/README.md) |
| 测试组织、历史路径与安全执行范围 | [tests说明](../tests/README.md) |

长路线任务和输入配对只在场景目录维护；各指南链接引用，不分别维护坐标表。新工具布局已经过离线验证，真实ROS/GUI/Runner集成不由文档迁移补签。旧动态输入的冻结源码路径已经迁走，新布局模板仍为draft；旧cohort也不能绕过源码漂移检查继续派发。

## 系统设计与合同

| 分类 | 文档 | 状态/适用范围 |
|---|---|---|
| ADR | [0001 项目范围](adr/0001-scope.md) | 研究与系统范围；不代表风险算法已实现 |
| 核心接口 | [core-contract](contracts/core-contract.md) | P0-01原审查稿；保留当时状态与API边界 |
| Runner | [runner-contract](contracts/runner-contract.md) | P0-05生命周期、期限、IPC、清理与退出语义 |
| 批次 | [p0-08-contract](contracts/p0-08-contract.md) | 冻结合同及已有版本修订，不改历史数字 |
| 数据 | [Scenario/manifest/result说明](../experiments/schema.md) | Schema仍在experiments；本次只修正文档链接 |
| 评价 | [评价合同](../experiments/evaluation_contract.md) | Concrete Spec与Catalog、覆盖和unknown语义 |
| 测试设计 | [核心](design/core-test-design.md)、[记录校验](design/run-schema-test-design.md) | 历史设计稿与案例ID，不能按“预期”宣称已通过 |
| 动态设计 | [P0-06动态场景](design/p0-06-dynamic-scenes.md) | 分阶段设计快照及后续工程补充 |
| 数学模型 | [几何距离](design/geometry-metrics.md) | 圆包络与线性段最近点，不等于物理碰撞 |

## P0阶段与证据

以下是索引，不重新验收。报告中的“未执行”“失败”“待本人确认”保留原义。

| 阶段 | 当前指南/合同 | 已有记录及范围 |
|---|---|---|
| P0-01 | 构建与测试、核心合同 | [早期核心交付及更正](verification/section4-core-20261020.md)、[增量审查草稿](verification/p0-01-gfGjKp8O-draft.md)；后续日志入口见根README，草稿不升级为通过 |
| P0-02 | 数据说明、记录校验设计 | [原草稿](verification/p0-02-RaLo7r83.md)、[后续日志交接](verification/p0-02-RaLo7r83-handoff-20261022.md) |
| P0-03 | Runner指南；[历史数据流](history/p0-03-dataflow.md) | [真实失败回合](verification/p0-03-nq1hmjaf.md)，REJECTED及cleanup失败不被后续成功覆盖 |
| P0-04 | 场景目录 | [双基线权威报告](verification/p0-04-p0-04-pair-q2MXrdh3.md)，限定官方短静态任务 |
| P0-05 | Runner指南/合同 | [单回合报告](verification/p0-05-bmzezluh.md) |
| P0-06 | 动态场景指南/设计 | [第1～6步](verification/p0-06-a3179_6v-step1-6.md)、[后续真实记录](verification/p0-06-kwdqt70u-live.md)、[长路线联调](verification/long-route-v1-20261029.md) |
| P0-07 | 几何评价指南/合同 | [原四轮评价](verification/p0-07-4v9sd2n4.md)、长路线联调；只覆盖声明对象对和区间 |
| P0-08 | 批次指南/合同 | [长路线报告](verification/p0-08-long-20261029-3cff94.md)：正常21次尝试/20次完整通过，故障6次通过；个人放行仍待确认 |
| P0-09 | [根README阶段切换](../README.md#阶段切换与-p1-路线) | 精简文档收尾，不宣称完整验收或正式发布 |

原始Run、日志、bag、manifest和结果仍在evidence；离线分析与原result分开，不回填原件。collision=null为未知，不能从Action成功、Schema合法或GUI窗口存在推导安全通过。

## 故障排查

- [F001：controller停用后的清理竞态](failures/F001.md)。
- [F002：Action成功但MCAP首个到达样本超差](failures/F002.md)。
- [GUI空白场景](guides/gazebo-observation.md#空白场景排查)：检查同partition真实场景，再检查渲染；历史修复快照单独保留。
- [历史快照索引](history/README.md)：解释旧文档“本次”“当前”的时间范围。
- [本次文档迁移报告](verification/docs-layout-20261030.md)：完整旧新路径、去重、链接与保护核对。

## 新文档的规则

1. 按职责选目录，不按P1/P2新建空目录。指南写可执行步骤；合同写稳定接口/约束；设计写依据；verification写实际证据；failures写失败与修复；history保存过时快照。
2. 新文件采用小写kebab-case。指南以任务命名；阶段专属合同可用p1-01前缀；报告使用阶段/主题、日期及唯一Run或批次标识。既有verification/F001等冻结名称不追改。
3. 文首说明范围、状态和版本来源。区分已实现、离线验证、真实集成验证、待人工确认；不把计划写成结果。
4. 新增P1/P2页面时更新本索引；场景入口仍只在scenario_catalog维护，工具参数以CLI/源码为准，测试命令以tests/README为准。使用相对Markdown链接，移动后检查目标和锚点。
5. 新报告、新Run、新分析使用独立标识；不覆盖旧失败，不把历史报告改成指南。不修改Manifest、资源哈希或合同语义来修复文档。
6. ADR沿用四位递增编号，至少写背景、选择、理由与影响。只有实际决策时新增，不预建空白模板。

P1-01优先做纯逻辑配送状态机，再做隔离模拟Nav2 Action Server协议测试，之后才接真实导航。计划本身不是运行或安全证据。

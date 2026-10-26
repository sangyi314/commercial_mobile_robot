# 文档体系迁移记录（2026-10-30）

开始日期2026-10-30，最终核验2026-10-31（Asia/Shanghai）；文件名保留开始日期。

工作区：/home/robot/commercial_mobile_robot；分支main；HEAD 76c822ebbe16e9d1c46a7ac50dfefc13539d8aa4。基于实际未提交工作区增量迁移；不以Mac/GitHub覆盖本地。没有commit/push。

## 审计与分类依据

读取了原35份docs文件，分析正文、阶段状态、Markdown链接及源码/实验/历史审计引用。P0-08长路线报告真实存在且原为未跟踪文件，未遗漏。baseline_runtime与P0-04报告字节相同（SHA256 988de8e049be59cfec8ac5eb0cde3ecc3c7a264c3b5d2d0538e117022cf1d9c2），保留verification权威报告，删除唯一完全重复副本。

混合指南和场景目录保留历史快照，当前6份指南按实际CLI和工具布局重写。长路线任务/输入只在scenario_catalog维护；报告保留各自冻结任务和真实数字。charter/environment/ownership/dataflow/navigation_smoke明确归历史，不作为最新机器状态或全局导航结论。

## 完整迁移表

下列覆盖全部原文档。history中的旧指南另有当前guides入口；scenario_catalog原混合版归历史，根同名文件是新的权威目录。

| 原docs路径 | 新docs路径 | 分类理由 |
|---|---|---|
| adr/0001-scope.md | adr/0001-scope.md | 保留架构范围决策 |
| baseline_runtime.md | verification/p0-04-p0-04-pair-q2MXrdh3.md | 完全重复，删除副本，权威报告原样保留 |
| charter.md | history/p0-01-charter.md | 早期P0-01范围快照，并非当前阶段进度 |
| environment.md | history/p0-01-environment.md | 历史机器/磁盘/工具环境，不冒充当前测量 |
| ownership.md | history/p0-01-ownership.md | 人工确认与贡献的历史状态 |
| dataflow.md | history/p0-03-dataflow.md | 特定失败回合的数据流与证据边界 |
| navigation_smoke.md | history/p0-03-navigation-smoke.md | 保留失败回合的独立摘要 |
| core_contract.md | contracts/core-contract.md | 原P0-01接口审查草稿，状态不升级 |
| runner_contract.md | contracts/runner-contract.md | 单回合生命周期合同 |
| p0_08_contract.md | contracts/p0-08-contract.md | 批次与故障冻结合同，结论不改 |
| core_test_design.md | design/core-test-design.md | 保留原案例和未执行状态 |
| run_schema_test_design.md | design/run-schema-test-design.md | Schema测试历史设计与案例ID |
| geometry_metrics_design.md | design/geometry-metrics.md | 几何模型及数学依据 |
| p0_06_design.md | design/p0-06-dynamic-scenes.md | 动态场景设计及分阶段历史说明 |
| section4_completion_report.md | verification/section4-core-20261020.md | 早期核心交付及后续更正 |
| goal_migration_long_route_v1.md | verification/long-route-v1-20261029.md | 真实长路线联调报告 |
| goal_migration_long_route_v1_files.md | verification/long-route-v1-files-20261029.md | 上述报告的文件/保护附录 |
| runner_usage.md | history/p0-05-runner-guide-20261030.md | 混合版原文归档；当前流程另见guides/runner.md |
| runner_gui_usage.md | history/gazebo-observation-guide-20261030.md | 旧修复与测试快照；当前流程另见guides/gazebo-observation.md |
| p0_06_usage.md | history/p0-06-guide-20261030.md | 旧准备/实测命令快照；当前流程另见guides/dynamic-scenes.md |
| p0_07_usage.md | history/p0-07-guide-20261030.md | 旧评价命令和开发记录；当前流程另见guides/geometry-evaluation.md |
| p0_08_usage.md | history/p0-08-guide-20261030.md | 旧cohort与当前交接混合快照；当前流程另见guides/batch-experiments.md |
| scenario_catalog.md | history/scenario-catalog-20261030.md | 旧混合目录快照；根同名目录重写为当前输入唯一索引 |
| failures/F001.md | failures/F001.md | 保留现有真实验收或故障记录，不重写原结论 |
| failures/F002.md | failures/F002.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-01-gfGjKp8O-draft.md | verification/p0-01-gfGjKp8O-draft.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-02-RaLo7r83-handoff-20261022.md | verification/p0-02-RaLo7r83-handoff-20261022.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-02-RaLo7r83.md | verification/p0-02-RaLo7r83.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-03-nq1hmjaf.md | verification/p0-03-nq1hmjaf.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-04-p0-04-pair-q2MXrdh3.md | verification/p0-04-p0-04-pair-q2MXrdh3.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-05-bmzezluh.md | verification/p0-05-bmzezluh.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-06-a3179_6v-step1-6.md | verification/p0-06-a3179_6v-step1-6.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-06-kwdqt70u-live.md | verification/p0-06-kwdqt70u-live.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-07-4v9sd2n4.md | verification/p0-07-4v9sd2n4.md | 保留现有真实验收或故障记录，不重写原结论 |
| verification/p0-08-long-20261029-3cff94.md | verification/p0-08-long-20261029-3cff94.md | 保留现有真实验收或故障记录，不重写原结论 |

## 重构前目录

~~~text
docs/adr
docs/adr/0001-scope.md
docs/baseline_runtime.md
docs/charter.md
docs/core_contract.md
docs/core_test_design.md
docs/dataflow.md
docs/derivations
docs/environment.md
docs/failures
docs/failures/F001.md
docs/failures/F002.md
docs/geometry_metrics_design.md
docs/goal_migration_long_route_v1.md
docs/goal_migration_long_route_v1_files.md
docs/navigation_smoke.md
docs/ownership.md
docs/p0_06_design.md
docs/p0_06_usage.md
docs/p0_07_usage.md
docs/p0_08_contract.md
docs/p0_08_usage.md
docs/papers
docs/run_schema_test_design.md
docs/runner_contract.md
docs/runner_gui_usage.md
docs/runner_usage.md
docs/scenario_catalog.md
docs/section4_completion_report.md
docs/verification
docs/verification/p0-01-gfGjKp8O-draft.md
docs/verification/p0-02-RaLo7r83-handoff-20261022.md
docs/verification/p0-02-RaLo7r83.md
docs/verification/p0-03-nq1hmjaf.md
docs/verification/p0-04-p0-04-pair-q2MXrdh3.md
docs/verification/p0-05-bmzezluh.md
docs/verification/p0-06-a3179_6v-step1-6.md
docs/verification/p0-06-kwdqt70u-live.md
docs/verification/p0-07-4v9sd2n4.md
docs/verification/p0-08-long-20261029-3cff94.md
~~~

## 重构后目录

~~~text
docs/
├── adr/
│   └── 0001-scope.md
├── contracts/
│   ├── core-contract.md
│   ├── p0-08-contract.md
│   └── runner-contract.md
├── derivations/
├── design/
│   ├── core-test-design.md
│   ├── geometry-metrics.md
│   ├── p0-06-dynamic-scenes.md
│   └── run-schema-test-design.md
├── failures/
│   ├── F001.md
│   └── F002.md
├── guides/
│   ├── batch-experiments.md
│   ├── build-and-tests.md
│   ├── dynamic-scenes.md
│   ├── gazebo-observation.md
│   ├── geometry-evaluation.md
│   └── runner.md
├── history/
│   ├── README.md
│   ├── gazebo-observation-guide-20261030.md
│   ├── p0-01-charter.md
│   ├── p0-01-environment.md
│   ├── p0-01-ownership.md
│   ├── p0-03-dataflow.md
│   ├── p0-03-navigation-smoke.md
│   ├── p0-05-runner-guide-20261030.md
│   ├── p0-06-guide-20261030.md
│   ├── p0-07-guide-20261030.md
│   ├── p0-08-guide-20261030.md
│   └── scenario-catalog-20261030.md
├── verification/
│   ├── docs-layout-20261030.md
│   ├── long-route-v1-20261029.md
│   ├── long-route-v1-files-20261029.md
│   ├── p0-01-gfGjKp8O-draft.md
│   ├── p0-02-RaLo7r83-handoff-20261022.md
│   ├── p0-02-RaLo7r83.md
│   ├── p0-03-nq1hmjaf.md
│   ├── p0-04-p0-04-pair-q2MXrdh3.md
│   ├── p0-05-bmzezluh.md
│   ├── p0-06-a3179_6v-step1-6.md
│   ├── p0-06-kwdqt70u-live.md
│   ├── p0-07-4v9sd2n4.md
│   ├── p0-08-long-20261029-3cff94.md
│   └── section4-core-20261020.md
├── README.md
└── scenario_catalog.md
~~~

## 去重与当前/历史边界

删除baseline_runtime.md的重复全文1份；未删除任何独立验收报告或失败案例。6份旧混合页面进入history后不再维护为当前指令；新指南链接场景唯一入口。报告中保留历史任务和坐标不算并行维护配置。

历史记录只做必要Markdown链接目标修正；原文字、数字、执行命令、通过/失败/未知结论保持。verification已有阶段文件名称和failures/F001/F002保持；新增3份根历史交付移入verification。旧审计中的路径与哈希保持原文，不以新hash回写。

## 冻结依赖与外部引用

未发现运行时Scenario/Profile或源码把待迁docs作为path+sha资源。历史evidence里有43个含旧文档路径的文件，主要是源指纹、保护/变更清单和归档发布脚本；它们不是当前启动锁，不修改、不重新执行。没有需要保留旧路径正文或新增软链接的活动调用方。

docs外仅改根README及两份文档的路径引用：experiments/schema.md的一处Markdown目标；tests/README.md的一处当前指南及历史路径说明。tools/tests/src/experiments/evidence中的Python、Bash、C++、YAML、Schema、原始记录均不改。README导航压缩重复命令，实际操作由guides维护。

## 验证结果

- 迁移前后安全离线pytest均为468 passed、3 failed、5 skipped、31 deselected；476项逐项状态完全一致，零新增回归。3项仍是test_runner_gui的dynamic旧冻结路径预检，不删除、不降断言、不增skip。
- 原114个本地链接/锚点有效；迁移后对docs、所有根Markdown及tools/tests/src/experiments Markdown扫描236个本地链接/锚点，缺失0、锚点错误0。1个外部Markdown链接未联网验证。
- 原34份非重复历史文档的文字、命令、数值和结论逐份核对一致，仅必要Markdown链接目标变化；其中17份发生链接修改。阶段报告与F001/F002未合并。
- 34141个范围外既有文件逐文件SHA256不变；无意外丢失。tools代码/源码锁不变，根8入口及既有目录保持；两个例外仅是tests/README与experiments/schema文档路径引用，根README另行更新。
- git diff --check与暂存区检查通过；HEAD与原分支不变。21份文档用git mv迁移，删除1份完全重复正文，新增10份当前导航/指南/索引/报告，共44份Markdown。
- 原已未跟踪的README、F002、P0-08报告、工具和测试结构继续保持原工作，不将其误记为本次新功能。

审计原件与实际日志长期备份位于/home/robot/cmr_backups/docs-layout-mvuje2tv：git-status-before.txt、before-sha256.json、docs-before.tgz、migration-plan.json、baseline.log/xml、after.log/xml、links-before.json、links-all-current.json、historical-link-rewrites.json、protection-final.json、test-comparison.json、task-docs.diff。原/tmp工作副本另保留；仓库外专用备份保存原件、完整任务差异与日志，本次没有写入受保护evidence。

原有docs/papers为空且没有活动引用，本次仅移除这个空目录，没有删除其中任何文件。

## 本次差异摘要

- docs：21项路径迁移、1项重复删除、10份新增页面；保留的历史正文只改链接目标。
- 根README：加入统一导航，修正移动路径；把重复运行命令移到当前guides，删除旧动态输入“当前可执行”的误导。
- experiments/schema.md：仅把测试设计链接更新到docs/design/run-schema-test-design.md。
- tests/README.md：仅更新当前几何指南和已迁文档的路径说明。
- tools、tests源码、src、实验输入/Schema及evidence均没有本任务内容变更；原工作区更改不撤销。


## 遗留与后续规则

外网链接本次不联网验证；evidence是本地资源且默认不入Git，克隆仓库后对应原始证据链接不保证可用。真实ROS/Gazebo/GUI/Runner、宿主负载、P1代码、正式评价与发布均未执行。新动态模板仍draft；旧cohort漂移门不绕过，P0-09仍为精简文档收尾。

未来P1/P2按职责归档，小写kebab-case，以任务命名指南，以阶段/日期/唯一标识命名报告；新增页面更新docs/README。原始输入与证据不可变、失败不覆盖、unknown不补成功。ADR沿用四位编号。不要为未开始阶段预建空目录。

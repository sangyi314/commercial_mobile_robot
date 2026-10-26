# CMR 实验输入

本目录保存当前Runner、动态准备、几何评价和批次调度所需的输入与合同。场景状态和任务坐标统一见[场景目录](../docs/scenario_catalog.md)；操作步骤见[文档导航](../docs/README.md)。文件名、draft/ready字段和Schema合法不能单独证明实验可运行或已经验收。

## 当前入口与边界

| 用途 | Scenario / Matrix | Profile / 配套文件 | 当前核验边界 |
|---|---|---|---|
| S0静态长路线，MPPI/DWB | [static](scenarios/p0_long_route_static_v1.yaml) | [static profile](profiles/p0_long_route_static_v1.yaml) | 清理前后--check成功；没有重跑导航 |
| 静态GUI观察 | [observe](scenarios/p0_long_route_observe_v1.yaml) | [observe profile](profiles/p0_long_route_observe_v1.yaml) | --gazebo-gui --wait-for-operator --check成功；未启动GUI |
| S1历史正式输入 | [crossing](scenarios/crossing.yaml) | [crossing profile](profiles/p0_06_crossing.yaml) | 原字节保留；旧源码路径已迁走，当前预检仍拒绝 |
| S3历史正式输入 | [occlusion](scenarios/observed_occlusion.yaml) | [occlusion profile](profiles/p0_06_observed_occlusion.yaml) | 同上，不能移植旧ready/probe声明 |
| S1新布局准备模板 | [crossing v2](tool_layout_v2/scenarios/crossing.yaml) | [profile v2](tool_layout_v2/profiles/p0_06_crossing.yaml) | draft、review/probe=null，未真实探测 |
| S3新布局准备模板 | [occlusion v2](tool_layout_v2/scenarios/observed_occlusion.yaml) | [profile v2](tool_layout_v2/profiles/p0_06_observed_occlusion.yaml) | 同上 |
| 正常批次 | [normal matrix](p0_smoke_matrix.yaml) | 各slot引用的p0_08_* Profile、[静态Scenario](profiles/p0_08_static.yaml)、[World](profiles/p0_08_world.sdf) | 新批次20槽位静态预检成功 |
| 故障批次 | [fault matrix](p0_fault_matrix.yaml) | 三种故障各MPPI/DWB，各自Profile | 独立6槽位预检成功 |
| P0-07 | [Concrete Specs与Catalog](evaluations/) | 明确指定原Run；使用归档模型、实体与源数据 | 8份Concrete Spec的--check均成功，无正式新评价 |

旧S1/S3入口的拒绝、旧批次源码漂移和原生二进制身份差异在清理前已存在，本次保留诊断。动态新模板须经原准备/真实probe/正式记录证据门，不能因清理而改成ready。历史完整Run合同有效也不等于导航或碰撞通过。

## 文件职责

- goals/：当前任务冻结文件及Goal请求；不修改目标或以新文件冒充旧Run输入。
- motions/：当前长路线轨迹、测试/通用probe仍使用的crossing.yaml，以及仍被当前依赖锁引用的crossing_r10.yaml。
- scenarios/：当前入口和合同测试/基Profile需要的旧静态Scenario。
- profiles/：静态/GUI/动态Profile、P0-08 Profile及派生World。
- schemas/：Scenario、manifest、result、动态报告和几何输入/输出Schema，均原样保留。
- evaluations/：8份具体Run Spec及2份索引。旧四轮仍由Catalog引用且可离线评价，不能删旧Spec或把Catalog传给--spec。
- tool_layout_v2/：已建立的新工具布局独立依赖版本，模板仍为draft；不重写历史锁。
- runner_profile.yaml：旧静态Runner、v2通用probe和回归测试的基Profile；不能被长路线Profile简单替换。
- p0_06_geometry-r02.json：当前场景及准备锁仍需的校准几何。旧未实测candidate几何已保留至外部备份。
- p0_06_native_dependencies.json：原生依赖身份。内含两个二进制的既有SHA漂移，保留并单独记录，不“修正”历史hash。
- recording_qos.yaml与recording_qos_p0_06.yaml：字节相同但两个路径都被代码/冻结锁使用，因此都KEEP；v2两个相同依赖清单同理。
- schema.md与evaluation_contract.md：数据与评价合同；本次不改字段、退出语义和算法。

## 本次精简

2026-10-31审计原89份文件，逐文件结果：KEEP 60、ARCHIVE 29、DELETE 0、REVIEW 0。29份ARCHIVE输入已从工作树移除，保存在单个仓库外、经恢复验证的原始输入压缩备份中，没有新增仓库内archive/。

移出的内容包括r06～r10及通用旧Probe Scenario/Profile、r06～r09 Motion与r06目标、旧短路线S3 Motion、旧根依赖清单、未实测几何candidate和3份manifests环境快照。r10 Motion仍KEEP，不随旧r10 Probe整体删除。环境快照原受Git忽略，不能只依赖Git恢复。

没有可证明能无条件丢弃的DELETE文件：两组完全重复内容都有冻结路径使用。ARCHIVE表示保留历史价值、移出当前入口，不等于错误，也不等于从未运行。四类完整清单、理由、源SHA、引用、前后树与验证见[清理报告](../docs/verification/experiments-cleanup-20261031.md)。

## 恢复历史输入

仓库外专用备份：/home/robot/cmr_backups/experiments-cleanup-20261031-mvvpkims

其中experiments-before.tgz保存精简前完整89份原输入；inventory-before.json记录逐文件SHA；classification-before.json及removed-files.json记录分类与移除理由。原evidence中的manifest/result/bag/准备证据不改。

独立恢复到全新临时目录，不覆盖当前工作区：

~~~bash
CMR_BACKUP="/home/robot/cmr_backups/experiments-cleanup-20261031-mvvpkims"
cd "$CMR_BACKUP"
sha256sum -c archive-sha256.txt
CMR_RESTORE="$(mktemp -d /tmp/cmr-experiments-restore-XXXXXXXX)"
tar --keep-old-files -xzf experiments-before.tgz -C "$CMR_RESTORE"
printf '%s\n' "$CMR_RESTORE/experiments"
~~~

备份已实际独立恢复并核对89份原文件SHA。要恢复旧路径时，按清单从临时恢复树逐文件拷回缺失位置，不覆盖已有文件。输入备份不承诺旧ROS/系统库、已变更二进制或旧源码环境全部可复现；须同时读取对应历史Run快照、Git版本和报告。旧Spec仍使用原Run，不以恢复文件替换其归档输入。

## 批次与验证注意

batch_core.source_inventory递归冻结整个experiments目录。移除历史文件和新增本README会改变新cohort的输入清单；漂移检查保持启用。现有已完成批次和结果可读，不因此改Manifest。未完成旧批次若要续跑，必须恢复它的完整源码/输入/环境身份，或按原合同新建cohort，不能借清理绕过检查。

本次仅静态和离线验证，不启动ROS/Gazebo、发初值/Goal、运行宿主负载、重新导航或正式几何评价。工具与测试目录结构不变。

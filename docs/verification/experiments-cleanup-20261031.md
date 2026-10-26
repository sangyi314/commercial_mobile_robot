# experiments输入精简（2026-10-31）

实际工作区/home/robot/commercial_mobile_robot；main；HEAD 76c822ebbe16e9d1c46a7ac50dfefc13539d8aa4。保留所有已有暂存/未暂存及未跟踪工作；未commit/push。

## 审计与实际处理

原89份输入全部读取/解析或按内容比对，包含Git忽略的manifests快照。构建输入引用图，检查当前源码的加载/继承、Profile/base_scenario、准备锁、已归档Run的读取语义、两个Matrix、Spec/Catalog关系和原始证据。非文本搜索单独决定删除：当前根及其传递闭包60份KEEP，进一步遍历84份结构化外部闭包，没有增加待保留的实验文件。全文检索另记录182个历史引用容器，其中178在evidence、4在历史docs，tools/tests/src无待移除路径的活动直接引用。

ARCHIVE 29份已移出工作树；在验证的仓库外压缩备份中按原路径保存。DELETE=0，REVIEW=0。两组逐字节重复内容（两个QoS、v2两个场景依赖锁）都有冻结路径或代码引用，均KEEP，不能为减少数字改历史锁。ARCHIVE文件不是被判错误或无实验价值。

## 四类逐文件清单

下表覆盖精简前全部89份文件。完整原SHA、直接引用及已有同SHA证据副本见仓库外classification-before.json。ARCHIVE的恢复成员名与原路径相同；其移出理由逐项列明。KEEP文件字节不变。

| 文件 | 分类 | 原字节 | 原SHA256前12位 | 理由 |
|---|---|---:|---|---|
| experiments/evaluation_contract.md | KEEP | 4115 | 3cfe5ecc9a0c | 必要数据/评价合同说明；保留语义 |
| experiments/evaluations/p0-06-crossing-kwdqt70u-c11.json | KEEP | 2333 | bd5409ba4a3d | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0-06-crossing-kwdqt70u-c12.json | KEEP | 2332 | e55e4ca6c141 | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0-06-occlusion-kwdqt70u-o5.json | KEEP | 2332 | 0ff539f2aea1 | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0-06-occlusion-kwdqt70u-o6.json | KEEP | 2337 | c8a6bd4dd51b | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0-long-crossing-higher-20261029-a1.json | KEEP | 2349 | dbeafb8309e7 | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0-long-crossing-low-20261029-a1.json | KEEP | 2348 | d2b1f399d1cf | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0-long-occlusion-higher-20261029-a1.json | KEEP | 2351 | 247d72efa390 | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0-long-occlusion-low-20261029-a1.json | KEEP | 2351 | abd0fa77c16c | 具体Run评价Spec，保留历史/长路线离线评价与来源指纹 |
| experiments/evaluations/p0_07_crossing.yaml | KEEP | 1458 | c8f3ca96267e | Concrete Spec索引；两代记录仍被索引，不能当正式Spec或删旧条目 |
| experiments/evaluations/p0_07_observed_occlusion.yaml | KEEP | 1472 | 955623e62e55 | Concrete Spec索引；两代记录仍被索引，不能当正式Spec或删旧条目 |
| experiments/goals/p0_06_crossing_r06.json | ARCHIVE | 391 | 5c5c6f066039 | r06专用旧目标请求，仅旧probe链引用；当前Goal不依赖，原始证据已有同SHA副本 |
| experiments/goals/p0_long_route_task_v1.json | KEEP | 1739 | 6afd53120782 | 当前长路线任务/请求/动态轨迹身份，直接或间接在冻结闭包中 |
| experiments/goals/p0_long_route_v1.json | KEEP | 361 | 17a88cc56542 | 当前长路线任务/请求/动态轨迹身份，直接或间接在冻结闭包中 |
| experiments/manifests/environment.json | ARCHIVE | 24606 | 9bae2fca9b34 | 早期环境/launch/doctor快照，无当前加载或资源锁；Git忽略目录，必须用新验证备份恢复，不能只依赖Git |
| experiments/manifests/ros2_doctor.txt | ARCHIVE | 29011 | ad0686f3bda1 | 早期环境/launch/doctor快照，无当前加载或资源锁；Git忽略目录，必须用新验证备份恢复，不能只依赖Git |
| experiments/manifests/tb3_launch_args.txt | ARCHIVE | 3228 | 1836d9275d5b | 早期环境/launch/doctor快照，无当前加载或资源锁；Git忽略目录，必须用新验证备份恢复，不能只依赖Git |
| experiments/motions/crossing.yaml | KEEP | 651 | 3ae3560dc19e | 动态合同motion负例测试与v2通用probe模板仍加载 |
| experiments/motions/crossing_long_route_v1.yaml | KEEP | 467 | dfa658b2b54c | 当前长路线任务/请求/动态轨迹身份，直接或间接在冻结闭包中 |
| experiments/motions/crossing_r06.yaml | ARCHIVE | 464 | d6cc19b2868b | 旧crossing候选Motion，仅对应旧Probe链引用；与对应Scenario/Profile/锁一起可恢复归档 |
| experiments/motions/crossing_r07.yaml | ARCHIVE | 466 | 94abba464b27 | 旧crossing候选Motion，仅对应旧Probe链引用；与对应Scenario/Profile/锁一起可恢复归档 |
| experiments/motions/crossing_r08.yaml | ARCHIVE | 466 | b6b93d2d06e2 | 旧crossing候选Motion，仅对应旧Probe链引用；与对应Scenario/Profile/锁一起可恢复归档 |
| experiments/motions/crossing_r09.yaml | ARCHIVE | 465 | 27bc967c3341 | 旧crossing候选Motion，仅对应旧Probe链引用；与对应Scenario/Profile/锁一起可恢复归档 |
| experiments/motions/crossing_r10.yaml | KEEP | 464 | be4458ed0bd5 | 旧r10 Motion仍在当前crossing/occlusion准备锁与v2锁中；存在间接冻结依赖 |
| experiments/motions/observed_occlusion.yaml | ARCHIVE | 768 | cd5154316ba6 | 旧短路线S3轨迹，当前使用long_route_v1；28份原始证据同SHA副本，历史输入价值保留 |
| experiments/motions/observed_occlusion_long_route_v1.yaml | KEEP | 774 | 3d43d1c4ff3a | 当前长路线任务/请求/动态轨迹身份，直接或间接在冻结闭包中 |
| experiments/p0_06_dependencies-long-route-v1.json | ARCHIVE | 58834 | 28f919bc6b69 | 阶段性根依赖清单；当前输入指向evidence内准备锁或tool_layout_v2独立锁，原版留可恢复外部备份 |
| experiments/p0_06_dependencies-r06.json | ARCHIVE | 51939 | c1911d54f1f9 | 阶段性根依赖清单；当前输入指向evidence内准备锁或tool_layout_v2独立锁，原版留可恢复外部备份 |
| experiments/p0_06_dependencies-r07.json | ARCHIVE | 51748 | 1d993af21c3e | 阶段性根依赖清单；当前输入指向evidence内准备锁或tool_layout_v2独立锁，原版留可恢复外部备份 |
| experiments/p0_06_dependencies-r08.json | ARCHIVE | 57829 | 811e44a197ec | 阶段性根依赖清单；当前输入指向evidence内准备锁或tool_layout_v2独立锁，原版留可恢复外部备份 |
| experiments/p0_06_dependencies-r09.json | ARCHIVE | 57829 | 8ac5cef1e24b | 阶段性根依赖清单；当前输入指向evidence内准备锁或tool_layout_v2独立锁，原版留可恢复外部备份 |
| experiments/p0_06_dependencies-r10.json | ARCHIVE | 57829 | 865635697089 | 阶段性根依赖清单；当前输入指向evidence内准备锁或tool_layout_v2独立锁，原版留可恢复外部备份 |
| experiments/p0_06_dependencies.json | ARCHIVE | 51370 | 11d8155774d1 | 阶段性根依赖清单；当前输入指向evidence内准备锁或tool_layout_v2独立锁，原版留可恢复外部备份 |
| experiments/p0_06_geometry-r02.json | KEEP | 1754 | 69e580656b35 | 当前S1/S3及probe读取已校准外参/几何；与未实测旧geometry不同 |
| experiments/p0_06_geometry.json | ARCHIVE | 990 | 9d4439d3e572 | 未实测candidate几何，r02已提供真实校准；历史probe原件仍有两份同SHA副本，保留原语义至外部备份 |
| experiments/p0_06_native_dependencies.json | KEEP | 33940 | 2446673445d9 | 当前依赖闭包保留本机原生库身份；现有漂移单独报告 |
| experiments/p0_fault_matrix.yaml | KEEP | 3646 | 28f5dca62c35 | 当前正常20槽位/故障6槽位矩阵；新cohort静态预检入口 |
| experiments/p0_smoke_matrix.yaml | KEEP | 11498 | 222a403805d2 | 当前正常20槽位/故障6槽位矩阵；新cohort静态预检入口 |
| experiments/profiles/p0_06_crossing.yaml | KEEP | 1604 | 52e8a38de0fb | 重点保护的历史ready正式入口；旧源码路径失配仍拒绝，保留原SHA及来源，不把它改写为新布局可用 |
| experiments/profiles/p0_06_crossing_r06_probe.yaml | ARCHIVE | 1102 | db5e5a69dfea | 旧probe Profile，当前生产/测试只用v2模板或正式长路线Profile；连同旧Scenario及依赖整体归档 |
| experiments/profiles/p0_06_crossing_r07_probe.yaml | ARCHIVE | 1102 | 5ee661cec70e | 旧probe Profile，当前生产/测试只用v2模板或正式长路线Profile；连同旧Scenario及依赖整体归档 |
| experiments/profiles/p0_06_crossing_r08_probe.yaml | ARCHIVE | 1102 | 1587fd5dc87c | 旧probe Profile，当前生产/测试只用v2模板或正式长路线Profile；连同旧Scenario及依赖整体归档 |
| experiments/profiles/p0_06_crossing_r09_probe.yaml | ARCHIVE | 1102 | 57e65cac57ec | 旧probe Profile，当前生产/测试只用v2模板或正式长路线Profile；连同旧Scenario及依赖整体归档 |
| experiments/profiles/p0_06_crossing_r10_probe.yaml | ARCHIVE | 1102 | 9edf40f70da9 | 旧probe Profile，当前生产/测试只用v2模板或正式长路线Profile；连同旧Scenario及依赖整体归档 |
| experiments/profiles/p0_06_observed_occlusion.yaml | KEEP | 1614 | b4f374ad91e0 | 重点保护的历史ready正式入口；旧源码路径失配仍拒绝，保留原SHA及来源，不把它改写为新布局可用 |
| experiments/profiles/p0_06_probe.yaml | ARCHIVE | 1089 | 1b1e34cb6502 | 旧probe Profile，当前生产/测试只用v2模板或正式长路线Profile；连同旧Scenario及依赖整体归档 |
| experiments/profiles/p0_08_controller-deactivate.yaml | KEEP | 6738 | a76998bf35f3 | P0-08两个Matrix及其Scenario/Profile/World闭包；保留故障/到达/场景语义 |
| experiments/profiles/p0_08_normal.yaml | KEEP | 6688 | 6de3d0ad2803 | P0-08两个Matrix及其Scenario/Profile/World闭包；保留故障/到达/场景语义 |
| experiments/profiles/p0_08_pause-gazebo.yaml | KEEP | 6720 | 6bc2cb45d2bb | P0-08两个Matrix及其Scenario/Profile/World闭包；保留故障/到达/场景语义 |
| experiments/profiles/p0_08_short-deadline.yaml | KEEP | 6724 | 340e79c7729c | P0-08两个Matrix及其Scenario/Profile/World闭包；保留故障/到达/场景语义 |
| experiments/profiles/p0_08_static.yaml | KEEP | 2278 | 853dfbf1afc4 | P0-08两个Matrix及其Scenario/Profile/World闭包；保留故障/到达/场景语义 |
| experiments/profiles/p0_08_world.sdf | KEEP | 2522 | 6213ae5f6643 | P0-08两个Matrix及其Scenario/Profile/World闭包；保留故障/到达/场景语义 |
| experiments/profiles/p0_long_route_observe_v1.yaml | KEEP | 6227 | 82cd45f430a0 | 当前静态长路线/GUI观察的Scenario或Profile；预检、任务和基线核验仍需 |
| experiments/profiles/p0_long_route_static_v1.yaml | KEEP | 6235 | 5b085a17d41a | 当前静态长路线/GUI观察的Scenario或Profile；预检、任务和基线核验仍需 |
| experiments/recording_qos.yaml | KEEP | 1600 | 1e052a6e74fe | 静态/长路线/P0-08及ROS协议测试使用；虽与动态QoS相同，路径仍被锁定 |
| experiments/recording_qos_p0_06.yaml | KEEP | 1600 | 1e052a6e74fe | dynamic_contract.profile_inputs与probe准备器固定路径，并被当前两个动态依赖锁冻结；不能仅因字节重复删除 |
| experiments/runner_profile.yaml | KEEP | 6042 | 24ca471f1a57 | 旧静态与v2 probe基Profile、回归测试仍依赖，不能仅保留长路线派生Profile |
| experiments/scenarios/crossing.yaml | KEEP | 5511 | 17155eab5326 | 重点保护的历史ready正式入口；旧源码路径失配仍拒绝，保留原SHA及来源，不把它改写为新布局可用 |
| experiments/scenarios/observed_occlusion.yaml | KEEP | 5602 | 3de6b18605d2 | 重点保护的历史ready正式入口；旧源码路径失配仍拒绝，保留原SHA及来源，不把它改写为新布局可用 |
| experiments/scenarios/official_static.yaml | KEEP | 1636 | 598d903a76d9 | P0-02 Scenario/Schema合同测试固定读取的draft；draft并不等于无用 |
| experiments/scenarios/p0_05_official_static.yaml | KEEP | 2339 | 3fcf918415d5 | Runner历史兼容、v2 probe基Scenario及多个回归测试仍读取 |
| experiments/scenarios/p0_06_crossing_r06_probe.yaml | ARCHIVE | 4784 | 53dbe3bf3265 | 旧probe Scenario，保留当时review/状态/阈值，不当作新布局正式入口；关联链整体归档 |
| experiments/scenarios/p0_06_crossing_r07_probe.yaml | ARCHIVE | 4822 | 7b6b74083480 | 旧probe Scenario，保留当时review/状态/阈值，不当作新布局正式入口；关联链整体归档 |
| experiments/scenarios/p0_06_crossing_r08_probe.yaml | ARCHIVE | 4822 | c500809a7bed | 旧probe Scenario，保留当时review/状态/阈值，不当作新布局正式入口；关联链整体归档 |
| experiments/scenarios/p0_06_crossing_r09_probe.yaml | ARCHIVE | 4838 | 7ff4cc04f233 | 旧probe Scenario，保留当时review/状态/阈值，不当作新布局正式入口；关联链整体归档 |
| experiments/scenarios/p0_06_crossing_r10_probe.yaml | ARCHIVE | 4822 | 13a5fa21eccb | 旧probe Scenario，保留当时review/状态/阈值，不当作新布局正式入口；关联链整体归档 |
| experiments/scenarios/p0_06_probe.yaml | ARCHIVE | 4791 | a1b92abaf502 | 旧probe Scenario，保留当时review/状态/阈值，不当作新布局正式入口；关联链整体归档 |
| experiments/scenarios/p0_long_route_observe_v1.yaml | KEEP | 2293 | a375851a1c6a | 当前静态长路线/GUI观察的Scenario或Profile；预检、任务和基线核验仍需 |
| experiments/scenarios/p0_long_route_static_v1.yaml | KEEP | 2296 | d6c756c0468b | 当前静态长路线/GUI观察的Scenario或Profile；预检、任务和基线核验仍需 |
| experiments/schema.md | KEEP | 13000 | 4466a4c230a8 | 必要数据/评价合同说明；保留语义 |
| experiments/schemas/dynamic_report.schema.json | KEEP | 1804 | 7df415bc0094 | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/schemas/evaluation_input.schema.json | KEEP | 5148 | 96e82fa90a85 | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/schemas/evaluation_report.schema.json | KEEP | 32453 | 7633ed66742c | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/schemas/evaluation_spec.schema.json | KEEP | 3367 | 934a47868501 | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/schemas/manifest.schema.json | KEEP | 5399 | 67dfe718823e | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/schemas/result.schema.json | KEEP | 11696 | eb8ad3948967 | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/schemas/scenario.schema.json | KEEP | 9872 | 2dbd1750cef1 | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/schemas/scenario.v2.schema.json | KEEP | 14759 | 8db34ed0e68e | JSON Schema合同；validator/geometry按固定文件名加载；保持字段与hash |
| experiments/tool_layout_v2/README.md | KEEP | 919 | 16e88edf2a78 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/dependencies/crossing.json | KEEP | 66137 | b347d8dc23f6 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/dependencies/observed_occlusion.json | KEEP | 66137 | b347d8dc23f6 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/dependencies/p0_06_probe.json | KEEP | 60137 | e0c80745f7bf | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/layout.json | KEEP | 34180 | f884fd2e157c | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/profiles/p0_06_crossing.yaml | KEEP | 1619 | cf92e1ba38b5 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/profiles/p0_06_observed_occlusion.yaml | KEEP | 1629 | 687ead86d2f1 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/profiles/p0_06_probe.yaml | KEEP | 1104 | d20470ffd626 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/scenarios/crossing.yaml | KEEP | 5064 | c9aa43feb072 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/scenarios/observed_occlusion.yaml | KEEP | 5135 | 92d9b827171c | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |
| experiments/tool_layout_v2/scenarios/p0_06_probe.yaml | KEEP | 4836 | 4a9cf5909184 | 工具布局v2独立依赖/输入/来源版本；含当前draft准备入口及离线测试，不与旧ready证明混用 |

## 清理前完整目录

~~~text
experiments/evaluation_contract.md
experiments/evaluations
experiments/evaluations/p0-06-crossing-kwdqt70u-c11.json
experiments/evaluations/p0-06-crossing-kwdqt70u-c12.json
experiments/evaluations/p0-06-occlusion-kwdqt70u-o5.json
experiments/evaluations/p0-06-occlusion-kwdqt70u-o6.json
experiments/evaluations/p0-long-crossing-higher-20261029-a1.json
experiments/evaluations/p0-long-crossing-low-20261029-a1.json
experiments/evaluations/p0-long-occlusion-higher-20261029-a1.json
experiments/evaluations/p0-long-occlusion-low-20261029-a1.json
experiments/evaluations/p0_07_crossing.yaml
experiments/evaluations/p0_07_observed_occlusion.yaml
experiments/goals
experiments/goals/p0_06_crossing_r06.json
experiments/goals/p0_long_route_task_v1.json
experiments/goals/p0_long_route_v1.json
experiments/manifests
experiments/manifests/environment.json
experiments/manifests/ros2_doctor.txt
experiments/manifests/tb3_launch_args.txt
experiments/motions
experiments/motions/crossing.yaml
experiments/motions/crossing_long_route_v1.yaml
experiments/motions/crossing_r06.yaml
experiments/motions/crossing_r07.yaml
experiments/motions/crossing_r08.yaml
experiments/motions/crossing_r09.yaml
experiments/motions/crossing_r10.yaml
experiments/motions/observed_occlusion.yaml
experiments/motions/observed_occlusion_long_route_v1.yaml
experiments/p0_06_dependencies-long-route-v1.json
experiments/p0_06_dependencies-r06.json
experiments/p0_06_dependencies-r07.json
experiments/p0_06_dependencies-r08.json
experiments/p0_06_dependencies-r09.json
experiments/p0_06_dependencies-r10.json
experiments/p0_06_dependencies.json
experiments/p0_06_geometry-r02.json
experiments/p0_06_geometry.json
experiments/p0_06_native_dependencies.json
experiments/p0_fault_matrix.yaml
experiments/p0_smoke_matrix.yaml
experiments/profiles
experiments/profiles/p0_06_crossing.yaml
experiments/profiles/p0_06_crossing_r06_probe.yaml
experiments/profiles/p0_06_crossing_r07_probe.yaml
experiments/profiles/p0_06_crossing_r08_probe.yaml
experiments/profiles/p0_06_crossing_r09_probe.yaml
experiments/profiles/p0_06_crossing_r10_probe.yaml
experiments/profiles/p0_06_observed_occlusion.yaml
experiments/profiles/p0_06_probe.yaml
experiments/profiles/p0_08_controller-deactivate.yaml
experiments/profiles/p0_08_normal.yaml
experiments/profiles/p0_08_pause-gazebo.yaml
experiments/profiles/p0_08_short-deadline.yaml
experiments/profiles/p0_08_static.yaml
experiments/profiles/p0_08_world.sdf
experiments/profiles/p0_long_route_observe_v1.yaml
experiments/profiles/p0_long_route_static_v1.yaml
experiments/recording_qos.yaml
experiments/recording_qos_p0_06.yaml
experiments/results
experiments/runner_profile.yaml
experiments/scenarios
experiments/scenarios/crossing.yaml
experiments/scenarios/observed_occlusion.yaml
experiments/scenarios/official_static.yaml
experiments/scenarios/p0_05_official_static.yaml
experiments/scenarios/p0_06_crossing_r06_probe.yaml
experiments/scenarios/p0_06_crossing_r07_probe.yaml
experiments/scenarios/p0_06_crossing_r08_probe.yaml
experiments/scenarios/p0_06_crossing_r09_probe.yaml
experiments/scenarios/p0_06_crossing_r10_probe.yaml
experiments/scenarios/p0_06_probe.yaml
experiments/scenarios/p0_long_route_observe_v1.yaml
experiments/scenarios/p0_long_route_static_v1.yaml
experiments/schema.md
experiments/schemas
experiments/schemas/dynamic_report.schema.json
experiments/schemas/evaluation_input.schema.json
experiments/schemas/evaluation_report.schema.json
experiments/schemas/evaluation_spec.schema.json
experiments/schemas/manifest.schema.json
experiments/schemas/result.schema.json
experiments/schemas/scenario.schema.json
experiments/schemas/scenario.v2.schema.json
experiments/tool_layout_v2
experiments/tool_layout_v2/README.md
experiments/tool_layout_v2/dependencies
experiments/tool_layout_v2/dependencies/crossing.json
experiments/tool_layout_v2/dependencies/observed_occlusion.json
experiments/tool_layout_v2/dependencies/p0_06_probe.json
experiments/tool_layout_v2/layout.json
experiments/tool_layout_v2/profiles
experiments/tool_layout_v2/profiles/p0_06_crossing.yaml
experiments/tool_layout_v2/profiles/p0_06_observed_occlusion.yaml
experiments/tool_layout_v2/profiles/p0_06_probe.yaml
experiments/tool_layout_v2/scenarios
experiments/tool_layout_v2/scenarios/crossing.yaml
experiments/tool_layout_v2/scenarios/observed_occlusion.yaml
experiments/tool_layout_v2/scenarios/p0_06_probe.yaml
~~~

## 清理后完整目录

~~~text
experiments/
├── evaluations/
│   ├── p0-06-crossing-kwdqt70u-c11.json
│   ├── p0-06-crossing-kwdqt70u-c12.json
│   ├── p0-06-occlusion-kwdqt70u-o5.json
│   ├── p0-06-occlusion-kwdqt70u-o6.json
│   ├── p0-long-crossing-higher-20261029-a1.json
│   ├── p0-long-crossing-low-20261029-a1.json
│   ├── p0-long-occlusion-higher-20261029-a1.json
│   ├── p0-long-occlusion-low-20261029-a1.json
│   ├── p0_07_crossing.yaml
│   └── p0_07_observed_occlusion.yaml
├── goals/
│   ├── p0_long_route_task_v1.json
│   └── p0_long_route_v1.json
├── motions/
│   ├── crossing.yaml
│   ├── crossing_long_route_v1.yaml
│   ├── crossing_r10.yaml
│   └── observed_occlusion_long_route_v1.yaml
├── profiles/
│   ├── p0_06_crossing.yaml
│   ├── p0_06_observed_occlusion.yaml
│   ├── p0_08_controller-deactivate.yaml
│   ├── p0_08_normal.yaml
│   ├── p0_08_pause-gazebo.yaml
│   ├── p0_08_short-deadline.yaml
│   ├── p0_08_static.yaml
│   ├── p0_08_world.sdf
│   ├── p0_long_route_observe_v1.yaml
│   └── p0_long_route_static_v1.yaml
├── results/
├── scenarios/
│   ├── crossing.yaml
│   ├── observed_occlusion.yaml
│   ├── official_static.yaml
│   ├── p0_05_official_static.yaml
│   ├── p0_long_route_observe_v1.yaml
│   └── p0_long_route_static_v1.yaml
├── schemas/
│   ├── dynamic_report.schema.json
│   ├── evaluation_input.schema.json
│   ├── evaluation_report.schema.json
│   ├── evaluation_spec.schema.json
│   ├── manifest.schema.json
│   ├── result.schema.json
│   ├── scenario.schema.json
│   └── scenario.v2.schema.json
├── tool_layout_v2/
│   ├── dependencies/
│   │   ├── crossing.json
│   │   ├── observed_occlusion.json
│   │   └── p0_06_probe.json
│   ├── profiles/
│   │   ├── p0_06_crossing.yaml
│   │   ├── p0_06_observed_occlusion.yaml
│   │   └── p0_06_probe.yaml
│   ├── scenarios/
│   │   ├── crossing.yaml
│   │   ├── observed_occlusion.yaml
│   │   └── p0_06_probe.yaml
│   ├── README.md
│   └── layout.json
├── README.md
├── evaluation_contract.md
├── p0_06_geometry-r02.json
├── p0_06_native_dependencies.json
├── p0_fault_matrix.yaml
├── p0_smoke_matrix.yaml
├── recording_qos.yaml
├── recording_qos_p0_06.yaml
├── runner_profile.yaml
└── schema.md
~~~

## 引用、冻结和历史身份

- 不修改任何保留的Scenario/Profile/Goal/Motion/World/Matrix/Schema/Spec/Catalog或历史SHA；r10 Motion、旧静态Scenario和runner_profile因传递/测试依赖保留。
- 旧四轮评价Spec仍被两个Catalog引用，保留8份Concrete Spec和两个Catalog。geometry只读原Run的归档实体/资源和源数据，不依赖刚移出的Probe模板。
- manifests三份快照缺少充分Git恢复保证，因此先作新可信备份与独立恢复；原evidence不修改。
- 当前crossing/occlusion准备锁实际位于evidence/env/goal-long-route-v1-20261029；新布局准备锁位于tool_layout_v2。根旧阶段清单不在当前闭包，但仍有历史工程价值。
- batch_core.source_inventory会递归扫描整个experiments；本次删旧文件/新增README使新cohort清单变化。旧批次此前已经cohort drift，检查保持拒绝，不修改Manifest或关闭校验。已完成批次及原始结果保留；精简不承诺旧源码/系统环境自动恢复。
- 当前文档仅新增实验README/本报告、在根README与docs导航/场景目录加入恢复入口。历史报告中的原路径文字与证据引用保留，通过本表和备份解释。无需修改代码或测试引用。

## 验证与既有问题

| 检查 | 清理前 | 清理后 |
|---|---|---|
| Schema元校验 | 8通过 | 8通过 |
| 保留结构化输入严格解析 | 56通过 | 56通过 |
| 保留Scenario | 7 VALID、2既有INVALID | 相同 |
| Concrete Evaluation Spec Schema | 8通过 | 8通过 |
| Catalog | 2份，每份4个Spec | 相同，索引指向的8份Spec保留 |
| 静态legacy/long MPPI/DWB预检 | 4次退出0 | 相同 |
| GUI等待模式--check | 退出0 | 相同 |
| 新normal/fault Matrix --check | 2次退出0 | 相同 |
| S1/S3旧正式入口--check | 2次退出2，缺少旧源码路径 | 原诊断逐字相同 |
| 两个已完成旧cohort --check | 2次退出2，cohort drift | 原诊断逐字相同 |
| 8个明确历史Run几何--check | 全部退出0 | 相同；没有正式评价 |
| 146个原Run合同 | 143 VALID、1 INCOMPLETE、2 INVALID | 逐Run诊断完全相同 |
| 默认安全离线pytest | 468通过、3失败、5跳过、31 ROS未执行 | 相同，476个用例逐项状态一致 |
| 本地Markdown链接/锚点 | 最终扫描260项 | 0失效；1个外部链接未联网验证 |

Profile没有独立的Profile JSON Schema文件；本次通过原Runner加载与Matrix验证检查其语义，没有虚构一个Profile Schema。静态Scenario的原draft字段不更改；能通过既有静态Runner分支预检不等于宣称ready或导航通过。

既有问题单独保留：
- 三项pytest失败为test_runner_gui.py的dynamic旧冻结入口预检，清理前后相同，未删断言或增加skip。
- p0_06_native_dependencies.json中cmr_scene_driver、cmr_truth_capture两个历史SHA与当前安装文件不同；原清单不改。新v2输入中的实际指纹属于独立版本，不能重算旧hash。
- 原Run p0-03-0pl9lbsf缺result仍INCOMPLETE；p0-04-mppi-cnr5xdgs与p0-04-mppi-zkxon7rx仍因UTC格式INVALID，没有补写或改历史记录。
- 当前S1/S3历史ready入口仍被源码路径身份门拒绝，新的v2模板仍draft；这次清理不修复或掩盖工具迁移后的真实集成缺口。
- 两个旧P0-08批次保持cohort drift拒绝。本次清理也会改变新批次source_inventory，必须新cohort或恢复旧完整身份，不能将旧成功计入新分母。

34156个其他既有文件SHA全部不变，60份KEEP输入SHA逐份不变；Git暂存区、分支和HEAD保持。CLI结果（含stdout/stderr）19项完全一致，146份原Run诊断完全一致。实际命令/退出、资源引用与错误见inputs-before/after.json/log；pytest/JUnit见pytest-before/after.log/xml。新链接检查见links-after.json，未启动任何仿真/ROS节点。


## 数量与体积

| 指标 | 数值 |
|---|---:|
| 精简前文件数 | 89 |
| 精简前字节 | 985447 |
| 移出工作树的ARCHIVE文件 | 29 |
| 移出字节 | 483711 |
| 保留原输入 | 60，501736字节 |
| 新增experiments/README.md | 1，6439字节 |
| 精简后文件数 | 61 |
| 精简后字节 | 508175 |
| 净减少 | 28文件，477272字节（48.43%） |

原manifests目录移空后删除空目录。没有新增仓库内archive层级。DELETE类为0：没有将仍有历史价值的输入谎称无用垃圾；实际减少来自经过验证的可恢复外部归档，而不是在experiments内部换文件夹。


## 回滚与审计原件

仓库外专用目录：/home/robot/cmr_backups/experiments-cleanup-20261031-mvvpkims。experiments-before.tgz、archive-sha256.txt、inventory-before.json、classification-before.json、removed-files.json、input-graph.json、external-closure.json、candidate-evidence-copies.json、historical-reference-files.txt和restore-verification.json记录原件与依据。完整89份已独立恢复并核对原SHA。恢复说明见[实验README](../../experiments/README.md)。

恢复文件不等于恢复旧运行环境：还须对应版本源码、系统包、模型/地图、原生二进制和已有Run输入/资源快照。两个原生程序存在历史SHA漂移，未改旧清单。

本次只移除经过闭包审计的历史输入。其余60份因当前入口、合同测试、基线或冻结依赖保留；不能因draft、低版本号或内容相同继续删。没有待决定的REVIEW删除，但既有运行阻塞单独保留。未启动Gazebo/Nav2/ROS节点、发送初值/Goal、执行真实实验、重建、正式评价或发布。

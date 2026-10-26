# CMR 测试组织与运行说明

本次为路径与导入重构，保持测试用例、断言、参数和fixtures语义。C++测试留在原包，未创建空的P1目录。

## 功能分类与完整迁移表

| 原路径 | 新路径 | 分类理由 |
|---|---|---|
| tests/test_run_schema.py | tests/contracts/test_run_schema.py | Scenario/manifest/result、引用/hash、时间与CLI合同 |
| tests/test_dynamic_contract.py | tests/contracts/test_dynamic_contract.py | 动态v2合同、准备凭据与资源绑定；主职责为合同验证 |
| tests/test_long_route_task.py | tests/navigation/test_long_route_task.py | 导航任务目标、出生位姿、版本绑定与录制边界 |
| tests/test_runner_lifecycle.py | tests/runner/test_runner_lifecycle.py | Runner状态归约、期限竞争、到达/停车及失败保留 |
| tests/test_runner_processes.py | tests/runner/test_runner_processes.py | 项目锁、PID身份、后代归属、信号升级与有界清理 |
| tests/test_runner_bag_content.py | tests/runner/test_runner_bag_content.py | 临时合成MCAP的离线序列化/内容检查，不发布ROS消息 |
| tests/test_runner_faults.py | tests/runner/test_runner_faults.py | 故障白名单、冻结策略和恢复调度 |
| tests/test_failure_stop.py | tests/runner/test_failure_stop.py | 失败/取消后的停止证据及负例 |
| tests/test_runner_gui.py | tests/runner/test_runner_gui.py | GUI生命周期适配、人工等待和窗口/场景证据 |
| tests/test_gui_scene_probe.py | tests/runner/test_gui_scene_probe.py | 假Transport服务/订阅下的场景探针 |
| tests/test_runner_ros_contract.py | tests/runner/test_runner_ros_contract.py | 隔离ROS Action/生命周期/录制协议，含真实安装定位组件 |
| tests/test_p0_fault_ros_contract.py | tests/runner/test_p0_fault_ros_contract.py | 隔离ROS关闭状态、故障白名单和manager协调 |
| tests/test_dynamic_runner.py | tests/dynamic/test_dynamic_runner.py | 动态会话的所有权、触发、暂停和完成边界 |
| tests/test_dynamic_recording.py | tests/dynamic/test_dynamic_recording.py | 原始位姿/scan对齐、运动、可见性/遮挡及缺测反例 |
| tests/test_dynamic_ros_contract.py | tests/dynamic/test_dynamic_ros_contract.py | probe worker的隔离ROS协议 |
| tests/test_geometry_metrics.py | tests/geometry/test_geometry_metrics.py | 圆间距、线性段最小值及数学反例 |
| tests/test_geometry_alignment.py | tests/geometry/test_geometry_alignment.py | 多率对齐、不变性与缺口覆盖 |
| tests/test_geometry_inputs.py | tests/geometry/test_geometry_inputs.py | 归档输入、实体、时间、未知语义和fixture隔离 |
| tests/test_geometry_cli.py | tests/geometry/test_geometry_cli.py | 几何CLI、排他输出、Schema和资源路径 |
| tests/test_geometry_prepare.py | tests/geometry/test_geometry_prepare.py | 具体Spec生成、身份绑定、预检与历史保护 |
| tests/test_smoke_matrix.py | tests/batch/test_smoke_matrix.py | 串行矩阵账本、恢复去重、冻结输入与预检 |
| tests/test_p0_summary.py | tests/batch/test_p0_summary.py | 批次审计、未开始/缺结果与分母守恒 |
| tests/test_runner_summary.py | tests/batch/test_runner_summary.py | 回合汇总中缺失/非法记录的只读索引 |

22个原跟踪文件使用git mv，原未跟踪的test_geometry_prepare.py使用rename并保留此前实现。没有删除测试文件。新增tests/__init__.py及六个分类目录的__init__.py，使跨文件辅助函数/fixture使用显式相对导入。未创建conftest.py：现有局部fixture无需提升作用域。

## 完整目录树

不显示旧__pycache__缓存；全部原fixtures文件保留原位置、内容和hash。

```text
tests/
├── batch/
│   ├── __init__.py
│   ├── test_p0_summary.py
│   ├── test_runner_summary.py
│   └── test_smoke_matrix.py
├── contracts/
│   ├── __init__.py
│   ├── test_dynamic_contract.py
│   ├── test_run_schema.py
│   └── test_tool_layout.py
├── dynamic/
│   ├── __init__.py
│   ├── test_dynamic_recording.py
│   ├── test_dynamic_ros_contract.py
│   └── test_dynamic_runner.py
├── fixtures/
│   ├── geometry/
│   │   ├── README.md
│   │   ├── conflicting_duplicate.json
│   │   ├── gap.json
│   │   ├── interior.json
│   │   ├── missing_pose.json
│   │   ├── negative_radius.json
│   │   ├── wrong_epoch.json
│   │   └── wrong_frame.json
│   ├── runs/
│   │   ├── collision_false_scoped/
│   │   │   ├── artifacts/
│   │   │   │   ├── config.txt
│   │   │   │   ├── constraints.txt
│   │   │   │   ├── evaluation.txt
│   │   │   │   ├── parameters.txt
│   │   │   │   └── scenario.yaml
│   │   │   ├── manifest.json
│   │   │   └── result.json
│   │   ├── collision_observed_partial/
│   │   │   ├── artifacts/
│   │   │   │   ├── config.txt
│   │   │   │   ├── constraints.txt
│   │   │   │   ├── evaluation.txt
│   │   │   │   ├── parameters.txt
│   │   │   │   └── scenario.yaml
│   │   │   ├── manifest.json
│   │   │   └── result.json
│   │   ├── incomplete/
│   │   │   ├── artifacts/
│   │   │   │   ├── config.txt
│   │   │   │   ├── constraints.txt
│   │   │   │   └── scenario.yaml
│   │   │   └── manifest.json
│   │   ├── invalid_collision_false/
│   │   │   ├── artifacts/
│   │   │   │   ├── config.txt
│   │   │   │   ├── constraints.txt
│   │   │   │   ├── parameters.txt
│   │   │   │   └── scenario.yaml
│   │   │   ├── manifest.json
│   │   │   └── result.json
│   │   ├── nav_timeout/
│   │   │   ├── artifacts/
│   │   │   │   ├── config.txt
│   │   │   │   ├── constraints.txt
│   │   │   │   ├── parameters.txt
│   │   │   │   └── scenario.yaml
│   │   │   ├── manifest.json
│   │   │   └── result.json
│   │   ├── startup_failed/
│   │   │   ├── artifacts/
│   │   │   │   ├── config.txt
│   │   │   │   ├── constraints.txt
│   │   │   │   └── scenario.yaml
│   │   │   ├── manifest.json
│   │   │   └── result.json
│   │   └── succeeded_unknown/
│   │       ├── artifacts/
│   │       │   ├── config.txt
│   │       │   ├── constraints.txt
│   │       │   ├── parameters.txt
│   │       │   └── scenario.yaml
│   │       ├── manifest.json
│   │       └── result.json
│   ├── scenarios/
│   │   ├── crossing_draft.yaml
│   │   └── observed_occlusion_draft.yaml
│   ├── README.md
│   └── gazebo_gui_logs.json
├── geometry/
│   ├── __init__.py
│   ├── test_geometry_alignment.py
│   ├── test_geometry_cli.py
│   ├── test_geometry_inputs.py
│   ├── test_geometry_metrics.py
│   └── test_geometry_prepare.py
├── navigation/
│   ├── __init__.py
│   └── test_long_route_task.py
├── runner/
│   ├── __init__.py
│   ├── test_failure_stop.py
│   ├── test_gui_scene_probe.py
│   ├── test_p0_fault_ros_contract.py
│   ├── test_runner_bag_content.py
│   ├── test_runner_faults.py
│   ├── test_runner_gui.py
│   ├── test_runner_lifecycle.py
│   ├── test_runner_processes.py
│   └── test_runner_ros_contract.py
├── README.md
└── __init__.py
```

C++保留位置：

- src/cmr_core/test/core_test.cpp：straight、arc、near_zero、wheels、gate五个CTest组。
- src/cmr_sim/test/scene_timing_test.cpp：scene_timing。
- src/cmr_sim/test/raw_proto_test.cpp：raw_proto_roundtrip。

这些是普通可执行程序＋CTest注册，没有GTest。CMake/package.xml和colcon包结构未改，没有Python测试安装配置需要迁移。

## 安全集合和导入规则

pytest.ini设置testpaths=tests、pythonpath仅为仓库根（tools.*包导入），默认-m "not ros_contract"。目录成为包，几何fixture/sample和ROS fixture保留原定义，改为相对导入；基于__file__的仓库根改为parents[2]，geometry静态资源仍定位tests/fixtures/geometry。无需手动设置临时PYTHONPATH。

默认排除以下31项（仍可完整收集，不记为skip）：
- runner/test_runner_ros_contract.py：22项，其中三次启动真实安装的Nav2定位组件；其他会创建假ROS节点、worker或recorder。
- runner/test_p0_fault_ros_contract.py：5项，ROS关闭/故障协议。
- dynamic/test_dynamic_ros_contract.py：4项，probe worker ROS协议。

这三文件标记ros_contract，本次仅收集，未成功进入ROS fixture。不要取消默认标记直接运行全套。它们需要单独授权、隔离domain72及完整依赖；目录重构不代表这些运行行为已重验。纯离线MCAP测试需要Jazzy Python绑定，但不调用rclpy.init、不启动节点。假Transport测试替换Node，不创建Gazebo连接。进程测试仅启动并清理自有临时Python子进程。

geometry/test_geometry_prepare.py中5项真实归档准备兼容测试由CMR_GEOMETRY_TEST_RUNS显式启用；未设置时保持原skip规则。本次安全基线与复测均未设置该变量。它们不是新仿真实验，历史可选测试语义未改。

## 常用命令

先在Ubuntu终端加载现有Jazzy绑定；以下集合不会启动Gazebo/Nav2或发初值/Goal。注意当前有3个原有失败，完整离线命令会如实返回非零。

```bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
python3 -B -m pytest -p no:cacheprovider tests -q
python3 -B -m pytest -p no:cacheprovider tests/runner -q
python3 -B -m pytest -p no:cacheprovider tests/geometry -q
python3 -B -m pytest -p no:cacheprovider tests/batch -q
python3 -B -m pytest -p no:cacheprovider tests/runner/test_runner_lifecycle.py -q
# 完整发现，包括ROS用例；只收集，不执行
python3 -B -m pytest -p no:cacheprovider --collect-only -m "" tests -q
```

从仓库外运行时，传入绝对测试路径和配置即可，例如：

```bash
python3 -B -m pytest -c /home/robot/commercial_mobile_robot/pytest.ini /home/robot/commercial_mobile_robot/tests/geometry -p no:cacheprovider -q
```

### 现有C++ CTest入口

使用已有构建产物，不重新编译。将注册文件复制到新目录，使本次CTest日志不覆盖原构建目录的Testing记录；注册中的可执行路径仍是原二进制。验证仅证明这些现有二进制能执行，不宣称新源码重建。

```bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
CMR_CTEST="$(mktemp -d /home/robot/commercial_mobile_robot/evidence/env/ctest-XXXXXXXX)"
mkdir "$CMR_CTEST/core" "$CMR_CTEST/sim"
cp build/p0-01-gfGjKp8O-release/CTestTestfile.cmake "$CMR_CTEST/core/"
cp build/p0-06-a3179_6v/cmr_sim/CTestTestfile.cmake "$CMR_CTEST/sim/"
ctest --test-dir "$CMR_CTEST/core" --output-on-failure
ctest --test-dir "$CMR_CTEST/sim" --output-on-failure
```

## 重构验证结果

| 指标 | 重构前 | 重构后 |
|---|---:|---:|
| Python测试文件 | 23 | 23 |
| 完整收集测试（含ROS） | 483 | 483 |
| 完整收集中的参数化实例 | 298 | 298 |
| 安全离线集合 | 452 | 452 |
| 安全集合参数化实例 | 280 | 280 |
| 通过 | 444 | 444 |
| 失败 | 3 | 3 |
| 跳过 | 5 | 5 |
| setup/teardown错误 | 0 | 0 |
| Xfail / Xpass | 0 / 0 | 0 / 0 |
| 未执行ROS用例 | 31 | 31 |
| C++核心CTest | 5/5 | 5/5 |
| C++场景/序列化CTest（Jazzy环境） | 2/2 | 2/2 |

完整nodeid映射后、参数实例与逐条执行结局一致。逐文件仅允许的路径/import/ROS标记改动被反向还原后，与备份原文逐字节相等；未改断言或输入。仓库外geometry执行52 passed、5 skipped。

保留的原有失败为runner/test_runner_gui.py::test_check_zero_side_effects_and_old_plan的三个dynamic参数分支（flags0/flags1/flags2）；预检输出SHA256 does not match file bytes。此失败在移动前已出现，未通过改profile/hash、删除参数或加xfail掩盖。后续修复需单独诊断并管理冻结输入，不属于本次重构。

首次基线尝试使用相对--ignore参数未排除ROS文件；31项在domain断言处即失败，未进入rclpy.init。该次失败日志保留，正式离线基线改用20个明确文件列表。首次C++场景测试未source Jazzy导致libgz-utils2.so.2缺失，原日志保留；加载Jazzy后独立新目录2/2通过。不得把这些准备失败抹掉或混入正式离线前后比较。

原始日志、JUnit、完整清单、Git起点、备份、nodeid/结果比较及保护指纹：
/home/robot/commercial_mobile_robot/evidence/env/tests-layout-20261030-cb4dc9/

主要文件：before-offline-explicit.log/.xml、after-offline.log/.xml、before-all-collection.json、after-all-collection.json、outcome-comparison.json、semantic-and-collection-audit.json、outside-cwd-geometry.log、ctest-before-core/、ctest-before-sim-jazzy/、ctest-after-core/、ctest-after-sim/。

## 历史路径与保护

上面的完整迁移表是历史日志、P0指南和验收文档的路径对照。docs/verification/*、docs/verification/long-route-v1*-20261029.md、docs/design/run-schema-test-design.md、仓库外P0指南及所有历史evidence保留原文；它们记录的是当时路径，不作为当前执行命令。docs/guides/geometry-evaluation.md的当前测试命令已更新，旧日期的执行说明保持。未发现CI配置、Shell/Python辅助程序或CMake/colcon配置引用旧tests/test_*.py需要更新。

本次只修改tests组织、pytest配置、README当前测试入口及P0-07当前命令；不触及tools、Schema、机器人源码、控制器配置、Run/Spec/Catalog和旧评价报告。新增证据使用唯一目录。没有commit或push；git mv会留下暂存的路径迁移，原有未提交改动保留。未改变P0验收结论；P0-09仍按现有README为精简收尾，不追认完整发布。

P1以后可按真实模块新增mission等目录；当前不创建空目录或P1测试。

## 工具结构兼容补充（上一版历史结果）

新增contracts/test_tool_layout.py的10项结构用例后，当前完整发现为493项，默认离线462项：454 passed、3个原有失败、5 skipped，31项ROS未执行。前面的483/452计数是测试目录迁移时的历史基线，保留原结论。工具依赖、兼容层、CLI与归档副本验证见 [工具说明](../tools/README.md)。


## 工具布局v2当前验证

目录结构测试现为24项，替换上一版10项兼容层测试；完整发现507项，默认执行476项：468 passed、3 failed、5 skipped，31项ROS未执行。原452项业务用例状态与迁移前一致（444/3/5）。三个失败仍是旧冻结动态输入预检拒绝，未修改断言或增加skip。显式启用两个真实归档Run的Geometry离线集合另为57 passed。原目录迁移及上一版工具验证数字保留为历史结果。

tests/contracts/test_tool_layout.py验证根8入口、阶段5脚本、无shim、统一导入、同一Runner实例、只读check、make_baselines导入不写配置、完整归档搬迁后CLI/Schema/真实Run几何预检。当前命令、完整树及路径映射见[工具说明](../tools/README.md)。真实ROS/GUI/Runner集成未执行。

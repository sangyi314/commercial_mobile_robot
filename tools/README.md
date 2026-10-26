# CMR 工具目录（布局 v2）

根目录只保留8个常用Python CLI及包初始化文件。阶段入口集中在stages；自动调用程序与业务实现归功能子包。移走的根文件没有兼容层、转发脚本或软链接。

## 实际完整目录

~~~text
tools/
├── batch/
│   ├── __init__.py
│   ├── batch_core.py
│   ├── matrix.py
│   └── summary.py
├── contracts/
│   ├── __init__.py
│   ├── dynamic_contract.py
│   ├── task_contract.py
│   └── validation.py
├── dynamic/
│   ├── __init__.py
│   ├── dynamic_geometry.py
│   ├── host_trial.py
│   ├── inspect_dynamic_recording.py
│   ├── probe_preparation.py
│   └── runner_dynamic.py
├── geometry/
│   ├── __init__.py
│   ├── evaluation.py
│   ├── geometry_inputs.py
│   ├── geometry_metrics.py
│   ├── plotting.py
│   └── spec_preparation.py
├── navigation/
│   ├── __init__.py
│   ├── baselines.py
│   ├── manual_navigation.py
│   └── nav_goal.py
├── runner/
│   ├── __init__.py
│   ├── archive.py
│   ├── failure_stop.py
│   ├── gui_scene_probe.py
│   ├── inspect_bag.py
│   ├── observe.py
│   ├── runner_analysis.py
│   ├── runner_core.py
│   ├── runner_faults.py
│   ├── runner_gui.py
│   ├── runner_processes.py
│   └── trial.py
├── stages/
│   ├── __init__.py
│   ├── make_baselines.py
│   ├── p0_03_env.bash
│   ├── p0_03_navigation.py
│   ├── p0_06_host_trial.py
│   └── prepare_dynamic_probe.py
├── README.md
├── __init__.py
├── evaluate_geometry.py
├── plot_geometry.py
├── prepare_geometry_spec.py
├── run_smoke_matrix.py
├── run_trial.py
├── run_trial_observe.py
├── summarize_runs.py
└── validate_run.py
~~~

根目录8个CLI；stages为4个Python入口和1个Bash入口。业务模块数量以实际目录树为准；全树50个Python文件、1个Bash文件（含包初始化）。

## 路径迁移与调用

下表按原扁平布局列出。8个根CLI路径不变，其业务实现已拆入子包；内部模块不通过CLI反向导入。

| 原路径 | 实现新路径 | 当前调用方式 |
|---|---|---|
| tools/run_trial.py | tools/runner/trial.py | python3 tools/run_trial.py …；实现 tools/runner/trial.py |
| tools/run_trial_observe.py | tools/runner/observe.py | python3 tools/run_trial_observe.py …；实现 tools/runner/observe.py |
| tools/run_smoke_matrix.py | tools/batch/matrix.py | python3 tools/run_smoke_matrix.py …；实现 tools/batch/matrix.py |
| tools/validate_run.py | tools/contracts/validation.py | python3 tools/validate_run.py …；实现 tools/contracts/validation.py |
| tools/summarize_runs.py | tools/batch/summary.py | python3 tools/summarize_runs.py …；实现 tools/batch/summary.py |
| tools/prepare_geometry_spec.py | tools/geometry/spec_preparation.py | python3 tools/prepare_geometry_spec.py …；实现 tools/geometry/spec_preparation.py |
| tools/evaluate_geometry.py | tools/geometry/evaluation.py | python3 tools/evaluate_geometry.py …；实现 tools/geometry/evaluation.py |
| tools/plot_geometry.py | tools/geometry/plotting.py | python3 tools/plot_geometry.py …；实现 tools/geometry/plotting.py |
| tools/runner_core.py | tools/runner/runner_core.py | 内部 import tools.runner.runner_core |
| tools/runner_processes.py | tools/runner/runner_processes.py | 内部 import tools.runner.runner_processes |
| tools/runner_analysis.py | tools/runner/runner_analysis.py | 内部 import tools.runner.runner_analysis |
| tools/runner_faults.py | tools/runner/runner_faults.py | 内部 import tools.runner.runner_faults |
| tools/runner_gui.py | tools/runner/runner_gui.py | 内部 import tools.runner.runner_gui |
| tools/failure_stop.py | tools/runner/failure_stop.py | 内部 import tools.runner.failure_stop |
| tools/inspect_bag.py | tools/runner/inspect_bag.py | Runner启动 tools/runner/inspect_bag.py；内部 import tools.runner.inspect_bag |
| tools/gui_scene_probe.py | tools/runner/gui_scene_probe.py | Runner启动 tools/runner/gui_scene_probe.py；内部 import tools.runner.gui_scene_probe |
| tools/nav_goal.py | tools/navigation/nav_goal.py | Runner启动 tools/navigation/nav_goal.py；内部 import tools.navigation.nav_goal |
| tools/runner_dynamic.py | tools/dynamic/runner_dynamic.py | 内部 import tools.dynamic.runner_dynamic |
| tools/dynamic_geometry.py | tools/dynamic/dynamic_geometry.py | 内部 import tools.dynamic.dynamic_geometry |
| tools/inspect_dynamic_recording.py | tools/dynamic/inspect_dynamic_recording.py | Runner启动 tools/dynamic/inspect_dynamic_recording.py；内部 import tools.dynamic.inspect_dynamic_recording |
| tools/dynamic_contract.py | tools/contracts/dynamic_contract.py | 内部 import tools.contracts.dynamic_contract |
| tools/task_contract.py | tools/contracts/task_contract.py | 内部 import tools.contracts.task_contract |
| tools/batch_core.py | tools/batch/batch_core.py | 内部 import tools.batch.batch_core |
| tools/geometry_inputs.py | tools/geometry/geometry_inputs.py | 内部 import tools.geometry.geometry_inputs |
| tools/geometry_metrics.py | tools/geometry/geometry_metrics.py | 内部 import tools.geometry.geometry_metrics |
| tools/prepare_dynamic_probe.py | tools/dynamic/probe_preparation.py | python3 tools/stages/prepare_dynamic_probe.py …；实现 tools/dynamic/probe_preparation.py |
| tools/make_baselines.py | tools/navigation/baselines.py | python3 tools/stages/make_baselines.py（显式执行会写配置，本次未执行）；实现 tools/navigation/baselines.py |
| tools/p0_03_navigation.py | tools/navigation/manual_navigation.py | python3 tools/stages/p0_03_navigation.py …；实现 tools/navigation/manual_navigation.py |
| tools/p0_06_host_trial.py | tools/dynamic/host_trial.py | python3 tools/stages/p0_06_host_trial.py …；实现 tools/dynamic/host_trial.py |
| tools/p0_03_env.bash | tools/stages/p0_03_env.bash | source tools/stages/p0_03_env.bash |

上一版已迁移到功能目录的实现保留在对应目录；其旧根兼容层已移除。runner/archive.py是新增归档辅助模块。历史日志、冻结输入与旧验收文档保留原路径文字，以上表作为对照，不改写当时记录。

## 依赖与直接运行

- run_trial → runner.trial → runner_* / contracts.validation / dynamic；worker位于navigation/nav_goal.py。
- run_trial_observe → runner.observe → 同一对象runner.trial与runner.runner_gui。关闭GUI直接委托原生命周期。
- 批次入口 → batch.matrix / summary → batch_core及同一Runner入口。
- 几何入口 → geometry.spec_preparation / evaluation / plotting → geometry_inputs / geometry_metrics、contracts.validation。
- stages只承接阶段入口，公共实现归navigation或dynamic。Mac宿主入口仍由dynamic.host_trial按原SSH方式调用Ubuntu Runner，参数和负载机制未变，本次没有执行宿主实验。

统一使用tools.*包导入。tools/__init__.py集中提供包bootstrap与PROJECT_ROOT；直接文件入口通过runpy调用该bootstrap，不在各文件修改sys.path，不需要手工PYTHONPATH。资源相对PROJECT_ROOT定位，支持仓库外通过绝对路径调用。导入make_baselines及测试发现不写YAML；写入仅在显式main执行，本次未重新生成基线。

## 常用命令

~~~bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
python3 -B tools/run_trial.py --help
python3 -B tools/run_trial_observe.py --help
python3 -B tools/run_smoke_matrix.py --help
python3 -B tools/validate_run.py --help
python3 -B tools/summarize_runs.py --help
python3 -B tools/prepare_geometry_spec.py --help
python3 -B tools/evaluate_geometry.py --help
python3 -B tools/plot_geometry.py --help
~~~

validate_run --help保持原退出码2，其余上述help退出0，不以统一退出码重写接口。

只读静态预检示例（不会创建Run或启动进程）：

~~~bash
python3 -B tools/run_trial.py \
  --scenario /home/robot/commercial_mobile_robot/experiments/scenarios/p0_long_route_static_v1.yaml \
  --profile /home/robot/commercial_mobile_robot/experiments/profiles/p0_long_route_static_v1.yaml \
  --method mppi --run-id tools-v2-check-only --check
~~~

仓库外调用使用绝对路径，例如：

~~~bash
cd /tmp
python3 -B /home/robot/commercial_mobile_robot/tools/evaluate_geometry.py --help
source /home/robot/commercial_mobile_robot/tools/stages/p0_03_env.bash
~~~

阶段入口的新调用路径如下；仅展示形式，本次未执行配置生成、导航、探测或宿主负载：

~~~text
python3 tools/stages/prepare_dynamic_probe.py <原参数>
python3 tools/stages/make_baselines.py
python3 tools/stages/p0_03_navigation.py <原参数>
python3 tools/stages/p0_06_host_trial.py <原参数>
source tools/stages/p0_03_env.bash
~~~

## 冻结依赖与新归档

历史Scenario/Profile、依赖哈希、实验记录和验收报告不改。旧动态输入绑定已移出的源码路径，本来已有源码SHA失配，现在仍拒绝，不能继续当作当前可运行输入。

experiments/tool_layout_v2新增3组独立Scenario/Profile/依赖清单及layout.json。它们保存旧/新路径与SHA对应关系，冻结当前完整Python包；目标、轨迹、几何、控制与时间预算保持。现有二进制实际SHA与旧清单不同的项目也如实记录，未改二进制。三组均为draft，review/probe=null；本次未真实探测，不宣称动态配置已获集成验收。后续须从新版本模板按原准备流程生成新输入，不能修改旧哈希或跳过漂移校验。详见该目录README。

新Runner归档使用artifacts/tools/完整包、artifacts/experiments/schemas/及artifacts/src/cmr_bringup/config/，code-layout.json记录布局版本、入口和逐文件SHA。Nav2 worker、录包/动态检查器和GUI场景探针从这份包启动，不再平铺复制旧顶层列表。新评价指纹扫描整个Python包；旧批次的漂移检查仍保留，不应绕过检查恢复旧cohort。

副本脱离源码目录后已验证包导入、CLI、配置/Schema定位、fixture合同检查和指定历史真实Run的几何--check。系统ROS/Gazebo/原生库、Run原始数据仍是外部依赖；这不是独立部署包，也未证明真实Runner/GUI运行。

## 离线验证与证据

证据目录：evidence/env/tools-v2-0c15527。保留迁移前Git状态、源文件备份及34064个文件指纹，未覆盖先前工作。

| 指标 | 迁移前 | 迁移后 |
|---|---:|---:|
| pytest完整发现 | 493 | 507 |
| 默认执行 | 462 | 476 |
| 通过 | 454 | 468 |
| 失败 | 3 | 3 |
| 跳过 | 5 | 5 |
| ROS标记未执行 | 31 | 31 |
| 目录/归档结构测试 | 10 | 24 |

旧10项兼容结构测试替换为24项新布局/归档验证；其余452项业务用例保持444通过、3失败、5跳过。既有失败为tests/runner/test_runner_gui.py的三个dynamic参数预检；迁移前SHA失配、迁移后旧冻结源码路径缺失，均正确拒绝，未降低断言或增加跳过。默认命令仍会退出1。

另以显式两个历史真实Run启用几何离线集合：57通过；未生成正式评价。14个受支持CLI的help/错误参数退出行为相同，仅p0_03_navigation帮助文字中的阶段路径按新位置更新。没有盲目执行make_baselines或gui_scene_probe --help。

AST复核202个原函数体在导入标准化后相同；10个函数仅有已审查的入口/资源/归档路径变化，原函数签名一致。make_baselines的原参数字典一致，增加显式执行边界。未改数学算法、目标、参数、阈值、停止或清理策略。

~~~bash
source /opt/ros/jazzy/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m pytest -p no:cacheprovider tests -q
python3 -B -m pytest -p no:cacheprovider tests/contracts/test_tool_layout.py -q
~~~

真实ROS协议、Runner完整生命周期、GUI、Mac宿主负载与新动态模板探测均待集成验证。本次没有启动Gazebo/Nav2、发送目标、构建、commit或push，也不追认P0验收。

# P2-01 实施绑定

2026-11-16（Asia/Shanghai）。权威源码在 `cmr-ubuntu:/home/robot/commercial_mobile_robot`。
本次基线 `da9cc1dca496a74851e4ccf030bc8646b79437cd`，分支 main。指纹、状态、既有差异和 Ubuntu 实际系统时钟保存在
`evidence/p2/p2-01/p201_20261116_05e207bb/r0/`。文件指纹按读取顺序采集。根目录父链及 src、tools、tests、docs、experiments 未发现适用 AGENTS.md。

规划审查引用《Phase_2_仓库只读审查记录.md》（P2-PLAN-2.0，2026-11-14 修订包）和原审查证据 01–14。
当前 HEAD 比规划审查时多三个提交。四个既有 dirty 文件分属 NavigationAdapter 的声明、实现、Mission 调用处和 execution_port_test，
涉及关闭方法改名。当前状态以 r0 下的差异为准。

| 合同 | 当前入口 |
|---|---|
| 唯一仿真进程所有者 | `tools/run_trial.py` → `tools/runner/trial.py`，ProjectLock 和 ProcessSupervisor |
| 运动授权 | `tools/execution/profile.py` 校验 Gate 与 bridge 路由，唯一路径 final → DiffDrive drive |
| 任务入口 | `tools/delivery/profile.py` 版本 4，Mission NavigationAdapter 发 Goal，ExecutionPort 管 lease |
| 时钟 authority | `src/cmr_execution/src/gate_node.cpp` 发布 GateState，镜像 `identity.run_id`、`identity.gate_epoch` 和 `clock_epoch`。Gate/Observer 显式用 steady 和 clock，use_sim_time=false |
| 停止证据 | `docs/contracts/p1-stop-evidence-v1.md`，仿真独立 body odom 可供停车辅助。P2 不取得执行权限 |
| 当前基线验证 | 独立 build/install 与 CTest 在 r0 目录及 logs 下。E01–E16 `experiments/mission/execution-sink-v1.yaml`，D01–D18 delivery CLI，L2 由 prepare_delivery_trials 派生 |

现有包为 cmr_core、cmr_interfaces、cmr_execution、cmr_mission、cmr_sim。cmr_bringup 是配置目录。
安装包清单见 r0/environment.txt。TB3 Waffle 仿真资产来自 nav2_minimal_tb3_sim 1.0.1。
robot_localization 3.8.3 及 ros_gz_bridge 1.0.24，未更改依赖。

P2 实施和复现入口：

- `src/cmr_tracking/` 的 CMake standalone 构建仅依赖 Eigen3 和 GTest，显式用 `-DCMR_TRACKING_WITH_ROS=OFF`。ROS 构建打开该选项并构建 interfaces 和 tracking_ros。
- `tools/p2/run_case.py --help` 列出当前用例。先用同一参数加 `--check` 检查 manifest 和新输出目录，再执行。受控仿真复用原 Runner 与 G01D Mission/Gate 授权链。
- `tools/p2/audit_bag.py --help` 列出只读录包核验入口。用于时间计算的 recorded-clock 模式须显式选择，历史 bag 从未重写。
- ROS pytest 用 `-o addopts= -m ""`，选择精确 `--case` 并指定 `--manifest`。普通 pytest 保持之前的 ROS 用例排除规则。
- 本轮保留用户修改的证明在 `r0/preserved-user-dirty-files.json`。Mission 单独增加健康拒绝瞬间诊断日志，基线原有关闭方法改名仍完整。原授权、健康、停止时间窗口及判据沿用。
- 完整 Python 回归的四项历史冻结输入失败，在基线源码副本复现。旧依赖锁对 `tools/navigation/nav_goal.py` 等文件的 SHA256 与当前环境不符，旧动态配置依赖的 `tools/batch_core.py` 路径缺失。同一旧动态 profile 的 driver 和 capture 二进制实际存在且哈希匹配。记录见 `r0/python-baseline-cause.json` 和 `logs/p1-baseline-copy-regression-02.log`。

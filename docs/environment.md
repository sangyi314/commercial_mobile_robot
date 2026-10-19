# 环境及证据索引

本次从 macOS 宿主通过已有 cmr-ubuntu SSH 入口实际访问 Ubuntu；所有编译测试均在 Ubuntu 执行。

| 项目 | 当前核验 |
|---|---|
| 用户 / HOME | robot / /home/robot，来自 id 与 HOME 实测 |
| 工作区 / Git 根 | /home/robot/commercial_mobile_robot，两者一致 |
| Ubuntu / 架构 | 24.04.5 LTS / aarch64 |
| VM CPU / 内存 | 4 vCPU / free -h 显示总内存 7.7 GiB |
| 磁盘 | /dev/vda1，96 GiB，总体约 86 GiB 可用（审计时） |
| ROS | Jazzy，构建只加载 /opt/ros/jazzy/setup.bash |
| ROS_DOMAIN_ID | 从现有进程环境确认 71；本次构建进程保留 71 |
| 宿主 | macOS / arm64；具体芯片、内存、UTM 版本本次未重新核验，待确认 |
| GPU / 渲染 | 本次未运行图形、Gazebo 或 Nav2；GPU 与实时因子本次未验证 |
| C++ 工具 | GNU 13.3.0、CMake 3.28.3、Ninja 1.11.1；既有 ament 可用；新版不要求 GTest |

最新本次证据目录由 evidence/env/section4-latest-run.txt 指定，内有 os.txt、uname.txt、cpu.txt、memory.txt、disk.txt、ros_packages.tsv、gazebo_vendor_packages.tsv、ros2_doctor.txt、build-environment.txt 与构建测试日志。Gazebo 由 ROS vendor 包提供，直接查询 gz-*/libgz-* 无匹配，见 gazebo-package-note.txt；未因此安装或升级软件。ros2 doctor 为环境诊断，不构成导航或通信集成通过。

旧 evidence/env/migration-Uz9dGuoc/、experiments/manifests/ 下的 environment.json、ros2_doctor.txt 与 tb3_launch_args.txt 原样保留；其历史结论本次未验证。

原 build/install/log 缓存未删除或复用。本次使用带 section4 唯一编号的专用目录。独立构建在 env -i 中进行，并使用 CMAKE_DISABLE_FIND_PACKAGE_ament_cmake=TRUE，证明无 ament 仍可构建测试；ROS 构建仅使用系统 Jazzy underlay，未 source 旧工作区 install。不要为本项目加载其他工作区 overlay，也不修改系统或 shell 启动配置。

已知边界：核心无 ROS 依赖；CommandGate 只判断到达时是否可接受，不主动停车，不处理会话重启、跨机器时间、共享线程或轮速反馈。设备间 NTP、GPU、真实底盘与安全停车本次未验证。

# 第三方来源
Nav2：官方 Jazzy 包与配置；保留原始文件、版本、hash 和原许可证。
起步 C++ 策略来自本项目执行指南，属于待审查的协作实现。
后续增加代码复用时逐项记录来源、许可证、修改和贡献边界。


## 2026-10-31 — P1 设计参考（未复制代码）

- Nav2 jazzy commit d5d768b7b38a05e7cfd2b6c693b786b402161bed：nav2_waypoint_follower/src/waypoint_follower.cpp、nav2_collision_monitor/src/collision_monitor_node.cpp；Apache-2.0；参考Action身份、生命周期与速度输出边界。
- YASMIN main commit abbe58d9511a79618023a530870f7d1794a829b7：yasmin/src/yasmin/state_machine.cpp、yasmin_ros/include/yasmin_ros/action_state.hpp；Apache-2.0；参考状态合法性和回调身份/生存期。
- twist_mux rolling commit a8225e2d9bf0171aac447c246376f0981defd17b：src/twist_mux.cpp、include/twist_mux/topic_handle.hpp；BSD-3-Clause；参考锁/优先级/超时，未引入运行依赖。
- SQLite官方C API/事务文档，核心public domain；计划使用Ubuntu已安装3.45.1，目前未链接或实现Store。

上述源码经GitHub读取并按commit复核blob一致；没有编译上游，不推定与本机Nav2二进制逐字一致。此次cmr_mission代码为独立协作实现，SPDX MIT与根许可证一致；cmr_core既有Apache元数据差异未修改。研究详表位于项目管理deliverables/Phase_1_开源源码研究_20261031.md。


## P1-01 S2–S6实现分类（2026-10-31）

直接链接：Ubuntu系统SQLite 3.45.1 C API（核心public domain；未拷贝amalgamation）。接口依据 https://www.sqlite.org/lang_transaction.html 、https://www.sqlite.org/c3ref/prepare.html 、https://www.sqlite.org/c3ref/step.html 。

参考设计后独立实现：mission.cpp参考先前固定YASMIN abbe58d9511a79618023a530870f7d1794a829b7 的yasmin/src/yasmin/state_machine.cpp状态合法性与分层思路，本轮经GitHub连接再次读取；未复制源码，未引入框架、全局信号处理或黑板。原YASMIN Apache-2.0身份保留在研究记录。

独立实现：CMR事务/活动槽/意图语义、RAII封装、Runtime、离线Port、C++/Python测试驱动与CLI，SPDX MIT。标准库线程/condition_variable和CMake/CTest为基础设施调用。Nav2/twist_mux仅保留研究参考，没有链接到此纯离线模块。

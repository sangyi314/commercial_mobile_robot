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


## 2026-10-31 — P1-02来源与复用分类

直接使用系统公共组件：Jazzy rclcpp/rclcpp_action 28.1.22（Apache-2.0）、nav2_msgs 1.3.13及rosidl生成工具/运行库；SQLite沿用系统3.45.1。没有拷贝第三方实现文件。新cmr_interfaces定义和CMR节点/适配/测试为MIT协作实现；不更改P0元数据。

参考设计后独立实现：Nav2 waypoint_follower.cpp，commit d5d768b7b38a05e7cfd2b6c693b786b402161bed（Apache-2.0），借鉴Goal UUID和资源管理，不照搬cancel_all；YASMIN action_state.hpp，abbe58d9511a79618023a530870f7d1794a829b7（Apache-2.0），借鉴晚接受/epoch/弱生命周期，不引入框架。两份本轮重读blob与既有研究一致。

ROS2官方测试参考：ros2/rclcpp 7c76a00345f304c7dd145975b872f7ec66a92c9d，rclcpp_action/test/test_client.cpp，blob4371f48a0e46537a4d8bd215d8c6833b620993de，Apache-2.0。实际接口以本机client.hpp/create_client.hpp/any_service_callback.hpp为准，已保存头文件指纹。固定上游源码不作为本机二进制逐字证明。

独立实现：MissionCommand延后响应、ActionCorrelation、CMR Port队列/单调期限、Fake控制屏障、Python监督器与独立审计器；未增加Waypoint Follower/YASMIN运行依赖。详见docs/design/p1-action-design.md。

P1-02测试监督器还直接调用系统rclpy与rosidl_runtime_py消息转换公共API，并使用rosgraph_msgs/Clock合成时钟；本机rclpy/client.py保留上游Apache-2.0声明，remove_pending_request仅作为公开接口调用，不复制实现。静态预检路径不导入这些ROS运行库。

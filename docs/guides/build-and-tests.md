# 构建、环境与分层测试

实际工作区为Ubuntu的/home/robot/commercial_mobile_robot。技术栈与已有版本记录见[项目README](../../README.md#技术栈)，历史机器/工具测量见[环境快照](../history/p0-01-environment.md)；旧磁盘余量、GPU及包版本不作为当前实时测量。

依赖ROS 2 Jazzy、Nav2、Gazebo Harmonic、ros_gz、TurtleBot3资源、MCAP插件及既有Python依赖。动态原生程序还依赖已安装cmr_sim。本项目不是clone-and-run发行包：Profile含绝对路径和冻结资源，evidence默认不入Git；不能手改历史哈希完成迁移。

## 核心独立构建

以下沿用既有CMake方式，每次新建目录；本次文档重构未构建或运行CTest：

~~~bash
cd /home/robot/commercial_mobile_robot
CMR_CHECK_ID="core-check-$(date -u +%Y%m%dT%H%M%SZ)-$$"
cmake -S src/cmr_core -B "build/$CMR_CHECK_ID-standalone" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=TRUE
cmake --build "build/$CMR_CHECK_ID-standalone"
ctest --test-dir "build/$CMR_CHECK_ID-standalone" --output-on-failure --no-tests=error
~~~

cmr_core当前采用独立CTest五组，不需要为了文档重构新增GTest依赖。ament/colcon、安装消费者与历史结果见[测试说明](../../tests/README.md)和[核心交付/更正](../verification/section4-core-20261020.md)。不要覆盖旧build/install/log或把旧安装当新源码已构建的证据。

## 离线与ROS测试边界

当前命令和分组目录以[tests/README](../../tests/README.md)为唯一维护入口。默认pytest排除ros_contract；这些用例会创建节点，不能为检查文档而开启。纯离线集合包含短寿命自有假进程及本地文件操作，不等于真实ROS/GUI集成。

工具布局v2的既有基线为468 passed、3 failed、5 skipped，31项ROS未执行；三个失败由旧冻结动态源码路径导致，本次不改测试或冻结输入。具体日期、比较和本次结果见[文档迁移报告](../verification/docs-layout-20261030.md)，历史轮次不相互替代。

导航会话的DDS/environment入口为tools/stages/p0_03_env.bash；实际导航前按[Runner指南](runner.md)检查。不要加载无关overlay或修改系统shell初始化文件。

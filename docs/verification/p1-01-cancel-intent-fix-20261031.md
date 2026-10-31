# P1-01 首次终止意图覆盖修复（2026-10-31）

结论：P1-02前置审查发现的Cancel→ABORTED→Recover→Retry错误重发已修复。此报告只确认该缺陷及回归结果，不等同于P1-02全部前置与协议验收完成。

## 改动与原理

src/cmr_mission/src/mission.cpp的pause辅助函数：仅在pending为空时设置pending/reason。已提交的业务终止意图不再被后到导航失败覆盖；仍处理goal_terminal、撤销effect和停止收尾，Store::apply仍按原逻辑记录ABORTED/REJECTED事件。

调用关系：Runtime有界邮箱→owner线程Store::apply→reduce→pause→状态/原始事件/effects统一事务提交。没有改变Mission合同、数据库schema或S1。

取消先提交：后到失败不改变CANCELED，Recover/Retry拒绝；失败先提交：后到取消不覆盖FAILED，符合原有白名单/停止/预算的恢复仍允许。修复不是把所有失败恢复关闭。

新增test/termination_intent_test.cpp三组：

- logic：Cancel后ABORTED/REJECTED/SUCCEEDED；Fatal/Tick不覆盖；Restart保留取消；终态CANCELED；失败先提交的恢复资格仍有效。
- store：通过真实Runtime/SQLite/独立Sink验证Send仅一次、重启保留pending、目标终态/合成停止后CANCELED；独立只读SQL确认原ABORTED/REJECTED事件各落库一次。
- races：条件变量屏障控制两个生产者，将Cancel与ABORTED/REJECTED各按两种顺序入队，检验owner事务顺序、恢复资格及无第二Send。

CMake注册3个命名组。AI_USAGE和docs索引仅追加说明。此次代码为针对现有CMR合同的独立修复，未复制/新增第三方实现或依赖；THIRD_PARTY无需变更。

## 红灯与绿灯证据

目录：/home/robot/commercial_mobile_robot/evidence/env/p1-01-cancel-fix-wu4hdtuc

| 检查 | 实际结果 | 证据 |
|---|---|---|
| 修复前新增回归 | 0/3通过、3失败，CTest exit8 | red-test.log、before-commands.json |
| 修复后Release全P1-01 | 16/16、0失败/跳过 | release-test.log、after-commands.json |
| 修复后ASan/UBSan | 16/16、0失败/跳过，无非预期诊断 | sanitize-test.log |
| 原样独立反例重新链接固定库 | exit0；pending保持CANCELED；Send=1 | probe-run.log、probe-build.log |

包含原13组P1-01测试、八个崩溃窗口、20seed×200事件、原Runtime并发测试，以及新增3组。新增测试先在未修复核心上失败，原失败证据未覆盖。没有通过放宽合同或删除断言取得绿灯。

原反例来源保持只读：evidence/env/p1-02-precheck-tjk8bdyk/cancel_abort_probe.cpp；重新编译输出和新DB均在本轮目录，原失败DB/日志不改。已经在旧代码下错误变为FAILED或已经重试的历史DB不会由这次代码变更自动回填；没有执行任何数据迁移或历史清理，若要继续使用此类DB需单独核对。

## 复跑

在Ubuntu仓库新建构建目录，避免旧测试DB混入：

```bash
cd /home/robot/commercial_mobile_robot
CMR_CANCEL_BUILD=$(mktemp -d /tmp/cmr-cancel-fix-XXXXXX)
cmake -S src/cmr_mission -B "$CMR_CANCEL_BUILD" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$CMR_CANCEL_BUILD" --parallel 2
ctest --test-dir "$CMR_CANCEL_BUILD" --output-on-failure --no-tests=error
```

预期16组。只查新增组可使用ctest的-R mission_termination_，但不要在同一已运行目录重复使用持久化测试DB；每次使用新构建目录。

## 保护与范围

起始HEAD=6ab9bb15bff4677372e966e42f7d141f3b7322df，起始工作树干净；最终HEAD未变，无commit/push/tag。protection.json记录起点全部跟踪文件指纹比较；仅预定FSM/CMake/文档追加变化，S1、Store、Runtime及P0源码/冻结输入未变。没有运行P0仿真，本轮未额外重跑cmr_core；纯核心逻辑改变的回归由完整P1-01集合覆盖。

无ROS/Gazebo/Bridge/Nav2启动，无真实目标；停止证据均synthetic，TSan及实际停车未验证。先前阶段报告保留，本报告与前置失败报告共同表达当前修正结果。

## 必须掌握的五个问题

1. pending和导航结果有什么不同？——pending是业务已提交的终止决定；ABORTED是导航事实，不能互相覆盖。
2. 为何修pause而非Recover？——在根源维护首次意图，避免同类失败路径把错误决定写入持久化。
3. 如何证明不是隐藏失败结果？——独立SQL检查events仍存在原ABORTED/REJECTED记录。
4. 为何要测两种事务顺序？——第一次提交决定语义，失败先提交和取消先提交应有不同恢复资格。
5. 是否能直接启动02？——此已知阻塞已解除，但仍须继续02任务书的其余前置/API/设计检查，不能据此宣称ROS协议已通过。

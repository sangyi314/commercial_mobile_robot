# P1-01 离线运行与解释

2026-10-31。仅合成任务事件，不创建ROS节点。任务Record/Port接口是P1-02接入之前的离线合同；站点在本阶段为固定标识/版本，真实pose解析和地图运行资格在后续适配时补齐。

## 新目录构建

在Ubuntu /home/robot/commercial_mobile_robot中逐条执行，失败保留目录：

```bash
CMR_OFFLINE_BUILD=$(mktemp -d /tmp/cmr-offline-build-XXXXXX)
cmake -S src/cmr_mission -B "$CMR_OFFLINE_BUILD" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$CMR_OFFLINE_BUILD" --parallel 2
ctest --test-dir "$CMR_OFFLINE_BUILD" -N
ctest --test-dir "$CMR_OFFLINE_BUILD" --output-on-failure --no-tests=error
```

预期13个CTest组；一个组含多个反例，不能称为12个单独场景。测试文件/DB保存在该build目录，每次完整重跑使用新build目录，避免旧DB影响测试。

## 演示

```bash
CMR_OFFLINE_DEMO=$(mktemp -d /tmp/cmr-offline-demo-XXXXXX)
"$CMR_OFFLINE_BUILD/mission_probe" --help
"$CMR_OFFLINE_BUILD/mission_probe" "$CMR_OFFLINE_DEMO/normal.db" normal
"$CMR_OFFLINE_BUILD/mission_probe" "$CMR_OFFLINE_DEMO/cancel.db" cancel
"$CMR_OFFLINE_BUILD/mission_probe" "$CMR_OFFLINE_DEMO/normal.db" inspect
```

normal输出duplicate=2/Existing、conflict=3/Conflict、busy=4/Busy，随后完整去返程COMPLETED。cancel输出CancelAck后仍PAUSED/held=1，补相关终态和停止后CANCELED。normal/cancel要求新DB；inspect用于重开现有记录，非终态会按恢复政策暂停，因此不是无副作用的只读查看器。它不会自动重发运动。

Crash演示由CTest的mission_crash_windows完成，原始目录crash-artifacts内每个窗口有receiver.log（未发生动作时可以不存在）、child stdout/stderr、recovery stdout/stderr、database.json；results.json汇总。不要手动kill未知进程。

## 重放

```bash
"$CMR_OFFLINE_BUILD/interleaving_test" "$CMR_OFFLINE_DEMO/replay" 7 "$CMR_OFFLINE_BUILD/interleaving-artifacts/seed-7.events"
```

预期PASS；保存重放DB、events、trace、receiver。使用同一构建版本。失败保留原序列，不替换种子或阈值。

## 必须理解

- Admission是纯判断，Store事务提交才成为可靠接纳。
- effect是持久数据，dispatch先记录派发状态再调用Port；中间崩溃仍未知，不能Exactly Once。
- Runtime只有一个写owner；SQLite写锁不是整个任务系统的唯一所有权。
- CancelAck不改变业务为CANCELED；目标已终态与停止证据都满足才Resolve。
- 重开非终态变PAUSED，旧意图UNCERTAIN，不恢复旧期限后自动继续。

已实现的时钟/停止均为离线注入，默认deadline为测试合同的固定值。真实时钟、站点pose、NavigationPort/ExecutionPort的ROS实现、持续授权和真实停止不在此验证范围。

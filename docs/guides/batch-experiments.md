# 批次调度与汇总

输入、矩阵和历史cohort统一见[场景目录](../scenario_catalog.md)。调度器只串行调用原Runner，不直接操作ROS节点或重写result。预算、资格、故障和恢复规则见[P0-08冻结合同](../contracts/p0-08-contract.md)。

## 新批次预检

源码变化后必须新cohort；旧源码清单不匹配时不能恢复旧批次。以下仅--check，不创建批次、锁、节点或子进程：

~~~bash
cd /home/robot/commercial_mobile_robot
source /opt/ros/jazzy/setup.bash
CMR_BATCH="$PWD/evidence/batches/normal-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B tools/run_smoke_matrix.py \
  --matrix experiments/p0_smoke_matrix.yaml --batch-dir "$CMR_BATCH" --check
~~~

实际派发会启动仿真，按需移除--check并指定--limit；本次未执行。故障矩阵与正常分母分开，显式选择场景目录中的fault矩阵，不能把故障预期退出3计为普通导航成功。

## 续跑边界

--resume指向已有批次，先--check；代码、Profile、资源与journal必须匹配。--limit限制本次新增调用，不重跑已完成槽位。失败后按真实归属/关闭/原始证据审查，必要时用具体--resume-reason记录新attempt；未决身份、截断journal或重复首因停止时不能盲重试。

不删除锁、journal或失败Run绕过检查；不把旧成功拼入新cohort。历史报告中的resume命令只描述当时版本，当前源码可能按设计报cohort drift，不能改原Manifest让它通过。

## 只读汇总

指定明确批次，新输出目录必须不存在。汇总写新分析，不修改原Run或账本：

~~~bash
CMR_SUMMARY="$PWD/evidence/env/batch-summary-$(date -u +%Y%m%dT%H%M%SZ)-$$"
python3 -B tools/summarize_runs.py \
  --batch-dir "$CMR_BATCH" --output-dir "$CMR_SUMMARY"
~~~

计划、意图、实际启动、冷启动、READY、accepted、Action终态、完整通过与未决分别查看。ledger和目录双向对账；恢复后推进时钟中的停车与暂停静止不能混称。

已存在的长路线报告在本地真实存在：[P0-08报告](../verification/p0-08-long-20261029-3cff94.md)，保留正常失败及补采分母。旧开发批次说明见[历史指南](../history/p0-08-guide-20261030.md)；F001/F002见[故障索引](../README.md#故障排查)。

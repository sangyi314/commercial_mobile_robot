# P1-04 G01：本人实际交接操作

2026-11-03。本轮用户说明暂时不能本人操作，所以**G01未执行**，AI没有代发human确认。以下是唯一尚需本人完成的交接验证流程；不是已通过证明。实际仓库在Ubuntu，Mac只通过ssh访问。

## 终端1：先只读核对，再启动原Runner

```bash
ssh cmr-ubuntu
cd /home/robot/commercial_mobile_robot
source /home/robot/commercial_mobile_robot/evidence/env/p1-04-stopfix-ds6qdezr/g01-user-env-final.bash
python3 -B tools/run_trial.py --mission \
  --scenario /home/robot/commercial_mobile_robot/evidence/env/p1-04-stopfix-ds6qdezr/inputs-r10/G01/scenario.json \
  --profile /home/robot/commercial_mobile_robot/evidence/env/p1-04-stopfix-ds6qdezr/inputs-r10/G01/profile.json \
  --method mppi --run-id p104_20261103_g01_r10 --check
```

`--check`不启动Node、数据库或仿真。必须通过，且run目录尚不存在；当前没有创建G01运行目录。若二进制hash已改变，不能手动改hash绕过检查，应重新生成新独立profile和唯一run_id。

准备好本人确认后，在同一终端执行：

```bash
python3 -B tools/run_trial.py --mission \
  --scenario /home/robot/commercial_mobile_robot/evidence/env/p1-04-stopfix-ds6qdezr/inputs-r10/G01/scenario.json \
  --profile /home/robot/commercial_mobile_robot/evidence/env/p1-04-stopfix-ds6qdezr/inputs-r10/G01/profile.json \
  --method mppi --run-id p104_20261103_g01_r10
```

原Runner拥有全部进程、冷启动和清理。此配置明确是**官方Rotation Shim→MPPI路径跟踪 + Oracle辅助停车资格**；没有把body真值接入Nav2定位/规划。不是裸MPPI算法比较或实车证明。不要另外启动Gazebo、Nav2或第二Goal发送者。

## 终端2：只在真实WAIT_HANDOVER后本人确认

打开另一个Ubuntu终端：

```bash
ssh cmr-ubuntu
cd /home/robot/commercial_mobile_robot
source /home/robot/commercial_mobile_robot/evidence/env/p1-04-stopfix-ds6qdezr/g01-user-env-final.bash
```

终端1到站并取得到达/停稳证据后，会打印`HUMAN_HANDOVER_REQUIRED`，并保存：

`/home/robot/commercial_mobile_robot/evidence/missions/p104_20261103_g01_r10/runtime/human-handover-required.json`

其中`command_after_sourcing_install_and_domain71`是本轮准确命令，包含当前mission_id、handover_id、entry_version及`--source human`。**本人阅读后复制该命令，在终端2手动执行一次。**任务书要求本人操作，因此本文不预造token、不提供自动查询并确认脚本，也不由AI代调用该服务。

交接期限从WAIT_HANDOVER事务进入时开始，共120秒。超时会安全失败收尾，不自动确认或返航；旧token/旧entry_version不能复用，也不能为重试重置期限。确认成功后真实Mission提交交接事实和返航意图，自动执行返航，用户不另发NavigateToPose Goal。

## 完成后独立验收

等待终端1完整退出、录制flush与归属清理结束，再运行：

```bash
python3 -B tools/mission/audit_delivery.py \
  --session evidence/missions/p104_20261103_g01_r10 \
  --output evidence/env/p1-04-stopfix-ds6qdezr/g01-r10-human-independent-audit.json
```

必须AUDIT_PASS/exit0，并核对mission-result中COMPLETED、handed_over=true、handover_source=human，两段不同UUID/lease、到达、body停止、harness及cleanup同时满足。输出文件拒绝覆盖，重复审计另用新名称。若PAUSED/FAILED或任何证据不足，保留该目录与日志，不把请求已发送等同于任务完成。

旧终端状态中的`action=NOT_SENT`指观察worker没有发送Goal；真实导航Goal由Mission持有，见mission.db、NAV_DISPATCH和Action状态原始记录。它不是Mission没导航的结论。

G01通过后才可补完整P1-04验收结论，仍停在P1-04，不自动进入P1-05。

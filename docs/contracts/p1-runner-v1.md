# P1 Runner v1 — 配送集成合同

2026-11-02；P1-04实施合同，设计不是运行证明。任务书/现有Mission、Action、Execution与Stop合同共同约束；本阶段状态见verification/p1-04-sdkcx3w5.md。

## 两模式与所有权

原tools/run_trial.py保留P0默认模式；新增显式Mission模式，必须与新版本派生profile匹配，不能靠猜参数开启。Runner仍独占Gazebo/Bridge/Nav2/Gate/Mission/录包/worker进程监督、启动协调与归属关闭。worker观察模式不创建ActionClient，并在命令分派处拒绝send/cancel及P0故障注入入口。Mission经既有NavigationAdapter独占业务Goal与精确取消；真实ExecutionPort独占租约；不运行owner_fixture。只读观察者不制造运动证据。

Goal动作、SQLite业务事务、Gate控制和外部执行端各有所有者，绝无跨DB/DDS原子提交承诺。接受未知、旧UUID未决、D未知不重发不放槽。Supervisor失联不另开取消客户端，先等待Gate租约/独立停止，不能以关仿真冒充停车。

## 配置与期限

新增版本化站点目录和派生BT/Bridge/SDF/profile；保存地图yaml/image/元数据、站点、BT和源码/ELF的实际hash。Map身份是资源内容，不仅字符串。不改P0输入或历史evidence。真实模式Domain71，Fake+synthetic隔离Domain72；正式模式禁止synthetic arrival/stop。Gate/Observer仍用显式Clock和steady/wall。

规划预算（编码/实测后在profile冻结，不能追认失败）：启动180s、活动任务总期限240s、每Goal150s、交接120s（超时专案另有预先冻结配置）、会话work660s、cleanup45s、finalize45s。任务期限从激活起、去返程和重试共享且不重置；排队无自动过期。期限now==deadline过期。重启不重建旧期限并自动继续，只PAUSED。

## 站点、到达、交接

站点字段station_id/frame/position/orientation/map_version/station_version；坐标有限、单位四元数、map frame及明确边界必检。请求接纳前冻结完整坐标和资源hash；队首提升前复核资源健康。Action成功不等于到达；到达要求匹配当前attempt/clock、终态后的连续新鲜map→base定位窗、TF健康、无跳变、.25m/.25rad容差，并与当前D窗口重叠。世界真值只做离线复核。

人工交接使用独立业务服务/CLI，携带mission_id、handover_id、entry version token和source。token绑定进入WAIT_HANDOVER时的版本/本次交接，后台停止报告不改变token含义；不是任意旧state_version。相同命令内容返回首次已提交决定，即使已返航/终态；异内容冲突。第一次提交须重新取得当前A/D/到达与owner健康，处理和commit前检查steady期限。超时或取消胜出后不自动返航；响应丢失重试不重复effect。

## 恢复与记录

审查本机默认BT包含RecoveryNode6、各规划/控制局部清图1、RoundRobin清图/Spin/Wait/BackUp。新P1 BT只允许有界清图+等待一次恢复，不启用恢复Spin/BackUp。Mission全任务最多新增attempt重试1次、去返程共享，只对白名单已终止失败且撤销/停止/定位/剩余总预算健康时显式重试，取消优先。低电输入未接入标unavailable，不自动假造电量返航。

Mission会话记录独立版本：manifest/events/result和队列账本；按首次唯一mission_id及调用次数分别统计；任务、Action、到达、停车、交接来源、harness/cleanup分列。缺文件/漏Goal/错hash/假证据升级不能判通过。--check不建DB/Run/节点/进程，不做迁移。仅终止本轮有PID/start_ticks/token的进程；不启第二套Gazebo。

至少G01必须用户本人经接口确认；AI不得发送human交接。未收到则保存等待/超时并标PARTIAL。其余自动仿真交接明确synthetic，不能算本人演示。

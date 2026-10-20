# P0-02 合成合同样本（审查草稿）

所有 runs 的 manifest/result 均为 `record_origin=fixture`，commit、版本、时间、插件、参数、事件和测量都是 **synthetic**。artifacts 的文本/场景标识也明确为 synthetic。不是七次实验、不是实际 ROS 记录，不放入 evidence/runs，不累计冷启动或导航次数。

场景记录按合同没有 record_origin 字段；两个动态文件通过本 fixtures 目录和 synthetic_event_1 标明合成来源。横穿为 Master S1，已观测遮挡为 S3，legacy s2_occluded 不代表 S2。

| 目录 | 独立预期（未执行） |
|---|---|
| succeeded_unknown | VALID；SUCCEEDED/true；碰撞、配送均 null |
| nav_timeout | VALID；NAV_TIMEOUT/CANCELED/false，保留超时原因 |
| startup_failed | VALID；FAILED/NOT_SENT/null，runtime=null，仿真时刻未知 |
| collision_observed_partial | VALID；有合成事件的 true、partial，评价仍无效并保留缺测原因 |
| collision_false_scoped | VALID；full/有效/false 只指 synthetic-disc-scope，不表示全程物理安全 |
| incomplete | INCOMPLETE；只有合法 manifest 和其引用，不写 result |
| invalid_collision_false | INVALID/COLLISION_UNSUPPORTED；只从无评价样本把 null 改为 false |

默认 CLI 拒绝 fixture；`--allow-fixtures` 只解除来源限制，仍执行字段、hash、路径、身份、终态和评价规则。完整样本各有 manifest/result 与自己 artifacts 内的非空引用；manifest_sha256 绑定原始 manifest 字节，修改后测试要独立重新绑定。

S05 在 pytest 临时目录独立构造全部 synthetic 资源/锁文件/人工证据，以验证记录层 ready。其 VALID 也只说明合同满足，不证明资源是可用世界、地图或机器人。

负例从合法样本复制并单项变更；文件 hash 由测试使用 stdlib 独立计算，不调用校验器生成 oracle。测试中所有删除/修改均在 pytest 临时副本，不触碰库内样本和历史材料。本阶段不执行测试。

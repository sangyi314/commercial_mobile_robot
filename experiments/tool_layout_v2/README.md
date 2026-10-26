# tools布局v2独立输入版本

本目录新增3组Scenario/Profile/依赖锁，历史experiments/scenarios、profiles及旧依赖文件原样保留。layout.json记录旧/新路径及SHA关系，依赖锁冻结新的完整Python包。现有原生二进制的实际指纹变化如实记录于provenance；本次没有重建二进制。

三组均是draft，preparation.review/probe均为null。目标、轨迹、几何、控制与时间预算不变；禁止将它们视为已获真实运行验证的ready输入。后续须经tools/stages/prepare_dynamic_probe.py的原人工review和probe流程，生成新的独立输入；本次没有执行准备器、宿主负载、仿真或正式评价。

旧动态输入的源码位置/指纹属于历史版本，当前预检仍应拒绝。不能重写历史哈希、关闭验证或把旧探测证据移植到新版本。完整迁移和运行边界见../../tools/README.md。

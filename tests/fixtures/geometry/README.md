# 显式合成反例

这些输入均为 `record_origin=fixture`，不是仿真记录。interior 的端点间距为 1 m，线性段内部最小间距 -1 m，最近时刻 5 ns；gap 保持端点但最大间隔限值为 9 ns，不能桥接 10 ns 的原始间隔。wrong_epoch、wrong_frame、negative_radius、conflicting_duplicate 必须退出 2；missing_pose 必须退出 3。CLI 只有显式 `--allow-fixtures` 才接受这些输入，不能混入真实指标。

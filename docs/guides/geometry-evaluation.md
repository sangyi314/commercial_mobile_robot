# 指定Run的几何评价

这是离线流程：读取已关闭的明确Run，不启动ROS/Gazebo、不发送目标，不回填原result。场景和历史输入版本见[场景目录](../scenario_catalog.md)；算法语义以[评价合同](../../experiments/evaluation_contract.md)和[模型设计](../design/geometry-metrics.md)为准。

## 自动准备Concrete Spec

以下命令只写新的Spec/审查日志。显式修改CMR_RUN选择其他Run，不能自动选最新Run：

~~~bash
cd /home/robot/commercial_mobile_robot
CMR_WS="$PWD"
CMR_RUN="$CMR_WS/evidence/runs/p0-long-crossing-low-20261029-a1"
CMR_PREP="$(mktemp -d "$CMR_WS/evidence/env/geometry-spec-XXXXXXXX")"
CMR_SPEC="$CMR_PREP/spec.json"
python3 -B tools/prepare_geometry_spec.py \
  --run-dir "$CMR_RUN" --output "$CMR_SPEC" \
  2>"$CMR_PREP/prepare-review.json"
~~~

失败先停下读stderr；生成器失败退出2，不产出正式Spec。它核对manifest/result、来源hash、epoch/frame、真实实体树和归档collision几何，输出生成依据与未知项。不用当前模型冒充Run资源，不按距离挑时间范围，不自动更新Catalog；已有文件、Run内输出、符号链接输出及缺失父目录均拒绝。

## 审查与只读预检

~~~bash
cat "$CMR_PREP/prepare-review.json"
python3 -B tools/evaluate_geometry.py \
  --run-dir "$CMR_RUN" --spec "$CMR_SPEC" --check
~~~

必须传Concrete Spec；带catalog_version/specs的文件只是索引。预检退出0表示声明模型范围full，可能同时发现圆模型相交；不是物理无碰撞。

| 退出码 | 评价器语义 |
|---|---|
| 0 | 声明范围full，包括模型相交 |
| 2 | 格式、身份、hash、合同、路径或已有输出非法 |
| 3 | 必需输入不足或无可评价数据 |
| 4 | 有效子区间但声明范围partial |
| 1 | 内部异常，不算通过 |

## 按需正式评价与绘图

仅在预检和审查通过后执行。以下写新的分析目录/PNG，本次文档重构没有执行：

~~~bash
CMR_EVAL_OUT="$CMR_WS/evidence/metrics/$(basename "$CMR_RUN")/$(basename "$CMR_PREP")"
python3 -B tools/evaluate_geometry.py \
  --run-dir "$CMR_RUN" --spec "$CMR_SPEC" --output-dir "$CMR_EVAL_OUT"
python3 -B tools/plot_geometry.py \
  --report "$CMR_EVAL_OUT/report.json" --output "$CMR_EVAL_OUT/clearance.png"
~~~

目录/文件已存在会拒绝覆盖。保存spec/input/sources/intervals/report及源码与Schema指纹；原Run保持原样。直接采样、有界插值与线性段最小值含义不同，缺口不连线；unknown/null不改成0或false。半径、对齐、max_gap及误差依据读取Spec与归档资源，不在操作指南再维护一份数值配置。

旧评价结果与测试轮次见[P0-07报告](../verification/p0-07-4v9sd2n4.md)、[长路线联调](../verification/long-route-v1-20261029.md)及[历史指南](../history/p0-07-guide-20261030.md)。当前测试入口见[tests说明](../../tests/README.md)，不把fixture当真实Run或将历史通过移植到新代码。

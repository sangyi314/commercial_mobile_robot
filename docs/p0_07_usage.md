# P0-07 离线几何评价使用说明

已在 Ubuntu 24.04.5 ARM64、`/home/robot/commercial_mobile_robot` 验证。工具只读关闭归档的回合，不启动 ROS、Gazebo，不发初值/目标/速度，不改历史 result。Python 3 和现有 jsonschema 即可运行；图表另用已安装的 matplotlib。无需编译 ROS 包。

## 已验证的只读预检

```bash
cd /home/robot/commercial_mobile_robot
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11 --spec /home/robot/commercial_mobile_robot/evidence/env/p0-07-dev-4v9sd2n4/specs/p0-06-crossing-kwdqt70u-c11.json --check
```

实际退出 0，覆盖 [11.295,35.171] 仿真秒，full。此命令可以重复执行。已用整个仓库的路径、大小、mtime 和 mode 快照验证无写入；读取造成的 atime 变化不计入内容修改。

## 已实际执行的正式评价命令

```bash
cd /home/robot/commercial_mobile_robot
python3 -B tools/evaluate_geometry.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11 --spec /home/robot/commercial_mobile_robot/evidence/env/p0-07-dev-4v9sd2n4/specs/p0-06-crossing-kwdqt70u-c11.json --output-dir /home/robot/commercial_mobile_robot/evidence/metrics/p0-06-crossing-kwdqt70u-c11/p0-07-4v9sd2n4-02
```

该输出已经存在。原命令已成功执行；再次执行会按设计退出 2，不覆盖。新分析必须使用新的输出目录，不删除旧分析。相同输入对应相同冻结范围，禁止为躲开相交而缩短区间。四轮的准确已执行命令见验收目录 `evaluate-02-*.json`，不是模板占位命令。

独立冻结 spec 的已执行命令：

```bash
cd /home/robot/commercial_mobile_robot
python3 -B tools/prepare_geometry_spec.py --run-dir /home/robot/commercial_mobile_robot/evidence/runs/p0-06-crossing-kwdqt70u-c11 --output /home/robot/commercial_mobile_robot/evidence/env/p0-07-dev-4v9sd2n4/specs/p0-06-crossing-kwdqt70u-c11.json
```

它先读取唯一 accepted/terminal UUID、运动触发和冻结脚本终点以确定区间，然后记录源文件 hash、实体树、frame/epoch；不计算距离后挑区间。该文件现已存在，重复创建也拒绝。仓库 `/home/robot/commercial_mobile_robot/experiments/evaluations` 内有四份字节一致的具体 spec 与两个场景索引。索引 YAML 使用合法 JSON 子集，仅供选择准确 spec，不直接传给评价 CLI。

## 退出码与输出

| 退出码 | 含义 |
|---|---|
| 0 | 声明对象对与时间范围的圆模型评价 full；包括发现模型相交的情况 |
| 2 | 格式/身份/hash/合同非法，或输出已存在、路径逃逸 |
| 3 | 缺少必须输入或没有可评价数据；不补写 result |
| 4 | 存在有效子区间，但声明范围不完整 |
| 1 | 内部异常；绝不当作通过 |

每个新分析目录包含 `spec.json`、`input.json`、`sources.json`、`intervals.json`、`report.json`。正式交付还含 `clearance.png` 与 `artifact-index.json`。报告里的 input/spec/source/interval 指纹可交叉检查，代码及 Schema 身份也保存。01 是保留的开发分析，02 是最终交付；01 对应的旧 Schema 保存在开发修订备份中，不回写旧分析。

报告 `pairs[].sample_min` 仅取两对象同时直接采样点；`aligned_sample_min` 允许有界插值；`linear_model_min` 在每一有效相对线性段内求最近点。每个最小值带时间和括号索引，索引对应 `normalized_samples` 中的原始文件行号、seq、pose_index。缺口不连线；孤立观测重叠仍保留在 `sample_min_geometric_contact`，但不能据此宣布完整时间范围。

半径来自归档 collision：机器人以模型原点为中心，全部七个碰撞体的三维包络球投影半径 0.2725947818607763 m；圆柱半径 0.1 m。这个机器人圆偏保守，不是最小圆，也不是 Nav2 膨胀半径。包络相交不代表车身接触。原始 pose 的 protobuf 子字段省略可按协议解码零值，整个 position/orientation 或实体消息缺失绝不补零。

同一 epoch 的源时间对齐；最大原始间隔固定 50 ms，无外推，不跨缺口，冲突重复/乱序/实体变化拒绝。采样间只是线性圆心模型：pose 服务的跳变/保持、物理响应及未知插值误差不能自动当连续真值。未知位置、半径、时间、插值误差分别 null，数值 epsilon 不代表测量精度。physical collision 与正式回合 contact_sensor_event 保持 null。

## 已验证测试和合成反例

```bash
cd /home/robot/commercial_mobile_robot
python3 -B -m pytest -p no:cacheprovider -q tests/test_geometry_metrics.py tests/test_geometry_alignment.py tests/test_geometry_inputs.py tests/test_geometry_cli.py
python3 -B -m pytest -p no:cacheprovider -q tests/test_run_schema.py tests/test_runner_lifecycle.py tests/test_runner_summary.py
python3 -B tools/evaluate_geometry.py --fixture-input tests/fixtures/geometry/interior.json --allow-fixtures --check
python3 -B tools/evaluate_geometry.py --fixture-input tests/fixtures/geometry/gap.json --allow-fixtures --check
```

分别为 48 passed、160 passed、退出 0、退出 4。测试用例明确 synthetic，不能替代真实记录。不要运行整个 tests 目录触发无关 ROS 测试。详细独立复算和真实局部接触自检见 `/home/robot/commercial_mobile_robot/evidence/env/p0-07-dev-4v9sd2n4/independent-final`。

## 图表

```bash
cd /home/robot/commercial_mobile_robot
MPLCONFIGDIR=/home/robot/commercial_mobile_robot/evidence/env/p0-07-dev-4v9sd2n4/matplotlib-cache python3 -B tools/plot_geometry.py --report /home/robot/commercial_mobile_robot/evidence/metrics/p0-06-crossing-kwdqt70u-c12/p0-07-4v9sd2n4-02/report.json --output /home/robot/commercial_mobile_robot/evidence/metrics/p0-06-crossing-kwdqt70u-c12/p0-07-4v9sd2n4-02/clearance.png
```

已执行成功；PNG 已存在，重复导出会拒绝。图表只是模型距离与实际采样圆心，不能代替原始激光、全世界碰撞评价或本人的目视确认。

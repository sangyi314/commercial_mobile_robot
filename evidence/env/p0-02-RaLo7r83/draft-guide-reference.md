# P0-02 顺序执行版：先约定实验记录，再实现校验与测试

整理日期：2026-10-20，Asia/Shanghai。  
项目工作路径：`/home/robot/commercial_mobile_robot`。  
顺序依据：《Phase_0_Replanned_Roadmap.md》的 P0-02；操作形式沿用你已经完成的《Phase_0_P0-01_顺序执行版.md》。

**现在做 P0-02：定义最小场景、运行身份和结果记录，编写一个只读校验工具，用人工构造的数据验证它。**完成后进入 P0-03，在官方 TurtleBot3 环境做第一次最小导航，把实测信息补进这些记录。

你已经完成 P0-01，本指南继承它的核心实现、测试和构建成果。本步不重复补核心测试，也不重新生成 MPPI/DWB 配置。

本次编写指南时，实际通过现有 SSH 入口只读检查了 Ubuntu 仓库、既有日志和官方 launch 文件；**没有实施下文的代码修改，没有重新运行核心测试，没有启动 ROS 节点、Gazebo 或导航。**附带文档里的执行命令是参考材料，不是本轮修改代码的授权。

## 先看这一页：操作顺序

| 步骤 | 做什么 | 操作方式 | 什么时候可以继续 |
|---|---|---|---|
| 1 | 进入原项目，确认已有成果与本步缺口 | 终端检查、阅读记录 | 知道哪些文件已存在，不重复创建 |
| 2 | 保存修改前状态和受保护文件指纹 | 终端操作 | 备份可读，本次编号确定 |
| 3 | 检查 Python 依赖与已有校验测试 | 终端操作 | 依赖可用；已有测试如实记录 |
| 4 | 理解本步必需的四条记录规则 | 学习、举例判断 | 能区分草稿、完成记录和未测量值 |
| 5 | 确定三种记录的字段与测试预期 | 阅读、文档设计 | 输入、输出、单位、终态和反例明确 |
| 6 | 让 AI 增量实现，再审查差异 | 发给 AI、代码审查 | 实现与测试符合第 5 步合同 |
| 7 | 检查代码语法和 Schema 本身 | 终端操作 | 静态检查通过 |
| 8 | 执行合同单元测试 | 终端操作 | 应接受的接受，应拒绝的按原因拒绝 |
| 9 | 从命令行实际验证一组文件 | 终端操作 | 文件读取、跨文件检查、退出码正确 |
| 10 | 登记本机官方场景资源，保留运行待确认项 | 只读调查、文档编辑 | 草稿合法；未确认内容没有冒充实测 |
| 11 | 核对最终差异、指纹和个人理解 | 终端检查、本人验收 | 核心与配置保留，证据对应最终文件 |
| 12 | 写完成报告，交接给 P0-03 | 文档编辑、勾选清单 | 合同与工具通过，运行验证边界明确 |

### 怎样使用本文

- **终端操作**下面的 `bash` 块才粘贴到终端；第 1 步连接后，后续全部在 Ubuntu Bash 执行。
- **发给 AI**下面的 `text` 块是实现要求，不粘贴到终端。把本指南也作为参考文件提供给负责实现的 AI。
- **学习 / 文档设计**是你需要确认的工程规则。AI 可以整理文字，你负责决定规则是否正确。
- 尽量沿用同一个终端。`CMR_WS` 是原项目目录，`CMR_EVIDENCE` 是本次检查证据目录，`CMR_CHECK_ID` 是本次唯一编号。
- 命令失败时，保存输出、定位原因，再执行依赖它的步骤。第 9 步包含**故意应当失败的输入**，使用专门的检查方法，不能把它们全改成退出码 0。
- 已完成且有对应版本证据的操作可以继承。第二次运行验收用新的证据子目录，避免覆盖第一次失败日志。

### 这一步究竟新增什么能力

以后有人交给你一个实验目录，你至少能回答：

1. 使用了哪个场景、哪个代码版本、哪份控制器配置？
2. 导航究竟成功、取消、失败，还是连目标都没发出去？
3. 是实验超时、录包失败，还是导航算法返回了失败？
4. 碰撞指标有证据、明确没测，还是记录自相矛盾？

P0-02 交付的是**记录规则、严格校验器和测试**。它为 P0-03 的人工记录、P0-05 的自动运行和 P0-07 的评价提供共同接口。此时无需启动机器人，也无需制作自定义地图。

本步不实现 runner、ROS Bag 录制、动态障碍物、碰撞算法、数据库或网页。CI 可在后续交付步骤接入，当前不用为了写一份工作流中断主线。

## 第 1 步：进入正确项目，确认已有成果与缺口

**方式：终端检查 + 阅读。目的：从你已经完成的 P0-01 接着做。**

### 1.1 连接并确认位置

如果当前在 Mac，只在 Mac 执行：

```bash
ssh cmr-ubuntu
```

如果已经使用 VS Code Remote-SSH 进入 Ubuntu，不再嵌套连接。以下命令在 Ubuntu 执行：

```bash
bash --noprofile --norc
export CMR_WS=/home/robot/commercial_mobile_robot
export PYTHONDONTWRITEBYTECODE=1
cd "$CMR_WS" || exit 1
uname -srm
cat /etc/os-release
git rev-parse --show-toplevel
git rev-parse HEAD
git status --short
```

预期：Ubuntu 24.04，Git 根就是 `CMR_WS`。目录不存在时检查机器和用户，不创建另一个空项目代替它。

本步使用 Python 文件处理，不要求 source ROS 环境，也不需要修改 `ROS_DOMAIN_ID`、shell 配置或系统时钟。

### 1.2 查看本步涉及的目录

```bash
cd "$CMR_WS"
python3 - <<'PY'
from pathlib import Path

for name in [
    "experiments", "tests", "tools", "docs/verification",
    ".github/workflows",
]:
    p = Path(name)
    print(f"\n[{name}]")
    if not p.exists():
        print("MISSING")
        continue
    for f in sorted(p.rglob("*")):
        if f.is_file() and "__pycache__" not in f.parts:
            print(f)
for name in [
    "src/cmr_core/include/cmr_core/core.hpp",
    "src/cmr_core/test/core_test.cpp",
    "src/cmr_core/CMakeLists.txt",
    "src/cmr_core/package.xml",
    "src/cmr_bringup/config/nav2_installed.yaml",
    "src/cmr_bringup/config/nav2_mppi.yaml",
    "src/cmr_bringup/config/nav2_dwb.yaml",
    "tools/make_baselines.py",
]:
    p = Path(name)
    if not p.is_file():
        raise SystemExit(f"Expected existing file is missing: {name}")
print("\nExisting core and baseline files found.")
PY
```

本次写指南时的只读现场：

| 对象 | 查到的状态 | 本步处理 |
|---|---|---|
| Git HEAD | `8f24b68e052ca5cdff8dafa11a4939f7c481de99` | 作为编写时快照，执行时以当前 HEAD 为准 |
| 核心测试文件 | 已扩展为 278 行 | 保留，不重新实施 P0-01 |
| P0-01 证据 | 读取的 Release、sanitizer 日志为 5/5，colcon 汇总为 5 tests / 0 errors / 0 failures / 0 skipped；两个安装消费者日志均为 1/1 | 继承相关记录；本轮没有重新执行或完整重验全部测试 |
| P0-01 草稿报告 | 仍写着“步骤 6，未验收” | 完成报告与实际后续证据需要对应，不靠旧草稿判断你没做过 |
| `experiments/manifests/` | 已有 `environment.json`、`ros2_doctor.txt`、`tb3_launch_args.txt` | 作为历史环境材料保留；不改造成单回合 manifest |
| `experiments/scenarios/`、`tests/` | 目录存在，未见本步实现文件 | 目录可复用 |
| `experiments/schemas/` | 未见目录 | 实施时确认仍不存在，再新增 |
| `tools/` | 已有 `make_baselines.py` | 保留，只按需新增 `validate_run.py` |
| Python 依赖 | pytest 7.4.4、jsonschema 4.10.3、PyYAML 6.0.1；存在 `Draft202012Validator` | 已够用，不为本步升级整套环境 |

仓库有未跟踪的 P0-01 日志属于可解释状态，不要求清理它们。后续如果你已经新增 P0-02 文件，先读取和合并，不用本文的“未见”覆盖当前事实。

### 1.3 处理 P0-01 的交接记录

阅读自己的 P0-01 最终验收记录。若只有草稿而实际测试已经做完，在最终整理时**新建补充说明，链接原始日志**；无需为更新报告再跑一遍没有变化的 C++ 测试。

新说明应区分“已读取的历史日志”“实际执行者确认”“本次没有重跑”。不要让 AI 直接修改旧草稿，把所有“未执行”批量替换成“通过”。

P0-02 的接口设计技术上可以独立于核心测试开展；这项报告整理不应把你拉回重复构建循环。

**本步完成标准：**确认原项目、现有成果和 P0-02 同名文件情况，知道历史材料和新实验记录不是同一种东西。

## 第 2 步：保存修改前状态，确定本次编号

**方式：终端操作。目的：增量实施时能看清哪些内容发生了变化。**

从本步开始，以下命令会在原仓库内创建本次证据目录。它不是新的项目工作区。

```bash
set -eo pipefail
cd "$CMR_WS"
test "$(git rev-parse --show-toplevel)" = "$CMR_WS"
mkdir -p "$CMR_WS/evidence/env"
export CMR_EVIDENCE
CMR_EVIDENCE="$(mktemp -d "$CMR_WS/evidence/env/p0-02-XXXXXXXX")"
export CMR_CHECK_ID
CMR_CHECK_ID="$(basename "$CMR_EVIDENCE")"
printf 'CMR_EVIDENCE=%s\nCMR_CHECK_ID=%s\n' \
  "$CMR_EVIDENCE" "$CMR_CHECK_ID"
git status --porcelain=v1 > "$CMR_EVIDENCE/git-status-before.txt"
git rev-parse HEAD > "$CMR_EVIDENCE/git-head-before.txt"
git diff --binary > "$CMR_EVIDENCE/unstaged-before.patch"
git diff --cached --binary > "$CMR_EVIDENCE/staged-before.patch"
```

接着保存已有实验文件、核心与配置指纹：

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib
import json
import os
import tarfile

root = Path(os.environ["CMR_WS"])
out = Path(os.environ["CMR_EVIDENCE"])
protected = [
    "src/cmr_core/include/cmr_core/core.hpp",
    "src/cmr_core/test/core_test.cpp",
    "src/cmr_core/CMakeLists.txt",
    "src/cmr_core/package.xml",
    "src/cmr_bringup/config/nav2_installed.yaml",
    "src/cmr_bringup/config/nav2_mppi.yaml",
    "src/cmr_bringup/config/nav2_dwb.yaml",
    "tools/make_baselines.py",
]
old_env = root / "experiments/manifests"
if old_env.exists():
    protected += [
        str(p.relative_to(root))
        for p in sorted(old_env.rglob("*")) if p.is_file()
    ]
hashes = {}
for name in protected:
    p = root / name
    if not p.is_file():
        raise SystemExit(f"Missing protected file: {name}")
    hashes[name] = hashlib.sha256(p.read_bytes()).hexdigest()
(out / "protected-before.json").write_text(
    json.dumps(hashes, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
targets = [
    "experiments", "tests", "tools/validate_run.py",
    "docs/run_schema_test_design.md",
]
present = [name for name in targets if (root / name).exists()]
(out / "existing-targets.json").write_text(
    json.dumps(present, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
with tarfile.open(out / "experiments-before.tgz", "w:gz") as archive:
    for name in present:
        archive.add(root / name, arcname=name)
with tarfile.open(out / "experiments-before.tgz", "r:gz") as archive:
    names = archive.getnames()
(out / "backup-index.txt").write_text("\n".join(names) + "\n", encoding="utf-8")
print("Backup readable; protected files:", len(hashes))
print("Evidence:", out)
PY
```

记住屏幕上的真实编号。不要把示例编号替换到旧证据目录，更不要删除旧日志来“保持干净”。

**本步完成标准：**备份能打开，受保护文件指纹已保存，后续所有日志使用本次编号。

## 第 3 步：检查依赖和已有测试，不安装多余工具

**方式：终端操作。目的：确认本步的小工具可以使用现有 Python 环境。**

### 3.1 检查 Python 与库

```bash
python3 - <<'PY' 2>&1 | tee "$CMR_EVIDENCE/python-environment.log"
import sys
from importlib.metadata import version
from jsonschema import Draft202012Validator
import yaml
import pytest

print("Python:", sys.version)
print("Executable:", sys.executable)
for name in ["jsonschema", "PyYAML", "pytest"]:
    print(name, version(name))
Draft202012Validator.check_schema({
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "type": "object",
})
print("Draft 2020-12 available.")
PY
```

编写时已经确认依赖存在，因此正常情况直接继续。只有明确出现 `ModuleNotFoundError`，才安装缺失的对应 Ubuntu 包；例如三个都缺时：

```bash
sudo apt-get install python3-jsonschema python3-yaml python3-pytest
```

安装后重跑 3.1。不要使用 `sudo pip` 或 `--break-system-packages`，也不需要重建工作区、切换 Python 主版本或升级 ROS。

### 3.2 如果本步测试已经存在，先运行基准

```bash
if [ -f tests/test_run_schema.py ]; then
  python3 -m pytest -p no:cacheprovider -v tests/test_run_schema.py \
    --junitxml="$CMR_EVIDENCE/schema-before.xml" \
    2>&1 | tee "$CMR_EVIDENCE/schema-before.log"
else
  printf '%s\n' 'NOT_IMPLEMENTED: tests/test_run_schema.py does not exist.' \
    | tee "$CMR_EVIDENCE/schema-before.log"
fi
```

当前文件不存在是本步待实现的事实，不是“已有测试通过”。若存在却测试失败，先保留失败原因；不得删掉旧测试换成更弱的版本。

**本步完成标准：**依赖检查通过，现有测试状态有真实记录。

## 第 4 步：先理解四条规则，再设计字段

**方式：学习 + 举例判断。目的：知道校验器为什么需要这些条件。**

### 4.1 三种记录各负责一件事

| 记录 | 回答的问题 | 谁创建、何时使用 |
|---|---|---|
| `ScenarioSpec`，场景说明 | 在哪个世界、地图和机器人上，从哪里到哪里，观察哪些信息？ | 人编写；P0-03 确认，后续 runner 读取 |
| `RunManifest`，单回合身份记录 | 这一次运行用了哪个场景版本、代码和配置？ | P0-03 先手工；P0-05 再由 runner 自动写 |
| `RunResult`，单回合结束记录 | 运行如何结束，有哪些测量，有哪些不知道？ | 运行结束时写；评价器可以按版本补充评价 |

`experiments/manifests/environment.json` 是已有环境材料，不是这三种对象的替代品。

本步选择 YAML 存放人编辑的场景，JSON 存放运行身份和结果。JSON Schema 检查字段、类型和枚举；Python 代码负责跨文件一致性及条件规则。无需把每条复杂关系硬塞进 Schema。

### 4.2 “数据合法”不等于“机器人已经工作”

建立四层判断：

1. **能解析：**文件语法能读取。
2. **合同合法：**字段、类型、标识和终态互相一致。
3. **准备条件有记录：**官方资源、位姿和接口经过规定的人工确认。
4. **运行与评价有效：**实际导航、录包和评价都有对应证据。

P0-02 主要完成前两层。正式场景在这一阶段可以是 `draft`，位姿等未测值保留 `null`。

校验器应输出 `VALID`，并明确 `validation_scope: "record_contract"`。不要输出“导航验收通过”或“安全验证通过”。

### 4.3 `null`、`false` 和 `0` 是不同含义

以碰撞为例：

| 值 | 含义 | 需要什么 |
|---|---|---|
| `collision: true` | 在明确范围内检测到了可信碰撞事件 | 事件来源、时间、对象和证据引用 |
| `collision: false` | 在明确评价范围内，完整有效的检查没有检出碰撞 | 评价有效、范围和覆盖完整、可追溯证据 |
| `collision: null` | 没测、缺数据、无法对齐，或证据不足 | 写清原因，不能进入有效碰撞率分母 |

“没看到碰撞话题”不支持 `false`；“导航成功”也不支持 `false`。

检测到一个可信碰撞事件时，即使其余时间的数据不完整，仍可以记录 `true`；但不能据此说整个轨迹都评价完整。

这里的 `collision` 总是针对 `evaluation.scope_id` 指定的对象和时间范围。圆形近似距离、接触传感器、完整物理碰撞不是同一种证据。一个局部几何指标没有接触，不能推出机器人全程没有物理碰撞。

不要写：

```python
# 错误示例，仅用于阅读。
collision = bool(data.get("collision"))
clearance = data.get("min_signed_clearance_m") or 0.0
```

这会把“未知”变成“没撞”或“零距离”。应保留 `None`，并分别检查类型和条件。

### 4.4 不把不同时间域相减

| 时间字段 | 用途 | 本步规则 |
|---|---|---|
| UTC 字符串 | 找到某次实验、关联日志 | 不用于控制超时 |
| ROS 仿真时间，秒 | 关联消息和场景事件 | 必须属于同一 `clock_epoch` |
| 单调时钟经过时长，秒 | 启动超时、导航超时和清理超时 | 仿真暂停时仍能推进 |
| RTF | 某个窗口的仿真时间增量 / 单调时间增量 | 明确窗口，分母大于零；无测量就 `null` |

`clock_epoch` 是本实验记录对一次连续仿真时间段的标识，不是声称 ROS 自带某个现成字段。时钟回跳或仿真重置后，不能继续把两段时间当成同一次连续测量；本步拒绝跨 epoch 的普通完成记录，后续 runner 记录该故障。

**本步完成标准：**你能解释“Action 成功、碰撞未知、清理失败”为什么可以同时发生；也知道场景草稿可以合法，但尚不能自动运行。

## 第 5 步：确定最小字段、终态与测试预期

**方式：文档设计。目的：先确定正确答案，再让 AI 写校验代码。**

本节是实现合同。由 AI 在第 6 步整理到 `experiments/schema.md`，并写入对应 Schema；你先审查规则。不需要手抄大量 JSON。

### 5.1 文件范围

以下路径均相对于 `/home/robot/commercial_mobile_robot`：

```text
experiments/
  schema.md
  schemas/
    scenario.schema.json
    manifest.schema.json
    result.schema.json
  scenarios/
    official_static.yaml
tools/
  validate_run.py
tests/
  test_run_schema.py
  fixtures/
    README.md
    scenarios/
      crossing_draft.yaml
      observed_occlusion_draft.yaml
    runs/
      succeeded_unknown/
      nav_timeout/
      startup_failed/
      collision_observed_partial/
      collision_false_scoped/
      incomplete/
      invalid_collision_false/
docs/
  run_schema_test_design.md
  verification/
    <本次编号>.md
```

每个完整 run fixture 包含 `manifest.json`、`result.json`；需要引用证据时在自己的 `artifacts/` 下放**明确标为合成数据**的小文件。`incomplete/` 故意只有合法 manifest。

这不是七次实际实验，不能放进 `evidence/runs/`，不能计入 Master Plan 的冷启动或导航回合数。

### 5.2 所有记录共有的规则

- 本轮 `schema_version` 固定为整数 `1`，Schema 同时写 `type: integer` 和 `const: 1`；布尔 `true` 不得冒充版本号。
- 三个 Schema 显式声明 JSON Schema Draft 2020-12。使用文件内 `$defs` 即可；本步禁止远程 `$ref` 和隐式联网下载。
- 文档化的对象禁止未知字段，通常用 `additionalProperties: false`；需要扩展就审查合同，而不是默默吞掉拼错的字段名。
- 字段表要求存在的键必须存在；允许未知时写 `null`，不通过缺键或自动填默认值隐瞒缺测。
- 数值必须有限；拒绝 JSON 的 `NaN`、`Infinity`、`-Infinity`、溢出成无穷的 `1e309`，以及 YAML 的 `.nan`、`.inf`。
- 重复键必须拒绝。JSON 和 YAML 都不能“后一个覆盖前一个”。
- 只允许 JSON 可表达的数据类型，映射键必须是字符串。YAML 使用 SafeLoader 的严格子类，拒绝重复键和 merge key；禁止构造任意 Python 对象。
- `null`、布尔、数字、字符串不互相强转。Schema 的数字类型和额外语义检查都不能把 Python `bool` 当成数值。
- 字符串标识非空；SHA-256 是 64 位十六进制字符串。fixture 中的 hash 也对应其合成文件字节，不伪称真实实验 hash。
- 场景 revision 改变意味着内容修订；Schema 版本改变意味着记录格式变化。二者独立。

Python 默认 JSON 解码器会接受部分非标准数字，并对重复键保留最后一个值；需要显式严格解析，而不是只调用默认 `json.load()`。写 JSON 时使用 `allow_nan=False`；读取后还要递归检查有限值，以捕获 `1e309` 等情况。[Python 3.12 JSON 文档](https://docs.python.org/3.12/library/json.html#standard-compliance-and-interoperability)

### 5.3 `ScenarioSpec v1`：场景说明

下面的键为必需键；表中标明可以 `null` 的内容可暂时未知。嵌套对象的具体结构也必须在 `schema.md` 写清楚。

| 字段 | 类型 / 约束 | 目的与当前填写方法 |
|---|---|---|
| `schema_version` | 整数 1 | 格式版本 |
| `scenario_id`、`scenario_revision` | 非空字符串 | 例如 `official_static`、`r1` |
| `kind` | `static_smoke / crossing / observed_occlusion / head_on` | 使用含义明确的类别 |
| `master_scenario_id` | `S0 / S1 / S2 / S3` | 静态→S0；横穿→S1；对向→S2；已观测遮挡→S3 |
| `legacy_id` | 字符串或 null | 历史 `s2_occluded` 可以保留，但不能映射成 Master 的 S2 |
| `status` | `draft / ready_for_manual_smoke` | 当前官方场景通常为 draft |
| `resources` | 对象 | `world / map_yaml / map_image / robot` 四个 ResourceRef 或 null；另含 `dependency_lock`，可以 null |
| `task` | 对象 | `initial_pose / goal_pose` 均可 null；非 null 时含 `frame_id,x_m,y_m,yaw_rad`；另有 `common_constraints_ref`，可 null |
| `interfaces` | 数组 | 每项含 `role,topic,type,frame_id,time_domain,usage`；未调查时可为空 |
| `dynamic_events` | 数组 | 静态为空；动态草稿可用 `event_id,actor_id,trigger,motion_ref`，后三者允许 null |
| `evaluation_scope` | 对象或 null | 非 null 时说明 `scope_id,objects,geometry_model,truth_source_ref`；草稿内容可待确认 |
| `verification` | 对象 | `resource_dependencies / world_map_alignment / poses_free / interfaces_observed` 各为 `pending / checked`；另有 `evidence_refs` |

约定小对象：

- `ResourceRef = {path, sha256, package, package_version}`。path 是当前宿主可解释的绝对路径，或相对项目根的路径，不使用未实现的自定义 URI 协议；package/package_version 对项目自有文件可为 null。draft 中 sha256 可为 null，但不能填假 hash；已声明 hash 的资源必须实际存在、非空且匹配。整个 ResourceRef 为 null 时不检查其文件。
- 位姿全部是二维，米和弧度；`frame_id` 非空。出生点、AMCL 初值和目标不是三个可以随意互换的值。
- `interfaces[].usage` 为 `algorithm_input` 或 `evaluation_only`；`time_domain` 为 `ros_sim` 或 `monotonic`。未确认的话题不凭经验填成已观察。
- `dynamic_events[].trigger` 非 null 时为 `{time_domain: "ros_sim", at_s: 非负有限数}`，`motion_ref` 是后续动作定义文件引用。这只是接口槽位，本步不实现动作执行器。
- `verification.evidence_refs` 为本机文件路径列表，相对路径按项目根解析，也允许绝对路径；引用必须存在且非空。只有实际已有人工检查证据时才填写，不根据文件名设置 checked。这种场景准备证据引用，与下面限制在 run 目录内的 ArtifactRef 是不同类型。

本版 `--require-ready` 只放行 status=`ready_for_manual_smoke` 的静态场景：四个资源及依赖清单均具备真实 hash、两个位姿/公共约束存在、四个 verification 项 checked 且有证据引用。依赖清单的文件至少列已解析资源路径和对应 hash；未解析依赖必须保留为待办，不能设 checked。`task.common_constraints_ref` 非 null 时也是 ResourceRef。合法 draft 在此选项下返回 NOT_READY；若文件自己声称 ready 却缺必要条件，普通校验也应返回 INVALID。

这里的 ready 是“按记录满足人工冒烟准备要求”，不是已完成导航或安全验收。动态类别本步只验证 draft 可表达；其 ready 条件在 P0-06 根据真实感知与事件接口再细化。

**P0-02 不要求把官方场景升成 ready。**P0-03 允许人工带着 draft 启动调查；确认地图、TF、位姿和接口之后再补齐。这避免“必须先运行才 ready，必须 ready 才能运行”的循环依赖。

### 5.4 `RunManifest v1`：一次运行的身份

| 字段 | 类型 / 约束 | 含义 |
|---|---|---|
| `schema_version`、`run_id` | 整数 1、非空字符串 | 一次运行的唯一标识 |
| `record_origin` | `real / fixture` | 测试数据必须为 fixture |
| `scenario` | 对象 | `scenario_id,scenario_revision,spec_sha256`；指向这一轮使用的场景快照 |
| `source` | 对象 | `git_commit,dirty,snapshot_ref`；dirty 为 true 时必须有保存实际修改内容的快照引用 |
| `baseline` | 对象 | `method: mppi / dwb`、`config_sha256,expected_plugin,common_constraints_sha256`；公共约束需有对应快照 |
| `environment` | 对象 | `ros_distro,platform,package_versions,external_seed,seed_scope,controller_seed_controlled` |
| `clock` | 对象 | `clock_epoch,started_at_utc,sim_start_s`；尚未获得仿真时间时 sim_start_s 为 null |
| `artifacts` | ArtifactRef 数组 | 场景、配置、必要的代码修改快照等文件引用 |

`ArtifactRef = {id,path,sha256,kind}`：id 非空且在同一记录数组中唯一；path 是**相对本次 run 目录**的文件路径；kind 为 `scenario / config / common_constraints / source_snapshot / runtime_parameters / log / bag_metadata / evaluation / other`。

校验器只读检查被引用文件存在、非空、hash 匹配；拒绝绝对路径、`..` 逃逸以及符号链接解析后逃离 run 目录。`source.snapshot_ref` 使用对应 ArtifactRef 的 id。场景、配置和公共约束 ArtifactRef 必须各有且只有一个，其 hash 与上表对应字段一致。公共约束快照只记录本轮共用的限速、足迹等配置及来源；没有运行核验时不能声称两控制器已公平调参。

场景快照再做严格解析和结构/语义校验，scenario_id、scenario_revision 与 manifest 一致。此处不强制 require-ready：P0-03 的第一次人工调查允许保留尚未完成准备核验的场景草稿。归档 run 中的场景资源路径可能来自另一台机器，校验快照内容时不重新访问那些宿主资源；其实际资源完整性另由准备记录和依赖快照负责。配置与其他 ArtifactRef 在本步只核对引用和字节，不调用 Nav2 或执行文件内容。

这项文件检查只能证明引用没有丢失或串错，不能证明日志内容真实、bag 内有有效消息或者评价算法正确。后续阶段分别负责这些语义验证。

`source.dirty` 是布尔值；当前仓库的 `git_commit` 使用 40 位十六进制 commit 标识。真实记录使用实际 commit；fixture 可用合成 commit 标识并在 README 声明。真实 dirty 运行不能只写 commit 和 `dirty: true`，却丢失修改内容。

`package_versions` 是包名到版本字符串的映射；这是允许任意非空包名键的明确例外。`external_seed` 是整数或 null，`seed_scope` 是字符串列表；`controller_seed_controlled` 是布尔值，未证实时填 false。设置一个 runner 种子不等于 Gazebo、MPPI 和所有随机过程都可复现。

本机现有配置静态核对到的插件标识：

| method | expected_plugin |
|---|---|
| mppi | `nav2_mppi_controller::MPPIController` |
| dwb | `dwb_core::DWBLocalPlanner` |

把这两个映射写进合同；实际运行身份仍由 P0-03/04 获取。

### 5.5 `RunResult v1`：一次运行的结束与测量

| 字段 | 类型 / 约束 | 含义 |
|---|---|---|
| `schema_version,run_id,record_origin` | 与 manifest 一致 | 防止串用其他回合的结果 |
| `manifest_sha256` | SHA-256 | 对应同目录 manifest 的实际字节 |
| `status` | 对象 | 使用下表四个独立状态；另有 `primary_reason` 字符串或 null |
| `runtime` | 对象 | `actual_plugin,parameters_ref`，均可 null；后者引用 result 的 ArtifactRef id |
| `time` | 对象 | `clock_epoch,sim_start_s,sim_end_s,elapsed_monotonic_s,rtf` |
| `metrics` | 对象 | `action_success,goal_position_error_m,goal_yaw_error_rad,stop_observed,intervention,delivery_success` |
| `evaluation` | 对象 | 见 5.6，所有测量未知时保留 null 与原因 |
| `artifacts` | ArtifactRef 数组 | 日志、运行参数、评价证据等 |

状态枚举与规则：

| 状态键 | 可选值 | 规则 |
|---|---|---|
| `startup` | `READY / FAILED` | 未能进入导航就 FAILED；失败不是丢弃这一轮的理由 |
| `action` | `NOT_SENT / REJECTED / SUCCEEDED / ABORTED / CANCELED / UNKNOWN` | 不用 SUCCESS 之类另造近义枚举 |
| `harness` | `COMPLETED / STARTUP_FAILED / NAV_TIMEOUT / RECORDER_FAILED / INTERRUPTED / INTERNAL_ERROR` | 表达实验流程如何结束 |
| `cleanup` | `SUCCEEDED / FAILED / NOT_ATTEMPTED` | 清理失败独立记录，不覆盖原导航终态 |

再确定这些条件：

1. `action_success`：SUCCEEDED→true；REJECTED/ABORTED/CANCELED→false；NOT_SENT/UNKNOWN→null。结果未知不是算法失败的同义词。
2. `startup=FAILED` 时 action 为 NOT_SENT、action_success 为 null；runtime 可全部 null。不能因为插件没有加载就拒绝记录一个真实的启动失败。
3. startup=READY 时 `actual_plugin` 必须等于 manifest.expected_plugin，并有运行参数文件引用。mppi/dwb 的 expected_plugin 必须对应本机核对过的插件标识；不把配置存在当作实际加载。
4. `NAV_TIMEOUT` 不能与 action=SUCCEEDED 同时出现。本合同把已确认的 SUCCEEDED 作为导航阶段终态，之后超时属于清理或其他阶段；发出取消后无法确认终态用 UNKNOWN，不伪造 CANCELED。
5. harness 非 COMPLETED、startup FAILED 或 cleanup FAILED 时，primary_reason 非空。若同时多处失败，保留最先导致结束的原因，其余细节放日志；不能被最后一条清理异常改写。
6. `goal_position_error_m` 为非负有限数或 null；`goal_yaw_error_rad` 是绝对角误差，范围 `[0, π]` 或 null。`stop_observed`、`intervention` 为布尔或 null；未知不能填 false。
7. P0 的 `delivery_success` 固定为 null。导航终态不是配送任务、交接或订单成功。
8. result 与 manifest 的 `clock_epoch` 相同；两者 sim_start_s 若都非 null 必须一致。sim_end_s 不能小于 sim_start_s；测不到时为 null。单调时长为非负有限数。
9. `rtf` 为 null，或 `{value,sim_delta_s,monotonic_delta_s}`；三个值有限，仿真增量非负、单调增量大于零，value 与两者比值按 `rel_tol=1e-9, abs_tol=1e-12` 比较一致。v1 只使用“本次记录起点到终点”的同一窗口；两种增量须分别与 time 中的起止差和单调时长对应。缺仿真起止值时 rtf 为 null，不用不同窗口拼出比例。
10. `started_at_utc` 要求带 `Z` 的有效 UTC 时间，实际解析日期；仅匹配正则表达式不够。不依赖默认不启用的 Schema format 检查。

如果运行目录只有合法 manifest、没有 result，输出 **INCOMPLETE**。不得自动生成一个“超时结果”或“失败结果”来填补。已经存在但损坏的 result 应输出 **INVALID**，不能当作普通未完成掩盖。

### 5.6 `evaluation`：保留有效性，防止伪造安全结论

必需键如下：

```text
evaluator_version       字符串或 null
scope_id                字符串或 null
objects                 对象 ID 字符串列表
geometry_model          字符串或 null
frame_id                字符串或 null
clock_epoch             字符串或 null
interval_s              {start, end} 或 null，ROS 仿真时间秒
coverage                "none" / "partial" / "full"
evaluation_valid        boolean
invalid_reason          字符串或 null
evidence_refs           result.artifacts 中的 ID 列表
collision               boolean 或 null
collision_event         {at_s, objects, source_ref} 或 null
min_signed_clearance_m  有限数或 null
geometric_contact       boolean 或 null
contact_sensor_event    boolean 或 null
```

语义要求：

- 没有评价：coverage=none，evaluation_valid=false，invalid_reason 非空；上述四个测量值全部 null，collision_event 为 null。
- `evaluation_valid=true` 必须有评价器版本、范围、非空对象列表、几何定义、坐标系、同 epoch 的合法区间及可追溯证据；valid 表示声明范围的评价满足其条件，不自动等于覆盖全程。
- `collision=false` 额外要求 coverage=full、evaluation_valid=true，明确覆盖该 scope 的完整区间和所有 objects。不得由 `geometric_contact=false` 自动推导。
- `collision=true` 必须有可信事件引用、事件时刻和对象；时刻属于评价区间和同 epoch。允许 coverage=partial、evaluation_valid=false，但要保留其余区间无效的原因。此时只承认有证据的事件。
- 评价区间需有限、start≤end，epoch 与 run 相同，且不越出已知的运行起止时间；事件对象属于本次评价 objects。`evidence_refs` 和事件的 `source_ref` 都要解析到 result 的 ArtifactRef，不能只检查非空字符串。evaluation_valid=false 时 invalid_reason 非空；为 true 时为 null。
- `collision=null` 可以与有效的局部几何距离共存；例如只测到圆形代理距离，没有足够依据判定物理碰撞。
- signed clearance 定义为有符号间距：正数分离、零为边界接触、负数重叠。负数不是解析错误；不同几何模型不可直接混在一起排名。
- 所有非 null 测量都需要说明其来源；评价未开始时不能填写零距离、false 接触或零碰撞。
- 校验器检查记录结构、条件和引用，不重放数据、不计算几何、不证明 coverage 的声明真实。真实有效性由 P0-07 的评价器测试、数据完整性检查及人工审查另行确认。

### 5.7 固定 CLI，方便后续直接使用

本步只实现两个子命令：

```text
python3 tools/validate_run.py scenario <场景文件> [--require-ready]
python3 tools/validate_run.py run <回合目录> [--allow-fixtures]
```

标准输出为一个 JSON 对象，至少包含：

```json
{
  "status": "VALID",
  "validation_scope": "record_contract",
  "record_origin": "fixture",
  "errors": [],
  "warnings": ["Synthetic fixture; not a robot experiment."]
}
```

场景没有 record_origin，输出该项为 null。errors 每项含 `code,path,message`，path 指明文件与字段。warnings 保留“草稿”“合成数据”“未做数据内容语义核验”等说明。

| 退出码 | status | 含义 |
|---|---|---|
| 0 | `VALID` | 该输入符合记录合同 |
| 2 | `INVALID` | 解析、结构、语义、引用或来源限制不满足；命令参数错误也可用 2 |
| 3 | `INCOMPLETE` | manifest 合法，但缺 result |
| 4 | `NOT_READY` | 合法场景未满足 `--require-ready` |
| 1 | 不包装成 VALID | 工具自身意外异常，保留诊断 |

检查顺序：解析与身份 → fixture 限制 → 结构和引用 → 是否缺 result → 跨文件及终态/评价规则 → ready 条件。输入错误优先于 INCOMPLETE/NOT_READY；`--allow-fixtures` 只允许测试来源，不放宽其他规则。

fixture 默认被拒绝，错误码 `FIXTURE_NOT_ALLOWED`。缺结果用 `MISSING_RESULT`，草稿不能就绪用 `SCENARIO_NOT_READY`，身份不一致用 `IDENTITY_MISMATCH`，无依据 false 用 `COLLISION_UNSUPPORTED`。其他错误可细分，但必须稳定、可读。

建议内部只有四个清楚的入口：`load_strict(path)`、`validate_scenario(path, require_ready=False)`、`validate_run_dir(path, allow_fixtures=False)`、`main(argv=None)`。前两个验证入口返回上述诊断对象，main 负责退出码；测试可以直接调用，也必须实际启动 CLI。避免引入插件框架、数据库或 ROS 依赖。

### 5.8 先列测试答案，不由校验器自己生成预期

请在 `docs/run_schema_test_design.md` 保留下列 ID，并把实现位置和执行证据填进去。一个 ID 可用参数化测试覆盖多个输入，不追求凑测试条数。

| ID | 输入 / 单独变更 | 应得到的结果 |
|---|---|---|
| S01 | 官方静态 draft，位姿 null | 普通校验 VALID；require-ready 为 NOT_READY |
| S02 | 横穿和已观测遮挡 draft | VALID；分别映射 S1、S3，不能将遮挡映射 S2 |
| S03 | 缺 schema_version、空 ID、未知字段/枚举、true 版本号 | INVALID，定位具体字段 |
| S04 | 声称 ready 却缺位姿、依赖或人工检查证据 | INVALID；合法 draft 的 require-ready 才是 NOT_READY |
| S05 | 完全合成的静态 ready fixture，临时资源与证据齐备 | require-ready VALID，但不是物理环境验收 |
| R01 | succeeded_unknown | VALID；action_success=true，collision=null，delivery_success=null |
| R02 | nav_timeout | VALID；保留 NAV_TIMEOUT，取消后为 CANCELED 或已定义的 UNKNOWN |
| R03 | startup_failed | VALID；runtime null、action NOT_SENT，不被强迫伪造插件 |
| R04 | incomplete | INCOMPLETE，目录不被补写 result |
| R05 | 损坏的 result、manifest/result ID 不同或 manifest hash 错 | INVALID；不能误报 INCOMPLETE |
| R06 | 成功但 actual_plugin 与 method/expected_plugin 不一致 | INVALID |
| R07 | action_success 与 action 不一致；cleanup 失败丢掉原终态 | 前者 INVALID；后者测试确保原终态保留、失败原因存在 |
| R08 | dirty=true 没有实际修改快照；引用缺失/hash 不匹配/路径逃逸 | INVALID，不能只验证 hash 字符串长得像 |
| E01 | 无评价的 collision=null | VALID，不转换成 false |
| E02 | 从 E01 只把 collision 改成 false | INVALID，包含 COLLISION_UNSUPPORTED |
| E03 | 可信事件 true、其余时间 coverage=partial | VALID；全程评价仍可无效 |
| E04 | 显式范围内完整有效的 false，证据是合成文件 | 允许 fixture，不能宣称真实机器人安全 |
| E05 | 局部几何无接触但 collision 仍 null；负 signed clearance | 合法组合保留原值；不自动生成全程 collision |
| T01 | epoch 不同、结束早于开始、负单调时长、无效 UTC 日期 | INVALID |
| T02 | RTF 分母零、非法比值；合法 null 和合法窗口 | 前两者 INVALID，后两者 VALID |
| P01 | JSON NaN/±Infinity/1e309、YAML .nan/.inf | INVALID，不能变成 null 后放行 |
| P02 | JSON/YAML 重复键、非字符串键、YAML merge key | INVALID，不能静默覆盖 |
| P03 | bool 冒充数值、字符串 `"false"`、null 被补成零 | INVALID 或保持 null，按字段合同判断 |
| C01 | fixture 不带 allow-fixtures | INVALID，包含 FIXTURE_NOT_ALLOWED |
| C02 | 从仓库外 cwd 调用 CLI；运行前后输入 hash 对照 | 路径正确、输出码一致，输入完全不变 |
| C03 | 缺/空证据文件、缺输入文件、错误 JSON | INVALID，有路径/原因；不能 traceback 后退出 0 |

每个负例从一个已知合法样本出发，尽量只破坏一条规则，并断言正确的错误码或字段。否则“因缺字段报错”可能掩盖“NaN 检查根本没写”的问题。

**本步完成标准：**你能独立判断上表的预期，知道哪些只是合成验证，并接受这些字段作为最小合同。

## 第 6 步：交给 AI 实现，再审查改动

**方式：发给 AI + 查看差异。目的：让 AI 处理重复编码，你掌握接口和验收依据。**

### 6.1 发给 AI 的实现任务

将本指南提供给实现任务，并把下面任务一起发送。请用第 2 步输出的真实证据编号，或让 AI 从你指定的本次目录核对，不能自行挑选“最新看起来像”的旧目录。

```text
任务：在现有 /home/robot/commercial_mobile_robot 增量实施 P0-02。
依据是随附《Phase_0_P0-02_顺序执行版.md》，尤其第 4–5 步的合同。
先读本指南和实际文件，再改；如果你没有收到本指南，请先说明缺失，
不要自行发明另一套字段或省略验收规则。

已知起点：
P0-01 已由负责人完成。本任务不重写 cmr_core、不重建 CMake、不重跑导航配置生成器。
先核对本次 CMR_EVIDENCE / CMR_CHECK_ID、修改前备份和依赖检查。
旧 experiments/manifests 下的文件只是环境材料，保留其内容。

允许新增或增量修改：
experiments/schema.md
experiments/schemas/scenario.schema.json
experiments/schemas/manifest.schema.json
experiments/schemas/result.schema.json
experiments/scenarios/official_static.yaml
tools/validate_run.py
tests/test_run_schema.py
tests/fixtures/README.md
tests/fixtures/scenarios/ 下本步两个动态场景草稿
tests/fixtures/runs/ 下本指南指定的七类合成样本及小型 artifacts
docs/run_schema_test_design.md
docs/verification/ 下本次独立报告
本次 evidence/env/<准确编号>/ 的检查产物。

已有同名文件先读取、识别当前功能、保留有效用例；不要整目录覆盖。
不改 src/cmr_core、任何 CMakeLists/package.xml、三份 nav2 YAML、
tools/make_baselines.py、旧环境 manifests、旧证据。
不改系统/SSH/shell 配置，不建新工作区，不 git reset/clean，不自动 commit/push。
不启动 ROS/Nav2/Gazebo/RViz，不录包，不访问实车。

设计要求：
实现 ScenarioSpec / RunManifest / RunResult v1。
严格按第 5 步字段、枚举、单位、fixture目录名、CLI和退出码实现。
JSON Schema Draft 2020-12；只用本机已有 stdlib、jsonschema、PyYAML；
测试用 pytest。不引入最新版本专有 API，不自动升级依赖。
schema 自包含，只用文件内 $defs，不发起网络请求。
未知字段拒绝；unknown 使用 null；不隐式转换 bool/数字/字符串。
JSON/YAML 都拒绝重复键和非有限数；递归检查，覆盖 1e309。
YAML 使用安全加载器，拒绝 merge key 和非字符串键。
schema_version 要求整数 1，拒绝 bool。
UTC 日期实际解析校验，不仅写 format: date-time。

CLI：
python3 tools/validate_run.py scenario <path> [--require-ready]
python3 tools/validate_run.py run <directory> [--allow-fixtures]
总是输出一个机器可读 JSON 诊断。
0 VALID，2 INVALID，3 INCOMPLETE，4 NOT_READY，内部意外异常非零。
结果明确 validation_scope=record_contract。
缺结果不生成结果；坏结果不降格成 INCOMPLETE。
fixture 默认拒绝；allow-fixtures 不能绕过字段/跨文件/证据检查。
从任意 cwd 执行都能通过 __file__ 找到项目内 schema。
只读校验，不修改输入、不填默认值、不修复文件。

核心语义：
scenario draft 可保留 null 位姿；P0-02 不要求真实 ready。
横穿对应 Master S1；已观测遮挡对应 S3；对向才是 S2。
startup/action/harness/cleanup 分开；失败、超时、缺测都保留。
action_success 按明确终态映射；delivery_success 在 P0 固定 null。
启动失败允许 runtime=null；READY 必须核对 actual_plugin/参数引用。
同回合 run_id、来源、manifest hash、epoch 等必须一致。
dirty 源码需要真实修改快照引用；ArtifactRef 限制在 run 目录内，
检查文件非空、hash、ID引用，不跟随路径逃逸。
collision 三态按第 5.6 节；无证据 false 拒绝。
可信 true 可以保留于 partial coverage；不能把局部几何 false
自动提升为完整物理碰撞 false。
证据文件存在/hash正确，只证明引用一致，不证明数据语义真实。

测试：
实现第 5.8 节所有案例 ID，可参数化；不要只测正常输入。
负例从已知合法样本作单项变更，并检查正确原因/字段。
测试独立写预期，不调用被测函数生成“正确答案”。
fixture 全部 record_origin=fixture，README 明确为合成数据；
证据文件也写明 synthetic。禁止放进 evidence/runs 或累计实验次数。
至少用 subprocess 实际调用 CLI，测试退出码、JSON输出、外部cwd、
默认拒绝fixture、缺结果不写回和输入文件指纹不变。
任何内部异常都不能被 except Exception: return VALID 吞掉。

当前执行停点：
先完成代码、schema、fixture、测试与设计文档草稿，展示修改清单
和关键差异供负责人审查。未实际执行的项目标未执行。
审查后再按本指南第 7–11 步逐项验证，不能拿历史 P0-01 日志
证明本步通过，也不能代替本人勾选理解项。
发现合同矛盾时给出具体输入、冲突规则和最小修订建议；
不要通过删除负例、放宽unknown语义或伪造证据凑通过。
```

### 6.2 查看修改范围

AI 返回草稿后，在 Ubuntu 运行：

```bash
cd "$CMR_WS"
git status --short
git diff --stat
git diff -- experiments tests tools/validate_run.py \
  docs/run_schema_test_design.md
git diff --check
```

未跟踪的新文件不会显示在普通 `git diff` 里，要直接打开阅读：

```bash
sed -n '1,240p' experiments/schema.md
sed -n '1,260p' tools/validate_run.py
sed -n '1,260p' tests/test_run_schema.py
cat experiments/scenarios/official_static.yaml
cat tests/fixtures/README.md
```

文件超过显示范围时继续阅读后续部分，不把“已显示前 260 行”当作完整审查。

### 6.3 审查最容易出错的地方

你重点看这些位置，不必手工抄所有 Schema：

1. 严格加载器是否真正被 CLI 和单元测试共用？只在测试里拒绝 NaN 没有意义。
2. 是否为读取后的整棵对象树检查非有限数，包括列表嵌套和 `1e309`？
3. 是否把 `.get(..., False)`、`bool(None)` 或 `value or 0` 用在测量上？
4. 是否有错误规则：所有 collision=true 都要求 full coverage？这会丢掉已观测到的真实碰撞事件。
5. 是否为了让 fixture 通过，`--allow-fixtures` 顺便跳过全部校验？
6. 是否把 manifest 的插件意图当成 runtime.actual_plugin 的观测？
7. 是否遇到缺 result 就写一个默认成功/失败文件？
8. 是否读写测试了 input hash，而不是仅声称函数“应该只读”？

使用 `Draft202012Validator.check_schema()` 检查 Schema 本身；验证实例是另一个动作。该版本库默认不替你执行所有 `format` 检查，因此日期等关键格式必须明确验证。[jsonschema 4.10.x 校验文档](https://python-jsonschema.readthedocs.io/en/v4.10.2/validate/)

**本步完成标准：**修改范围、字段规则、测试预期和错误处理均已审查。然后进入第 7 步实际运行。

## 第 7 步：先做静态检查，确认规则文件本身没有写错

**方式：终端操作。目的：把语法错误与业务规则错误分开。**

每次重新验收创建新的子目录，保留前一次失败记录：

```bash
export CMR_ATTEMPT
CMR_ATTEMPT="$(mktemp -d "$CMR_EVIDENCE/check-XXXXXXXX")"
printf 'CMR_ATTEMPT=%s\n' "$CMR_ATTEMPT"
python3 - <<'PY' 2>&1 | tee "$CMR_ATTEMPT/static-check.log"
from pathlib import Path
import ast
import json
from jsonschema import Draft202012Validator

for name in ["tools/validate_run.py", "tests/test_run_schema.py"]:
    p = Path(name)
    ast.parse(p.read_text(encoding="utf-8"), filename=name)
    print("Python syntax OK:", name)
def reject_constant(value):
    raise ValueError(f"Nonstandard JSON number: {value}")
def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate schema key: {key}")
        result[key] = value
    return result
for kind in ["scenario", "manifest", "result"]:
    p = Path("experiments/schemas") / f"{kind}.schema.json"
    schema = json.loads(
        p.read_text(encoding="utf-8"),
        parse_constant=reject_constant,
        object_pairs_hook=unique_object,
    )
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        raise SystemExit(f"Wrong or missing schema dialect: {p}")
    Draft202012Validator.check_schema(schema)
    print("Schema syntax/meta-schema OK:", p)
PY
git diff --check
```

预期：两个 Python 文件语法通过，三个 Schema 元校验通过，命令退出码为 0。

这只证明规则文件自身基本有效。它不能证明 `collision=false` 会按合同被拒绝，更不能证明机器人没撞；这要分别由第 8 步和后续真实评价解决。

**本步完成标准：**静态检查通过，没有靠跳过有问题的 Schema 获得绿色结果。

## 第 8 步：执行单元测试，确认合同真的能拦住错误

**方式：终端操作。目的：验证第 5 步的独立预期。**

### 8.1 先发现测试，再执行

```bash
cd "$CMR_WS"
python3 -m pytest -p no:cacheprovider --collect-only -q \
  tests/test_run_schema.py \
  2>&1 | tee "$CMR_ATTEMPT/tests-collected.log"
python3 -m pytest -p no:cacheprovider -v \
  tests/test_run_schema.py \
  --junitxml="$CMR_ATTEMPT/schema-tests.xml" \
  2>&1 | tee "$CMR_ATTEMPT/schema-tests.log"
```

不能把“0 tests”当成通过；pytest 没有收集到测试的正常退出码为 5。[pytest 7.4 退出码说明](https://docs.pytest.org/en/7.4.x/reference/exit-codes.html)

检查 JUnit，要求本步没有被跳过的必测项：

```bash
python3 - <<'PY'
from pathlib import Path
import os
import xml.etree.ElementTree as ET

p = Path(os.environ["CMR_ATTEMPT"]) / "schema-tests.xml"
root = ET.parse(p).getroot()
cases = root.findall(".//testcase")
bad = [
    case.attrib.get("name", "<unnamed>")
    for case in cases
    if any(case.find(tag) is not None for tag in ["failure", "error", "skipped"])
]
print("Executed cases:", len(cases))
print("Failed/error/skipped:", bad)
if not cases or bad:
    raise SystemExit(1)
PY
```

JUnit 有案例仍不等于覆盖了整份合同。对照 `docs/run_schema_test_design.md`，每个 ID 必须能对应到一个或多个实际测试。

### 8.2 至少独立检查三个反例

直接阅读对应测试，亲自解释：

- `succeeded_unknown` 为什么合法？为什么它没有碰撞率结论？
- 同一个数据只把 collision 从 null 改成 false，为什么必须拒绝？
- manifest 合法但 result 缺失，为什么是 INCOMPLETE，而不是算法失败？

对于非法数字测试，确认预期失败是数字规则，而不是因为同时缺 run_id。必要时用 `-k` 单独运行测试名定位；不用删除其他测试。

**本步完成标准：**所有必测案例实际运行且通过，测试没有用宽泛异常或跳过标记掩盖问题。

## 第 9 步：实际从命令行验证文件，检查退出码和只读性

**方式：终端操作。目的：证明解析、Schema、跨文件检查和 CLI 已连接起来。**

这是本步的**文件级集成验证**。它不属于 ROS/Nav2/Gazebo 集成验证。

### 9.1 用一个检查脚本执行全部预定 CLI 案例

下面的脚本启动实际命令行工具，不调用其内部函数；它也处理预期的非零退出码，避免 `set -e` 提前结束验收。脚本写入的是本次证据目录里的输出日志，不修改被校验文件。

```bash
python3 - <<'PY' 2>&1 | tee "$CMR_ATTEMPT/cli-matrix.log"
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import tempfile

root = Path(os.environ["CMR_WS"])
out = Path(os.environ["CMR_ATTEMPT"])
tool = root / "tools/validate_run.py"
fixtures = root / "tests/fixtures"
scenario = root / "experiments/scenarios/official_static.yaml"

def input_hashes():
    paths = [p for p in fixtures.rglob("*") if p.is_file()]
    paths += [scenario]
    return {
        str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in paths
    }

before = input_hashes()
matrix = [
    ("draft", ["scenario", str(scenario)], 0, "VALID", None),
    ("draft-ready", ["scenario", str(scenario), "--require-ready"],
     4, "NOT_READY", "SCENARIO_NOT_READY"),
    ("fixture-blocked", ["run", str(fixtures / "runs/succeeded_unknown")],
     2, "INVALID", "FIXTURE_NOT_ALLOWED"),
]
for name in [
    "succeeded_unknown", "nav_timeout", "startup_failed",
    "collision_observed_partial", "collision_false_scoped",
]:
    matrix.append((
        name, ["run", str(fixtures / "runs" / name), "--allow-fixtures"],
        0, "VALID", None,
    ))
matrix += [
    ("incomplete",
     ["run", str(fixtures / "runs/incomplete"), "--allow-fixtures"],
     3, "INCOMPLETE", "MISSING_RESULT"),
    ("invalid-collision",
     ["run", str(fixtures / "runs/invalid_collision_false"), "--allow-fixtures"],
     2, "INVALID", "COLLISION_UNSUPPORTED"),
]
failures = []
with tempfile.TemporaryDirectory(prefix="cmr-p02-cli-") as external_cwd:
    for name, args, code, status, expected_error in matrix:
        try:
            p = subprocess.run(
                [sys.executable, str(tool), *args],
                cwd=external_cwd, text=True, capture_output=True, timeout=15,
            )
            (out / f"cli-{name}.stdout.json").write_text(p.stdout, encoding="utf-8")
            (out / f"cli-{name}.stderr.txt").write_text(p.stderr, encoding="utf-8")
            data = json.loads(p.stdout)
            actual_codes = {e["code"] for e in data["errors"]}
            ok = (
                p.returncode == code
                and data["status"] == status
                and data["validation_scope"] == "record_contract"
                and (expected_error is None or expected_error in actual_codes)
            )
            print(name, "PASS" if ok else "FAIL",
                  "exit=", p.returncode, "status=", data.get("status"))
            if not ok:
                failures.append(name)
        except (subprocess.TimeoutExpired, ValueError, KeyError, TypeError) as exc:
            print(name, "FAIL", type(exc).__name__, str(exc))
            failures.append(name)
after = input_hashes()
if before != after:
    failures.append("input-files-changed")
    print("FAIL: input file set or hashes changed")
(out / "cli-input-hashes.json").write_text(
    json.dumps({"before": before, "after": after}, ensure_ascii=False, indent=2)
    + "\n", encoding="utf-8",
)
if failures:
    raise SystemExit("CLI integration failed: " + ", ".join(failures))
print("All CLI contract checks passed; no robot experiment was run.")
PY
```

本矩阵预期正式 `official_static.yaml` 仍为 draft。若此前你已经在其他工作中完成了真实 P0-03 并有证据，不把它改回 draft：保留真实文件，将前两项改为一个**专门的 draft fixture**，并在报告记录这一适配。

### 9.2 阅读诊断内容，而不只看 PASS

```bash
cat "$CMR_ATTEMPT/cli-succeeded_unknown.stdout.json"
cat "$CMR_ATTEMPT/cli-incomplete.stdout.json"
cat "$CMR_ATTEMPT/cli-invalid-collision.stdout.json"
```

应看见：

- 合法合成样本明确标注 fixture，校验范围是 record_contract。
- INCOMPLETE 提示缺 result，原目录仍然缺 result，没有被工具补写。
- 无依据 false 指出 `evaluation.collision` 等相关字段及原因，不只输出一个笼统的“JSON 错误”。

**本步完成标准：**CLI 矩阵通过，外部工作目录可运行，输入内容与文件集合没有变化。

## 第 10 步：把本机官方资源登记到场景草稿

**方式：只读调查 + 文档编辑。目的：让后续 P0-03 从真实资源开始，避免凭空写坐标或建新地图。**

第 6 步可以先创建资源为 null 的合法 draft；现在根据本机安装内容补齐能静态确认的信息。没有新增 Schema 语义需求时，不改字段设计。

### 10.1 读取官方 launch，核对默认资源

```bash
sed -n '30,225p' \
  /opt/ros/jazzy/share/nav2_bringup/launch/tb3_simulation_launch.py \
  | tee "$CMR_ATTEMPT/official-launch-source.txt"
```

本次编写时读到的默认资源：

| 用途 | 已安装路径 |
|---|---|
| 世界 | `/opt/ros/jazzy/share/nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro` |
| 地图 YAML | `/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml` |
| 地图图像 | 地图 YAML 引用同目录 `tb3_sandbox.pgm` |
| 仿真机器人 | `/opt/ros/jazzy/share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro` |
| robot_state_publisher 描述 | `/opt/ros/jazzy/share/nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf` |

本机包版本曾读取为 nav2_bringup 1.3.13、nav2_minimal_tb3_sim 1.0.1。执行时仍以本机文件为准；若路径不同，先读当前 launch，再调整下方采集清单。

这些是官方 launch 的**静态默认值**，并不证明下一次运行没有通过 launch 参数覆盖它们。

### 10.2 记录资源指纹和地图元数据

下面不启动 launch，不展开世界，不下载模型，只读取已有文件：

```bash
python3 - <<'PY' | tee "$CMR_ATTEMPT/official-resources.json"
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import yaml

share = Path("/opt/ros/jazzy/share")
paths = {
    "world": share / "nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro",
    "map_yaml": share / "nav2_bringup/maps/tb3_sandbox.yaml",
    "robot": share / "nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro",
    "robot_state_publisher_urdf":
        share / "nav2_minimal_tb3_sim/urdf/turtlebot3_waffle.urdf",
}
for name, p in paths.items():
    if not p.is_file():
        raise SystemExit(f"Missing {name}: {p}; inspect installed launch first.")
metadata = yaml.safe_load(paths["map_yaml"].read_text(encoding="utf-8"))
image_name = Path(metadata["image"])
paths["map_image"] = (
    image_name if image_name.is_absolute()
    else paths["map_yaml"].parent / image_name
).resolve()
resources = {}
for name, p in paths.items():
    if not p.is_file() or p.stat().st_size == 0:
        raise SystemExit(f"Missing or empty resource: {p}")
    resources[name] = {
        "path": str(p),
        "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
        "bytes": p.stat().st_size,
    }
versions = {
    name: ET.parse(share / name / "package.xml").getroot().findtext("version")
    for name in ["nav2_bringup", "nav2_minimal_tb3_sim"]
}
print(json.dumps({
    "record_type": "static_resource_inventory",
    "resources": resources,
    "package_versions": versions,
    "map_metadata": metadata,
    "dependency_closure_verified": False,
    "world_map_alignment_verified": False,
    "poses_free_verified": False,
    "runtime_interfaces_verified": False,
}, ensure_ascii=False, indent=2, allow_nan=False))
PY
```

这个采集脚本读取的是已安装官方文件；真正处理实验输入的严格加载器仍按第 5 步实现，不能用这里的普通 `yaml.safe_load` 代替它。

本机地图 YAML 曾显示 resolution=0.05、origin=[-10,-10,0]。它们只是地图元数据，**不是**机器人可通行证明。后续还要检查图像方向、世界坐标与 map 坐标关系、机器人足迹和实际激光。

### 10.3 只填写静态确定的字段

由 AI 或你自己增量更新 `experiments/scenarios/official_static.yaml`：

- resources 中的真实 path、hash、package 和 package_version 可以填写。
- `dependency_lock` 暂为 null，`resource_dependencies` 仍 pending，除非确实完成 xacro/include/模型依赖清单。
- `initial_pose`、`goal_pose` 未通过真实场景确认时保持 null。
- 不能仅因为 launch 写有默认出生点，就将 `poses_free` 标为 checked。
- 未运行话题调查时 interfaces 为空，`interfaces_observed` 为 pending。
- world_map_alignment 保持 pending；地图和世界同名不等于几何一致。
- 静态场景 dynamic_events=[]；evaluation_scope 可为 null。
- status 仍为 draft。

“主世界文件 hash”没有覆盖它引用的所有模型；地图 YAML hash 也没有覆盖地图图像。后续锁定实验资源时需要对应依赖清单。P0-02 将这一缺口显式记录即可，不把完整资产打包系统作为当前前置。

### 10.4 用最终场景文件再验证一次

由于刚修改了场景文件，重跑受影响的场景与 CLI 检查有实际必要。新建一次验收目录，重新执行第 7–9 步，作为最终结果；不要覆盖第一次的日志。

**本步完成标准：**官方资源有真实静态来源，场景草稿能校验，未测坐标、TF、话题和碰撞没有被填成“已确认”。

## 第 11 步：核对最终文件与个人理解

**方式：终端检查 + 本人验收。目的：证明增加了本步能力，同时保留原有成果。**

### 11.1 检查受保护文件没有变化

```bash
cd "$CMR_WS"
python3 - <<'PY' | tee "$CMR_ATTEMPT/protected-check.log"
from pathlib import Path
import hashlib
import json
import os

root = Path(os.environ["CMR_WS"])
out = Path(os.environ["CMR_EVIDENCE"])
before = json.loads((out / "protected-before.json").read_text(encoding="utf-8"))
changed = []
for name, expected in before.items():
    p = root / name
    actual = hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
    if actual != expected:
        changed.append(name)
if changed:
    raise SystemExit("Protected files changed; inspect, do not reset: " + ", ".join(changed))
print("All protected files unchanged:", len(before))
PY
git diff --check
git status --short | tee "$CMR_ATTEMPT/git-status-after.txt"
git diff --binary > "$CMR_ATTEMPT/unstaged-after.patch"
git diff --cached --binary > "$CMR_ATTEMPT/staged-after.patch"
```

若指纹不同，先确认是否是其他已授权工作，不能自动把用户改动恢复掉。通常本步不需要改任何受保护文件。

记录本步最终源码、Schema、场景、fixture 和测试指纹：

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib
import json
import os

root = Path(os.environ["CMR_WS"])
paths = set()
for relative in [
    "experiments/schema.md", "experiments/schemas",
    "experiments/scenarios/official_static.yaml",
    "tools/validate_run.py", "tests/test_run_schema.py", "tests/fixtures",
    "docs/run_schema_test_design.md",
]:
    p = root / relative
    if not p.exists():
        raise SystemExit(f"Missing deliverable: {relative}")
    if p.is_dir():
        paths.update(f for f in p.rglob("*") if f.is_file()
                     and "__pycache__" not in f.parts)
    else:
        paths.add(p)
hashes = {
    str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in sorted(paths)
}
out = Path(os.environ["CMR_ATTEMPT"]) / "tested-files-sha256.json"
out.write_text(json.dumps(hashes, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("Final tested-file snapshot:", out)
PY
```

保存指纹后若再改代码、Schema 或 fixture，重新执行受影响检查并保存新的指纹，不能给新文件配旧日志。

### 11.2 你必须自己能回答

关闭 AI 提示后，至少讲清楚：

1. ScenarioSpec、RunManifest、RunResult 为什么分开？
2. draft 合法与 ready 的区别是什么？P0-03 为什么可以从 draft 开始人工调查？
3. 为什么 Action 成功不能自动填 collision=false 或 delivery_success=true？
4. 超时后取消、取消未确认、清理失败分别记录在哪里？
5. 为什么缺 result 是 INCOMPLETE，而损坏 result 是 INVALID？
6. 为什么只检查 JSON 语法和 SHA-256 不足以证明真实评价有效？
7. 哪个测试会抓到 `bool(None)`、NaN、插件身份串错和输入被修改？
8. 为什么已观测遮挡是 Master S3，而不是看到旧名 `s2_occluded` 就写 S2？

不要求你手写所有重复代码。要求你能找到负责这条规则的实现和测试，解释一个失败诊断，并在规则改变时判断哪些测试必须更新。

**本步完成标准：**保护范围、最终验收版本和个人理解均如实记录，没有把 AI 的通过结论当成自己的理解。

## 第 12 步：写完成报告，进入 P0-03

**方式：文档编辑。目的：给下一步留下一份简洁、可信的交接。**

报告路径使用 `docs/verification/<CMR_CHECK_ID>.md`，由 AI 整理，你核对后定稿。报告至少包括：

- 当前 HEAD、dirty 状态、本次编号、最终 `CMR_ATTEMPT`、被测试文件 hash。
- 新增/修改文件清单。
- 规则版本与重要决定：三态、终态、fixture、时间域、草稿/ready。
- S/R/E/T/P/C 测试 ID 对应的执行结果与日志链接。
- 官方资源静态清单，以及尚未完成的依赖、位姿、地图对齐与话题调查。
- 局限：只证明合同与引用一致性；不验证场景物理真实性、bag 消息内容、碰撞算法或导航性能。
- 个人已掌握和仍需练习的内容，不能代打勾。

### 12.1 验证层级必须这样分开写

| 验证层级 | 完成 P0-02 时应如何填写 |
|---|---|
| 文件已实现 | 三份 Schema、校验器、fixture、测试和文档存在 |
| 静态检查 | Python 语法、Schema 元校验、差异检查实际通过 |
| 单元测试 | 必测案例实际执行，失败/错误/跳过为零，按 ID 对应 |
| 文件级集成 | 实际 CLI 的解析、跨文件检查、退出码和只读性通过 |
| 官方资源调查 | 已读取资源和指纹；几何/TF/话题仍待运行验证 |
| ROS2/Nav2/Gazebo 集成 | 本步未执行，不写通过 |
| 碰撞评价与算法性能 | 本步未执行；fixture 不是有效实验结果 |
| 实车 | 本步未执行 |

### 12.2 完成清单

- [ ] 仍在 `/home/robot/commercial_mobile_robot`，没有重建仓库。
- [ ] 已有核心、测试、CMake 和双基线配置保留，受保护指纹一致。
- [ ] 三种记录的职责、字段、单位和版本已明确。
- [ ] 官方静态场景草稿有真实资源来源，没有编造运行位姿或接口。
- [ ] 横穿和已观测遮挡能以 draft 表达，Master 场景映射正确。
- [ ] fixture 与真实实验严格区分，默认不会被当作真实记录接受。
- [ ] 合法成功、启动失败、超时、缺测和未完成均可如实表示。
- [ ] 无证据 collision=false 被拒绝；未知值没有被补成零或 false。
- [ ] 非有限数、重复键、错误类型和跨文件不一致都有负例。
- [ ] CLI 从外部 cwd 可运行，能输出稳定诊断，输入文件没有被修改。
- [ ] 静态、单元、文件级集成分别有真实日志，最终 hash 对应测试版本。
- [ ] 完成报告写清本步没有运行导航或验证碰撞性能。
- [ ] 本人能够解释第 11.2 节的问题，待掌握项如实保留。

满足这些条件即可标记 **P0-02 完成**。不要求官方场景在本步 ready，不要求先跑 10 回合，也不要求自行绘制地图、开发动态障碍或几何评价器。

下一步是 **P0-03：官方世界的一次最小导航与定向排障**：

1. 确认现有 Jazzy/Harmonic 官方启动路径和配套资源。
2. 检查仿真时钟、激光、odom 与基础 TF。
3. 在 RViz 给出与出生点一致的初始位姿，解决真实的 AMCL/map TF 问题。
4. 使用已有 MPPI 配置完成一个人工选择的短目标。
5. 保存 Action 终态、运行插件、速度链和停止观测，用本步合同手工记录。

第一次真实导航后，若发现合同确实漏了一个必要字段，允许一次明确版本的增量修订；不用为了“以后可能用到”预先建庞大平台。MPPI/DWB 双控制器正式验收在 P0-04，单回合自动化在 P0-05。

## 附录 A：遇到错误时按这个顺序排查

| 现象 | 先检查什么 | 正确处理 |
|---|---|---|
| import jsonschema/yaml/pytest 失败 | `python3` 实际解释器和 3.1 日志 | 只补缺失依赖，避免装到另一个 Python |
| Schema 元校验失败 | `$schema`、type、required、if/then、`$defs` | 修复 Schema 结构，不跳过 check_schema |
| 本地运行可以，换 cwd 失败 | 是否相对当前目录寻找 schemas | 根据工具自身 `__file__` 定位仓库 |
| 所有非法输入都“通过” | catch-all、默认值、返回码 | 找到被吞掉的错误，增加对应负例 |
| 无依据 false 被其他原因先拒绝 | 负例是否从合法输入单项修改 | 隔离问题，并断言 COLLISION_UNSUPPORTED |
| 合法启动失败被拒绝 | READY 和 FAILED 是否共用强制插件规则 | 对启动失败允许 runtime null，保留失败证据 |
| missing result 被记成 INVALID | manifest 自身是否先不合法 | 先修合法 manifest；只有合法身份缺 result 才是 INCOMPLETE |
| 草稿被 require-ready 拒绝 | 是否原本预期 NOT_READY | 这是正确结果，不填假坐标凑 ready |
| YAML 中值类型怪异 | 未加引号的日期、yes/no、数值 | 字符串显式加引号；不做静默强转 |
| JSON format 写了但错误日期仍过 | format 默认检查和实际日期解析 | 明确日期验证，补不可能日期用例 |
| hash 对不上 | 文件字节是否变更、引用是否串回合 | 确认版本与引用，不能随便改 hash 掩盖变更 |
| pytest 0 tests 或跳过 | 文件名、测试函数名、skip 标记 | 修测试发现或真实依赖，不能计通过 |
| 受保护指纹变化 | 第 2 步基准与并行改动 | 查明原因，保留双方，不自动 reset |
| 终端重开变量丢失 | 第 2、7 步保存的真实目录 | 恢复准确变量，不重新生成编号混用日志 |

变量恢复示意：将下面两个占位字符串换成你屏幕上记录的真实目录名，再执行。不要直接照抄占位字符串。

```bash
export CMR_WS=/home/robot/commercial_mobile_robot
export CMR_EVIDENCE="$CMR_WS/evidence/env/替换为本次p0-02编号"
export CMR_CHECK_ID
CMR_CHECK_ID="$(basename "$CMR_EVIDENCE")"
export CMR_ATTEMPT="$CMR_EVIDENCE/替换为本次check编号"
export PYTHONDONTWRITEBYTECODE=1
cd "$CMR_WS"
test -f "$CMR_EVIDENCE/protected-before.json"
test -d "$CMR_ATTEMPT"
set -eo pipefail
```

## 附录 B：本指南的证据与资料边界

项目目标仍以已审阅的 Master Plan 为准：P0 提供可信基础与可追溯基线，不改变后续观测年龄校准、风险感知 MPPI 和执行一致性的研究主线。本文细化的是重规划路线的 P0-02，并未增加新的研究方向。

本指南按你的 P0-01 顺序执行版组织；从该文档继承的是“先检查、再设计、AI 实现后审查、分层测试、最后报告”的方法，没有机械复制本步用不上的 Release/sanitizer/colcon 操作。

编写时对 Ubuntu 只进行了文件、日志、包元数据和已有资源的只读检查。已有 P0-01 报告仍包含旧停点，后续日志已经存在；本指南不据此否定你已经完成 P0-01，也不把只读日志检查称为本轮重测。

本文中的 CLI、Schema 和 fixture 是待实施合同；本次交付这份 Markdown 不代表这些代码已经在项目里实现。正式实现和验收由你按第 1–12 步推进。

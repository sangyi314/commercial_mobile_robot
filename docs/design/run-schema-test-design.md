# P0-02 校验器测试设计（待审查草稿）

日期：2026-10-21（Asia/Shanghai）。依据顺序执行版第 5.8 节；本表保留全部 26 个案例 ID。此时只有源码与 fixture 草稿，**没有运行语法/Schema 元校验、pytest 收集/执行或 CLI 集成，不填写通过结论**。

生产代码为 tools/validate_run.py，测试为 tests/test_run_schema.py，三份 Schema 与字段语义见 [schema.md](../../experiments/schema.md)。位置是本草稿文本行号，运行证据待负责人审查后按第 7–11 步填写；表内预期不是工具计算所得。

| ID | 独立预期 / 单项变更 | 测试源码锚点 | 实现入口 | 执行证据 |
|---|---|---|---|---|
| S01 | draft 位姿 null 普通 VALID；require-ready NOT_READY | `test_S01_official_draft:91` | `validate_scenario:396` | 未执行 |
| S02 | 横穿 S1、遮挡 S3 VALID；错映射 S2 INVALID | `test_S02_dynamic_draft:99` | `scenario_rules:176` | 未执行 |
| S03 | 缺版本/空或空白 ID/未知字段枚举/bool 或 float 版本 INVALID | `test_S03_structure:120`, `test_S03_resource_uri_not_implemented:128` | `validate_structure:120` | 未执行 |
| S04 | 声称 ready 却缺必要条件 INVALID | `test_S04_false_ready:169` | `scenario_rules:176` | 未执行 |
| S05 | 临时 synthetic 静态 ready VALID；锁漏资源/未解析项/坏 hash/空准备证据拒绝 | `test_S05_synthetic_ready:177`, `test_S05_ready_evidence_integrity:192` | `scenario_rules:176` | 未执行 |
| R01 | SUCCEEDED/true，collision/delivery null，VALID | `test_R01_success_unknown:210` | `result_rules:344` | 未执行 |
| R02 | NAV_TIMEOUT 保留 CANCELED/false 或 UNKNOWN/null，VALID | `test_R02_timeout:220` | `result_rules:344` | 未执行 |
| R03 | 启动 FAILED/NOT_SENT/null，可 runtime=null 或空成员对象，VALID | `test_R03_startup_failure:231` | `result_rules:344` | 未执行 |
| R04 | 合法 manifest 缺 result INCOMPLETE，不写回 | `test_R04_incomplete_readonly:240` | `validate_run_dir:411` | 未执行 |
| R05 | 坏 result、ID/来源/hash 不符或结构错误 INVALID | `test_R05_bad_existing_result:249`, `test_R05_record_structure:360` | `validate_run_dir:411` | 未执行 |
| R06 | 实际/预期插件、method 不匹配或缺参数引用 INVALID | `test_R06_runtime_identity:262` | `result_rules:344` | 未执行 |
| R07 | 所有 action 映射独立真值；不匹配 INVALID；清理失败保留成功终态、理由必需 | `test_R07_terminal_mapping:276`, `test_R07_cleanup_keeps_terminal:287`, `test_R07_success_is_not_nav_timeout:298` | `result_rules:344` | 未执行 |
| R08 | dirty 缺快照、坏引用/hash/逃逸/重复 ID/错场景拒绝；实际 synthetic patch 接受；归档不回访外机路径 | `test_R08_artifact_integrity:310`, `test_R08_dirty_snapshot_present:332`, `test_R08_archived_scenario_does_not_access_other_host:343` | `manifest_rules:255` | 未执行 |
| E01 | 未评价的 collision=null 合法且只读 | `test_E01_unknown_preserved:369` | `evaluation_rules:297` | 未执行 |
| E02 | 只把 null 改 false，COLLISION_UNSUPPORTED | `test_E02_false_without_evidence:377` | `evaluation_rules:297` | 未执行 |
| E03 | 可信合成事件 true/partial 合法；错 epoch/时刻/对象/来源拒绝 | `test_E03_observed_partial:384`, `test_E03_event_boundaries:426` | `evaluation_rules:297` | 未执行 |
| E04 | full/有效/有证据 scoped false 合法；缺完整范围和引用拒绝 | `test_E04_full_scoped_false:392`, `test_E04_false_requires_full_scope:403` | `evaluation_rules:297` | 未执行 |
| E05 | 局部几何无接触、负间距仍可 collision=null；不改值 | `test_E05_geometry_does_not_infer_collision:416` | `evaluation_rules:297` | 未执行 |
| T01 | epoch、仿真起止、单调时长和不可能 UTC 日期拒绝 | `test_T01_time_domains:440` | `result_rules:344` | 未执行 |
| T02 | rtf null 和同窗口合法；零分母/错比值/混窗口/缺 sim 拒绝 | `test_T02_rtf_window:451` | `result_rules:344` | 未执行 |
| P01 | JSON NaN/±Infinity/±1e309，YAML 非有限/指数溢出递归拒绝 | `test_P01_json_nonfinite:467`, `test_P01_yaml_nonfinite:477` | `load_strict:91` | 未执行 |
| P02 | JSON/YAML 重复键、非字符串键、merge、危险 tag、递归别名拒绝 | `test_P02_strict_parsing:486` | `load_strict:91` | 未执行 |
| P03 | bool 不冒充数值，字符串 false 拒绝，null 原样保留 | `test_P03_bool_is_not_number:500`, `test_P03_string_false_and_null:506` | `validate_structure:120` | 未执行 |
| C01 | 真实 subprocess：默认 fixture 拒绝；允许 fixture 仍检查坏碰撞 | `test_C01_fixture_default_rejected:516` | `main:446` | 未执行 |
| C02 | 真实 subprocess：外部 cwd、七目录退出码、只读指纹、缺结果不写回、scenario 0/4 | `test_C02_external_cwd_readonly:531`, `test_C02_scenario_exitcodes:544` | `main:446` | 未执行 |
| C03 | 真实 subprocess：缺/空证据、缺输入、坏 JSON/参数均非零 JSON；内部异常退出 1 | `test_C03_io_errors:557`, `test_C03_argument_and_internal_errors:569` | `main:446` | 未执行 |


## 测试独立性与隔离

- fixtures 的 hash 与 manifest 绑定由 stdlib 独立计算；测试不调用被测函数制造正确预期。合法样本复制进 pytest 临时目录，负例单项修改；修改 manifest 身份外的字段时同步重算其绑定 hash，避免旧 hash 抢先掩盖目标规则。
- 每个负例断言状态以及正确 code 或字段；无依据 false 只修改 collision 一项，必须出现 COLLISION_UNSUPPORTED，不能因缺字段碰巧拒绝。日期用真实不可能日期，时间窗用保持比值但改变窗口的反例。
- 固定 fixture 只读；测试内 unlink、写入、符号链接只作用于临时副本，原始 seven runs 不修改。C02 对所有七种输入逐文件 SHA256 前后比较，故意缺 result 的目录必须保持缺失。
- cli helper 启动真实 Python 子进程，要求 stdout 能解析为唯一 JSON 对象、stderr 无异常输出；核对 0/2/3/4，内部异常 mock 用 main 确认退出 1。脚本位置从 __file__ 定位，不依赖当前 cwd。
- S05 资源/人工证据全为 synthetic；ready 只检查声明合同，不能证明物理世界准备完毕。E04 的 false 只属于 synthetic-disc-scope，不用于实车碰撞率。

## 审查后证据填写

沿用准确 evidence/env/p0-02-RaLo7r83，每次验收新建 check 子目录，不覆盖旧失败记录。第 7 步做 Python 语法与三个 Schema 元校验；第 8 步 collect 与 pytest/JUnit，实际条目非零、无失败/错误/跳过；第 9 步外部 CLI、退出码与只读性；第 10 步官方资源只读清单并更新 draft 后重新验受影响项；第 11 步保护指纹、差异、最终被测试文件 hash。

当前实现后 hash 只是审查草稿指纹，不是 tested-files-sha256，不与 P0-01 或历史日志拼接。官方场景仍为 draft、null 位姿/资源和 pending 检查；第 10 步静态资源调查尚未执行。P0-02 不启动 ROS/Nav2/Gazebo/RViz、不录包、不访问实车、不验证数据内容真实性或算法性能。

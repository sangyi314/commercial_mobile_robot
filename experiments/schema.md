# ScenarioSpec / RunManifest / RunResult v1（P0-02 审查草稿）

依据：随附《Phase_0_P0-02_顺序执行版.md》第 4–5 步及本轮实施任务；2026-10-21（Asia/Shanghai）。本阶段文件已起草，步骤 7–11 尚未执行，未宣称 P0-02 验收完成。

## 共通结构与严格解析

三份 JSON Schema 使用 Draft 2020-12，各自自包含，只引用文件内 $defs，不联网。以下列出的对象键全部必需，所有固定对象 additionalProperties=false；唯一任意键映射为 environment.package_versions（非空包名→非空版本字符串）。ID/标签字符串至少有一个非空白字符，sha256 为 64 位、commit 为 40 位十六进制；hash 按文件实际字节比较，允许十六进制大小写。

schema_version 的 Schema 同时有 type=integer、const=1；工具使用实际 Python int 类型检查，拒绝 true 和 1.0，不把整数值浮点数作为整数记录字段。其余 integer 字段同样不接收 bool/float。数值只接受有限 int/float，不互转字符串、布尔或 null；未知保留 null，不补默认值。

JSON 拒绝重复键、NaN/±Infinity；递归拒绝非有限浮点数（含 1e309 溢出）。YAML 使用 SafeLoader 严格子类，拒绝重复键、merge key、非字符串键、任意 Python 对象、递归别名和日期等非 JSON 类型；补充识别无小数点的指数数字，因此 YAML 1e309 也不能躲成字符串。未加引号的日期若被加载为日期对象将被拒绝。UTC 字符串显式加引号。

## 场景及小对象

| 字段 | 类型与含义 |
|---|---|
| schema_version | 实际整数 1 |
| scenario_id / scenario_revision | 非空字符串，内容 revision 与格式版本独立 |
| kind / master_scenario_id | static_smoke→S0，crossing→S1，head_on→S2，observed_occlusion→S3 |
| legacy_id | 字符串或 null；s2_occluded 映射 S3 |
| status | draft / ready_for_manual_smoke |
| resources | world/map_yaml/map_image/robot/dependency_lock 各为 ResourceRef 或 null |
| task | initial_pose/goal_pose 为 Pose 或 null；common_constraints_ref 为 ResourceRef 或 null |
| interfaces | Interface 数组，可空 |
| dynamic_events | DynamicEvent 数组，event_id 唯一；static_smoke 必须空 |
| evaluation_scope | Scope 或 null |
| verification | resource_dependencies/world_map_alignment/poses_free/interfaces_observed 各 pending/checked；evidence_refs 是本机文件路径数组 |

- ResourceRef=`{path,sha256,package,package_version}`。path 为绝对路径或相对项目根的路径，不使用 URI；sha256 可 null，package/package_version 可 null。draft 没有 hash 时不承诺文件已就位；声明 hash 时检查存在、非空和字节。场景资源允许宿主路径，不套用 run 的路径限制。
- Pose=`{frame_id,x_m,y_m,yaw_rad}`，frame_id 非空，x/y 单位 m，yaw 单位 rad，数值有限；初始位姿和目标均可未知。
- Interface=`{role,topic,type,frame_id,time_domain,usage}`。前四项为非空字符串；time_domain 为 ros_sim/monotonic，usage 为 algorithm_input/evaluation_only。接口调查尚未做时用空数组。
- DynamicEvent=`{event_id,actor_id,trigger,motion_ref}`；后三项可 null。非 null actor_id/motion_ref 为非空字符串；trigger=`{time_domain:"ros_sim",at_s:非负有限秒}`。只预留接口，不运行事件。
- Scope=`{scope_id,objects,geometry_model,truth_source_ref}`；scope_id 非空，objects 为唯一字符串数组，geometry_model/truth_source_ref 可 null。未确认整个 scope 可 null。
- verification.evidence_refs 是相对项目根或绝对本机路径，引用必须非空存在；工具不据文件名将 pending 改为 checked。

ready 必须是 static_smoke，四个主资源、dependency_lock 和 common_constraints_ref 均有实际 hash；两个位姿存在；四个检查 checked 且有非空准备证据。自己声称 ready 却不满足则 INVALID；合法 draft 普通 VALID，require-ready 时 NOT_READY。动态类别 v1 只接受 draft 的准备状态。

指南要求依赖清单有解析路径/hash、未解析项保留待办，但未指定文件内部编码。本草稿将其最小编码明确为 JSON/YAML `{resources:[{path,sha256},...],unresolved:[]}`：键固定、已解析资源至少一项且路径唯一；每项检查文件，必须覆盖四个主资源。unresolved 非空不能 ready。这项具体编码供负责人审查，不声称自动解析 xacro/include 或证明依赖闭包真实完整。

独立场景校验会检查声明的宿主资源/准备证据；归档 run 中的场景快照只严格检查内容、ready 声明条件和身份，不回访另一台机器的路径或读取其 dependency_lock。归档资源完整性由原准备记录负责，不能从快照校验推断。

## RunManifest v1

| 对象/字段 | 必需成员、类型与单位 |
|---|---|
| 根 | schema_version=1，run_id，record_origin=real/fixture，scenario/source/baseline/environment/clock/artifacts |
| scenario | scenario_id，scenario_revision，spec_sha256 |
| source | git_commit（40 hex），dirty（bool），snapshot_ref（ArtifactRef ID 或 null） |
| baseline | method=mppi/dwb，config_sha256，expected_plugin，common_constraints_sha256 |
| environment | ros_distro/platform 非空；package_versions 映射；external_seed 整数或 null；seed_scope 唯一字符串数组；controller_seed_controlled bool |
| clock | clock_epoch 非空；started_at_utc 有效 UTC 字符串且以 Z 结尾，实际解析日期；sim_start_s 有限秒或 null |
| artifacts | ArtifactRef 数组 |

ArtifactRef=`{id,path,sha256,kind}`：id 在各自数组内唯一；path 是相对 run 目录的文件路径，拒绝绝对路径、驱动器路径、反斜杠歧义、.. 和符号链接解析后逃离 run 的路径；文件非空且 hash 匹配。kind 为 scenario/config/common_constraints/source_snapshot/runtime_parameters/log/bag_metadata/evaluation/other。引用按同一记录数组查找，不串用 manifest 与 result 的 ID。

manifest 中 scenario/config/common_constraints 三种各且仅一份，对应 hash 与身份字段一致。场景快照严格解析并校验结构、语义、scenario_id/revision。dirty=true 要求 snapshot_ref 指向实际非空 source_snapshot 修改快照；文件与引用检查不能证明修改内容与运行源码真实一致，仍需执行者负责。真实记录不能用 synthetic 快照代替。

固定插件：mppi→nav2_mppi_controller::MPPIController，dwb→dwb_core::DWBLocalPlanner。外部 runner seed 不表示控制器全部随机过程已控制，未确认 controller_seed_controlled=false。

## RunResult v1 与终态

根键全部必需：schema_version/run_id/record_origin/manifest_sha256/status/runtime/time/metrics/evaluation/artifacts。run_id、record_origin 与 manifest 相同，manifest_sha256 匹配 manifest 实际字节。

status=`{startup,action,harness,cleanup,primary_reason}`：

- startup：READY/FAILED。
- action：NOT_SENT/REJECTED/SUCCEEDED/ABORTED/CANCELED/UNKNOWN。
- harness：COMPLETED/STARTUP_FAILED/NAV_TIMEOUT/RECORDER_FAILED/INTERRUPTED/INTERNAL_ERROR。
- cleanup：SUCCEEDED/FAILED/NOT_ATTEMPTED。
- primary_reason：非空字符串或 null；harness 非 COMPLETED、startup FAILED 或 cleanup FAILED 时必须非空。保留最先导致结束的原因，其余失败保留日志，校验器不改写原因或原终态。

metrics=`{action_success,goal_position_error_m,goal_yaw_error_rad,stop_observed,intervention,delivery_success}`。Action 映射为 SUCCEEDED→true，REJECTED/ABORTED/CANCELED→false，NOT_SENT/UNKNOWN→null。startup FAILED 只允许 NOT_SENT/null；NAV_TIMEOUT 不允许 SUCCEEDED，取消未确认用 UNKNOWN。goal_position_error_m 为非负有限米或 null；goal_yaw_error_rad 是 [0,π] 内绝对误差 rad 或 null；stop_observed/intervention 为 bool 或 null；P0 delivery_success 固定 null。

runtime 通常为 `{actual_plugin,parameters_ref}`（各字符串或 null）。**按本轮任务明确要求，启动失败还允许整个 runtime=null**；这是对指南“对象、成员可 null”描述的明确兼容选择。READY 必须为对象、actual_plugin 等于 manifest.expected_plugin，parameters_ref 指向 result.artifacts 的 runtime_parameters 文件；失败记录不强迫伪造插件。参数存在/hash 正确只证明引用一致，不证明插件实际被加载。

time=`{clock_epoch,sim_start_s,sim_end_s,elapsed_monotonic_s,rtf}`：epoch 与 manifest 相同；双方非 null 的开始值一致；开始/结束为有限仿真秒或 null，已知时 end≥start；单调时长非负有限秒。rtf=null 或 `{value,sim_delta_s,monotonic_delta_s}`：value/仿真增量非负，单调增量>0，均有限；两种增量分别对应 run 已知起止差与单调时长，value 对应比值（rel_tol=1e-9, abs_tol=1e-12）。缺仿真起止时 rtf 必须 null，不能混窗口或跨 epoch 相减。

## 评价与碰撞三态

必需键与类型：evaluator_version/scope_id/geometry_model/frame_id/clock_epoch 为字符串或 null；objects 为唯一字符串数组；interval_s 为 `{start,end}` 有限仿真秒或 null；coverage 为 none/partial/full；evaluation_valid 为 bool；invalid_reason 为字符串或 null；evidence_refs 为 result.artifacts ID 数组；collision/geometric_contact/contact_sensor_event 为 bool 或 null；min_signed_clearance_m 为有限米或 null；collision_event 为 `{at_s,objects,source_ref}` 或 null（时刻有限、对象数组非空）。

- coverage=none：评价无效且理由非空，collision/min_signed_clearance_m/geometric_contact/contact_sensor_event 全 null，event=null。
- 任何非 null 测量或有效评价都需要评价器、scope、非空 objects、几何定义、frame、同 epoch 区间与证据引用。evaluation_valid=false 必须理由非空，true 时理由为 null。
- collision=false 另需 full/有效且上述完整范围上下文；无依据 false 用 COLLISION_UNSUPPORTED。full 只指声明 scope 的时间与对象，不扩展为全物理碰撞。
- collision=true 必须有 event、可追溯 source_ref、区间内的事件时刻与属于 scope 的对象；允许 partial/无效，但保留其余时间无效原因。不因部分缺测丢失可信事件。
- 区间 start≤end，epoch 与 run 一致，不超过已知起止。evidence_refs 与 event.source_ref 均解析到 result.artifacts。
- collision=null 可与有效局部几何测量共存；负 clearance 是重叠，不是格式错误。几何无接触不自动推出物理无碰撞，未知不补成 false/0。

这些规则只校验字段、声明和引用，不重放 bag，不检查 scope/coverage 声明真实性，不执行碰撞算法。无测量时不伪造结果。

## CLI、诊断与只读承诺

```text
python3 tools/validate_run.py scenario <path> [--require-ready]
python3 tools/validate_run.py run <directory> [--allow-fixtures]
```

始终输出一个 JSON 对象，含 status、validation_scope=record_contract、record_origin（scenario 为 null）、errors（code/path/message）、warnings。按首个失败给出稳定定位，不宣称收集所有后续错误。CLI 参数错误也输出 JSON；内部意外异常输出 INTERNAL_ERROR、退出 1，不吞成 VALID。

退出码：0 VALID，2 INVALID，3 INCOMPLETE，4 NOT_READY。fixture 默认 FIXTURE_NOT_ALLOWED；allow-fixtures 只解除来源限制。合法 manifest（含结构/引用）缺 result 才 MISSING_RESULT/INCOMPLETE；已有坏 result 是 INVALID，不补写文件。合法 draft 要求 ready 时 SCENARIO_NOT_READY/NOT_READY，坏 ready 是 INVALID。身份错误为 IDENTITY_MISMATCH，无依据 false 为 COLLISION_UNSUPPORTED；其余详见工具中的稳定 code。

顺序：严格解析/基本身份→fixture 来源→结构/manifest 引用→缺 result→结果结构、身份、终态与评价→ready 条件。场景验证先检查文件合法性，再处理 require-ready。工具根据 __file__ 找项目内 Schema，从任意 cwd 使用；输入只读，不补默认值、不修复、不生成结果。未知字段/值类型不自动转换。

四个入口为 load_strict、validate_scenario、validate_run_dir、main。其他函数为内部小 helper，不作为新增框架接口。来源见 [fixture README](../tests/fixtures/README.md)，案例位置与未执行状态见 [测试设计](../docs/run_schema_test_design.md)。

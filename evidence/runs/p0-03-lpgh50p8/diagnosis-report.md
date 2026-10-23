# CMR P0-03 排障与一次验证报告

结论：**P0-03 未通过，本次未修复到可导航状态。** 已实施回合内启动顺序调整并做了一次验证；验证在定位 STARTUP 服务调用阶段失败，未创建导航进程、未发布 AMCL 初值、未发送目标。已正常清理，真实失败记录合同校验为 VALID。VALID 不代表导航成功。

- 固定项目：`/home/robot/commercial_mobile_robot`
- 本次唯一新回合：`/home/robot/commercial_mobile_robot/evidence/runs/p0-03-lpgh50p8`
- Git HEAD：`a978f22157296b9e0285650badd3a3cedfb4a410`
- 实际回合时间（北京时间）：2026-10-22 20:37:20 至 20:42:09；2026-10-23 仅继续整理记录。
- 结果：startup=FAILED；action=NOT_SENT；harness=STARTUP_FAILED；cleanup=SUCCEEDED。
- 记录：`manifest.json`、`result.json`；校验：`logs/run-final-check.json`、`logs/run-final-check.exit`。
- 所有旧回合保留；未运行 DWB、P0-04 或实车，未修改 cmr_core、Schema、validator，未提交或推送。

## 1. 根因与证据边界

用户授权参考已列出的各真实回合。本报告以 `3hg1954j`、`chnntxl6` 的初始化失败为主要对照，同时核对旧拒绝轮 `nq1hmjaf` 和导航成功但清理失败轮 `5u34h8ya`。没有根据目录时间自动替用户选择，也未使用 fixtures。

下表前缀均为 `/home/robot/commercial_mobile_robot/evidence/runs/`；文件行号指原始文件。

| 问题 | 已证实事实 | 待验证假设 | 证据文件及行号/时间 | 下一条最小检查 |
|---|---|---|---|---|
| nq1hmjaf 目标拒绝 | global_costmap 等待 base_link→map 超时，导航 bringup 中止；目标明确被拒绝 | 更深层时序原因须与初值时刻对齐；不能用于解释后两轮 | `p0-03-nq1hmjaf/logs/nav2-launch.log:498–500`；`logs/navigate-goal.log:21` | 保留定位与 map TF 就绪后激活导航的顺序；勿重调 critics |
| 3hg1954j 初始化阻断 | route_server 的 GetState 服务不可用；随后仍有 AMCL 与 map TF；未发目标 | 节点构造/发现阶段阻塞；底层原因未知 | `p0-03-3hg1954j/logs/pre-goal-audit.err:8`；`runtime/discovery-recheck.json:1081`；`runtime/initialize-summary.json:16–18,37–53` | 需要真正的线程调用栈或服务收发证据；已有堆栈采集失败 |
| chnntxl6 初始化阻断 | 导航启动服务等待超时、未发目标；UDPv4 隔离未解除问题；map TF 与传感器已存在 | 与上一轮同类现象，不足以证明同一底层缺陷 | `p0-03-chnntxl6/logs/navigation-startup.out:1`、`.exit:1`；`result.json:8–11`；`runtime/initialize-summary.json:16–18,37–53` | 不再把共享内存当作已证实根因 |
| lpgh50p8 本次失败 | 唯一定位 STARTUP 调用在 30 秒内没有返回；之后 map_server、amcl 的 GetState 都返回 unconfigured；关闭服务后来成功 | 首次服务请求/响应传递、回调执行或启动竞态；尚未定位到其中哪一层 | 本轮 `logs/localization-startup.out:1–2`、`.exit:1`；`runtime/failure-readonly.json:359–372`；`logs/shutdown-localization.out:4–5` | 针对同一服务关联请求标识、服务端回调入口和响应发送，区分请求未到、未执行、响应未回 |
| 清理失败的区别 | 旧两轮有初始化异常进程需强杀；旧成功轮也有异常退出。本轮正常关闭，无异常退出记录 | 旧轮各异常退出原因不可合并推断 | 旧两轮 `runtime/cleanup-evidence.json`；本轮 `runtime/cleanup-evidence.json:2–7`、`logs/robot-launch.log:56–72` | 本轮已完成清理；不用它冒充旧轮初始化阻塞已修复 |

**两轮是否同因？** `3hg1954j` 与 `chnntxl6` 同属“初始化/服务就绪阶段未完成、NOT_SENT”，但底层同因没有证实。它们与 `nq1hmjaf` 的直接失败阶段不同，不能统称 Goal was rejected。

**明确的直接阻断链：** 本轮定位启动调用未完成 → 两个定位节点后来仍为 unconfigured → 没有地图/AMCL 输出及 map→odom → 门槛未满足 → 不创建导航进程、不发目标。请求是否抵达服务端、回调是否曾进入、响应是否丢失，没有足够证据。

**不能确定的部分：**

- 旧两轮底层卡住位置未知。历史 `route-startup-gdb.err:1` 是 ptrace 失败，`controller-blocked-stack.err:1` 是 sudo 需要密码；没有有效堆栈，不能据此宣称已定位 DDS 死锁。
- 本轮线程等待位置包含 futex，但正常响应 GetState 的进程也会等待 futex；这不是死锁证据。
- 本轮管理器能响应后来的 SHUTDOWN，不支持“整个进程一直卡死”；也不能反推出首次 STARTUP 一定执行过。
- 旧两轮的初值存在 12–18 ms 未来外推警告，但之后已有 AMCL 和 map TF，现有证据不支持把它列为最早阻断。
- Gazebo 的 gz_frame_id 扩展提示没有阻断本轮 scan/odom 数据；不是已证实根因。
- 2026-10-23 核查发现后来新增了其他回合，本报告未采纳其结果，也未操作其进程。

## 2. 实际修改及理由

所有运行修改都限于本轮 `artifacts/` 和 `session.bash`。

1. `tb3_simulation.launch.py` 把包含项从 `bringup_launch.py` 改为 `localization_launch.py`。意图先建立真实 clock、scan、odom、定位 TF，再单独创建官方 navigation_launch 中的导航节点。相较此前，仅等待激活的做法，这次延后的是导航进程创建。由于定位阶段失败，后半段没有执行。
2. 保留已有临时世界文件清理保护：文件存在时才删除。差异保存在 `artifacts/launch.patch`，官方安装文件未修改。
3. 复用已有观察、一次目标客户端、定位检查与清理脚本。新加 `control.py` 保存统一会话指纹、实际 argv、时间与进程身份；`failure_readonly.py` 只读保存服务、生命周期和线程等待信息。
4. 清理副本增加服务关闭超时的异常处理，使其仍能继续清理本轮已核验进程；关闭服务等待改为 15 秒，launch 父进程等待改为 25 秒。保存 `artifacts/cleanup.patch`。本轮关闭服务成功，异常处理分支没有被触发，不能声称该分支经过运行验证。
5. MPPI YAML 与参考轮哈希完全相同。Collision Monitor 没有被禁用；由于导航未创建，本轮没有实际 Collision Monitor 插件或速度链运行证据。
6. 沿用已有 `use_rviz:=False`，未声称 RViz 目视验收。图形会话是从当时桌面进程读取并经 xdpyinfo 验证，不是猜 DISPLAY。

修复值得试一次的依据：参考 `5u34h8ya` 已保存同配置导航 SUCCEEDED 的 UUID 和运动证据，而后两轮在导航进程创建/服务发现阶段失败；其导航进程先创建、clock 桥接稍后出现。分阶段创建是针对这个差异的兼容性假设，不是已证实的底层修复。

## 3. 本轮时间线

以下为本机同一 boot_id 下的 monotonic 秒；命令启动时刻不等于服务端收到请求的时刻。

| 时刻 | 事实 | 证据 |
|---|---|---|
| 22395.889 | 建立新回合、保存源码/旧证据指纹 | `repo-at-start.json:4–6` |
| 22452.185 | 唯一 Gazebo 服务器启动 | `logs/control-events.jsonl:1` |
| 22494.658 | 官方机器人、桥接、定位 launch 启动；未包含导航 | `logs/control-events.jsonl:3` |
| 22494.673 | 启动唯一定位 STARTUP 客户端；约 30 秒后超时 | `logs/control-events.jsonl:4`；`logs/localization-startup.exit:1` |
| 22546.672–22554.211 | 只读采样：2514 clock、210 odom、37 scan；无 map/AMCL；窗口内 clock 无回跳 | `runtime/pre-summary.json:3–18`；`runtime/pre-samples.json` |
| 22595.452 附近 | GetState 返回两个定位节点均 unconfigured | `runtime/failure-readonly.json:2,359–372` |
| 22679.094 | SHUTDOWN 服务成功返回 | `runtime/cleanup-evidence.json:45` 起 |
| 22684.605 | 清理完成并结束本轮 | `runtime/session-ended.json` |

A/B/C 的 Gazebo、机器人启动和检查命令使用相同 `session.bash` 指纹：`dc6a3c19f5a6bf9f053abb4cfe5a6666d27a974ff167e0a9f3bb9c4d1d6fbaf2`；ROS_DOMAIN_ID=71，GZ_PARTITION=cmr_p0-03-lpgh50p8。证据为 `logs/control-events.jsonl:1–7`。本轮没有第二次启动请求或目标重发。

## 4. 验证证据表

| 检查项 | 本轮结果 | 证据 |
|---|---|---|
| 官方机器人、世界、地图来源 | Waffle、tb3_sandbox、仅渲染器兼容副本；机器人生成成功。地图参数已传入，但 map_server 未激活，不声称地图已加载 | `runtime/robot-process.json`；`artifacts/world-variant.json`；`logs/robot-launch.log:25–26` |
| 实际插件 | 未验证；controller_server 未创建。配置期望是 MPPI，不冒充运行插件 | `logs/control-events.jsonl`；`result.json` 的 runtime=null |
| 生命周期 | map_server=unconfigured；amcl=unconfigured；导航节点未创建 | `runtime/failure-readonly.json:359–372` |
| clock / scan / odom | 真实样本存在；clock 从 91.830 到 99.369 s；仅该采样窗口无回跳 | `runtime/pre-summary.json:3–29` |
| TF / 定位 | 有 odom→base_footprint、base_link→base_scan；未观测 map→odom 或 AMCL pose | `runtime/pre-summary.json:15–18,31` 起 |
| 初值 / 冻结目标输入 | 未发布初值；未选择或冻结目标。manifest 在失败后整理，明确记录这一事实 | `runtime/finalization-context.json`；`artifacts/scenario.yaml` 的初值/目标均 null |
| 速度链 | 未进入导航运行阶段，未验证完整速度链 | `result.json` |
| 目标 UUID | 无：没有发送目标 | `logs/control-events.jsonl`；`result.json` |
| Action 状态 | NOT_SENT。不是 REJECTED 或 ABORTED；30 秒超时的是定位启动服务客户端，不是导航 Action | `logs/localization-startup.out:2`；`result.json` |
| 位移 / 到达 | 无目标对应运动窗口，位移与到达指标未知，不填 0 | `result.json` 的 metrics |
| 成功后停止 | 未验证，stop_observed=null | `result.json` |
| 碰撞 | 未做独立评估，collision=null | `result.json` |
| 清理 | SUCCEEDED；生命周期关闭响应成功，子进程正常退出，无异常退出；只处理本轮核验进程 | `runtime/cleanup-evidence.json:2–7`；`logs/cleanup.jsonl`；`logs/robot-launch.log:56–72` |
| 旧文件完整性 | 504 个受保护文件、667 个旧证据文件哈希未变 | `runtime/final-integrity.json` |
| 记录合同 | VALID / exit 0；只证明合同与引用一致 | `logs/run-final-check.json`、`.exit` |

软件包的确切运行时版本快照未在本轮建立；manifest 的 package_versions 留空，整理时读取的版本放在 `runtime/finalization-context.json` 并标明时间范围，没有复制旧轮版本冒充本轮采集。sim_start/end 和 RTF 也未用中途采样冒充全回合边界。

## 5. 假设检验与下一项最小检查

- **被否定的是充分性判断**：“只把导航进程延后创建，就足以让整体流程完成”在本次没有成立。
- **尚未检验的是具体作用**：“定位就绪后创建导航能否消除旧两轮导航初始化阻塞”。本轮未到该阶段，不能说这条假设已被证实或证伪。
- **新增证据**：失败可以出现在尚未创建导航进程的定位 STARTUP 调用；两个定位节点可响应只读 GetState，管理器稍后可响应关闭。这与旧轮“部分节点连服务都无法发现”不能直接等同。
- **下一项最小检查**：后续另行诊断时，先针对 `/lifecycle_manager_localization/manage_nodes` 的首次调用建立请求/响应与服务端回调入口关联记录，并核对客户端的服务发现与等待过程。先区分未到达、未执行、已执行但响应未回，再决定补丁。不先改 MPPI、不扩大超时、不连续重发目标。本任务中没有启动该后续试验。

P0-03 尚欠：可靠的定位/导航启动、全部关键节点 active 与实际插件/速度链核实、实际初值输入与短目标冻结、有效观察覆盖下同一 UUID 的目标终态、真实移动、到达误差和成功后连续停止。清理这一项在本轮通过，但不能与其他轮拼接成一个成功回合。

## 6. 交接文件

本轮目录内的 `diagnosis-report.md` 是此报告；`artifacts/launch.patch` 与 `artifacts/cleanup.patch` 是实际差异。所有结果、原始日志、脚本副本均保存在同一回合。

上次收尾曾因自动审批服务使用额度耗尽而中断；被拦截的是只读核查命令，未执行。用户要求继续后，已通过正常审批完成收尾，不再存在该访问阻塞。

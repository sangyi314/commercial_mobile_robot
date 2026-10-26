# 第 4 节整理与验收交付（最终仅新版核心）

交付日期：2026-10-20（Asia/Shanghai）。实际执行目标为 Ubuntu 24.04.5 ARM64，非 macOS 编译。第 3、4 节软件部分已完成；作者推导、理解与独立修改仍待人工验收。

## 路径与备份

- 实际用户 robot，工作区与 Git 根均为 `/home/robot/commercial_mobile_robot`。
- 初始修改前完整备份：`/home/robot/cmr_backups/phase0-section4-20261019T162250Z-0OtrUaQG`，包含工作树压缩包、未跟踪文件清单、暂存/未暂存二进制补丁、HEAD 文件归档、Git 历史 bundle、状态和远程地址；已验证归档与 bundle。
- 按最新要求移除旧核心前再次备份：`/home/robot/cmr_backups/phase0-new-core-only-20261019T162802Z-C69fNaNf`，包含当时全部非构建/非 bag 工作树、状态与补丁，并有 SHA256。旧源码可从此备份或 Git 恢复。
- 备份均在工作区外，没有放入 src。构建缓存和 bag 路径未打包；审计未发现 `.bag`、`.db3`、`.mcap` 文件，bag 目录留在原处。
- Git HEAD 和远程地址保持不变；未 git init、commit、push、reset、clean 或强制 checkout。

## 审计与最终修改

初始旧内容：CMake 强制依赖 ament，并引用工作树中已经缺失的 command_policy 头文件和测试；旧接口历史存在于 Git。新版已完成内容：`core.hpp` 与指南第 4 节一致，保留其原实现。未完成内容：`core_test.cpp` 是空文件，四份核心文档缺失。重复/空目录：`cmr_bringup/config`、`launch` 仅为空配置目录，无 package.xml，未作为完成的 ROS 包。

最初按原请求恢复旧策略并通过了 11 项旧测试；随后依照“只保留新版”的最新要求，最终移除了以下旧文件，不再运行旧策略：

- `src/cmr_core/include/cmr_core/command_policy.hpp`
- `src/cmr_core/test/command_policy_test.cpp`

最终变更：

- `.gitignore` 保留已有规则，合并 build/install/log/cache、两类 bag 目录和 `.bag/.db3/.mcap` 忽略规则，已实际检查忽略效果。
- `src/cmr_core/CMakeLists.txt` 按差异合并：ament 可选、CTest 五组测试、INTERFACE C++17 传递、独立/ament 安装导出；保留有效 include 与目标导出，移除旧 GTest 测试块。
- `src/cmr_core/package.xml` 只移除已无使用者的 ament_cmake_gtest 测试依赖；原 Sang Yi 维护者、真实邮箱、Apache-2.0、版本与 ament 包声明保留。
- `src/cmr_core/test/core_test.cpp` 补齐五组测试和所有指定边界，用运行时检查与非零退出码报告失败，Release/NDEBUG 不会关闭测试。
- 新增 `docs/charter.md`、`docs/environment.md`、`docs/ownership.md`、`docs/core_test_design.md` 及本文；未知硬件与掌握情况均标待确认。
- 新增本次环境证据与日志，最新目录由 `evidence/env/section4-latest-run.txt` 指向。

保留的已有成果：新版 core.hpp、LICENSE、AI_USAGE.md、THIRD_PARTY.md、docs/adr/0001-scope.md、已有 .vscode/settings.json 未提交修改、原实验与环境记录、目录、Git 历史、远程地址、既有 build/install/log。既有实验记录与第 5 节起内容本次未验证，不计入第 4 节验收。旧构建目录虽可能含旧二进制，未作为最终验收或新安装来源。

## 最终测试结果

最终编号：`section4-new-only-20261019T162823Z-5a0cbb`。UTC 时间用于唯一目录命名。该编号的独立和 colcon 构建目录在删除旧策略后首次创建，没有复用此前结果。

| 验收 | 结果 |
|---|---|
| 独立 CMake/Ninja Release 构建、CTest | 通过，5/5；env -i 清空环境并禁用 ament_cmake 查找 |
| straight | 通过；直行、倒车、零 dt、负 dt、NaN/Inf 状态/输入及溢出 |
| arc | 通过；正反转向四分之一圆、倒车圆弧、纯旋转、整圆、正反向积分 |
| near_zero | 通过；0 与 ±1e-12 直线极限、小角分支两侧几何界 |
| wheels | 通过；左右顺序、纯转/倒车、上限包含边界、非法模型与非有限值 |
| gate | 通过；零序号、重复/乱序、TTL 到期即拒绝、未来时间、零/负/过长 TTL、拒绝不耗序号 |
| cmr_core colcon 构建与测试 | 通过；5/5 |
| colcon test-result --verbose | 通过；5 tests, 0 errors, 0 failures, 0 skipped |
| 纯 CMake 与 ament 安装目标的消费者 | 各通过 1/1；请求 C++11 后实际编译为 C++17；仅使用新版头文件 |
| 新安装不包含 command_policy.hpp | 通过，两条安装路径均核验 |
| git diff --check / 新文件空白检查 | 通过 |
| 最终旧策略回归 | 未运行；按最新要求旧接口和测试已删除，此前 11/11 仅为历史记录 |
| Gazebo、Nav2、真实电机、硬件/GPU/跨机时间 | 未运行，均不在本次范围 |

测试 oracle 来自独立解析端点、几何界与接口合同，具体目的见 `docs/core_test_design.md`。未安装、升级或修改系统依赖；Gazebo 包以 ROS vendor 包存在，原生 gz-* 包名查询无匹配已如实记录。ROS_DOMAIN_ID 从现有进程确认 71，本次保持 71，不使用指南示例 42。

## 当前目录树（省略构建产物，证据文件归并展示）

```text
commercial_mobile_robot/
  .gitignore
  .vscode/settings.json
  AI_USAGE.md
  LICENSE
  THIRD_PARTY.md
  src/
    cmr_core/
      CMakeLists.txt
      package.xml
      include/cmr_core/core.hpp
      test/core_test.cpp
    cmr_bringup/
      config/                 # 空目录，不是已完成 ROS 包
      launch/
  docs/
    adr/0001-scope.md
    charter.md
    environment.md
    ownership.md
    core_test_design.md
    section4_completion_report.md
    derivations/
    failures/
    papers/
  tools/
  tests/
  scripts/
  experiments/
    manifests/                # 已有环境与 launch 参数记录
    results/
    scenarios/
  evidence/
    env/                      # 历史及本次日志；section4-latest-run.txt 指向最终结果
    runs/
    graphs/
    bags/
    metrics/
  data/bags/
```

## 从新终端准确复现

在 Ubuntu 终端执行以下完整命令；若从 Mac 开始，先 `ssh cmr-ubuntu`。第一条创建不加载启动文件的干净 Bash，后续逐条在其中执行，避免旧 overlay。每次创建唯一构建/安装目录，不删除原缓存。

```bash
env -i HOME="$HOME" USER="$(id -un)" PATH=/usr/bin:/bin LANG=C.UTF-8 bash --noprofile --norc
set -eo pipefail
export CMR_WS="$HOME/commercial_mobile_robot"
cd "$CMR_WS"
export CMR_CHECK_ID="section4-check-$(date -u +%Y%m%dT%H%M%SZ)-$$"

cmake -S src/cmr_core -B "build/$CMR_CHECK_ID-standalone" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=TRUE
cmake --build "build/$CMR_CHECK_ID-standalone"
ctest --test-dir "build/$CMR_CHECK_ID-standalone" --output-on-failure

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
colcon --log-base "log/$CMR_CHECK_ID-build" build \
  --base-paths src --packages-select cmr_core \
  --build-base "build/$CMR_CHECK_ID-colcon" \
  --install-base "install/$CMR_CHECK_ID-colcon" --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon --log-base "log/$CMR_CHECK_ID-test" test \
  --packages-select cmr_core --build-base "build/$CMR_CHECK_ID-colcon" \
  --install-base "install/$CMR_CHECK_ID-colcon" --event-handlers console_direct+
colcon test-result --test-result-base "build/$CMR_CHECK_ID-colcon" --verbose
git diff --check
```

只在需要使用本次安装时加载准确的 `install/$CMR_CHECK_ID-colcon/local_setup.bash`。不要 source 根目录旧 install/setup.bash 作为本次安装的证据。

## 仍需作者亲自完成

- 推导圆弧积分、中点角与 sinc 的关系，解释近零角速度的数值稳定性。
- 推导差速左右轮角速度与 v/omega 的关系，解释左右顺序、单位和饱和反例。
- 解释 TTL 半开区间以及拒绝命令不消耗序号。
- 逐行理解代码，关闭 AI 后独立修改条件、补测试、解释失败并排故。
- 明确 CommandGate 没有主动过期停车、会话身份、线程保护或跨主机时间机制；本轮测试不代表实车安全。

上述项均未替作者打勾。硬件型号与实际轮径轮距仍待确认。

## 进入第 5 节的下一步命令（本次未执行）

在上述已加载 Jazzy、域号 71 的 Ubuntu 终端中，先检查本机 launch 参数：

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py --show-args
```


## 2026-10-20 P0-01 复核更正（审查草稿，尚未新验收）

本次实际读取 Ubuntu 项目当前源码：HEAD 为 `062eed30e1d59adf586d0743f1c825110180f0a7`，修改前 core_test.cpp 为 56 行五组简版。其端点、近零、轮速与少量门控检查并未实现本文历史表中列出的全部边界；不据此推测历史变化原因。本文旧结论与旧日志原样保留，不能单凭 5/5 证明当前源码包含所称覆盖。

已核对步骤 2–3 的真实编号 `p0-01-gfGjKp8O`：8 项重点与 69 项已跟踪指纹匹配修改前文件，备份内容一致；既有 Release 基准日志为 5/5，只用于修改前版本。本次未重跑或改写这些基准日志。

本次在原五组内补齐清单缺口并更新接口/测试说明。新草稿未编译、未运行新增测试，独立 Release、ASan/UBSan、Jazzy/ament 和两种安装消费者均待负责人审查后顺序验收；ROS/Nav2/Gazebo/实车均未执行，不写 P0-01 通过结论。实际改动、草稿指纹、差异与新证据链接见 [P0-01 草稿报告](p0-01-gfGjKp8O-draft.md)。本人理解与人工审查未被观察，继续待确认。

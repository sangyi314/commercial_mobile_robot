# P0-05 设计与代码审查

审查在任何真实仿真回合之前完成。父进程唯一spawn者；worker没有subprocess/shell。--check只读且以禁用Popen测试验证。

已修正：服务发现前发令、测试录包缺全局clock、延迟假服务回调销毁、clock重复接收但不推进、接受响应边界竞争、取消额外窗口纳入work预算、Gazebo正常退出监控、逃逸SID后代标记归属、bag时间轴与单调时间分离、summary不把VALID直接当通过。

重要限制：受控启动失败不验证真实导航执行中制动；SIGKILL/断电不能保证finally。profile只允许既定官方静态场景，不接受任意shell/可执行插件。

保护范围保持原样：核心、三份原Nav2 YAML、旧启动器、DDS入口、Schema和validator。新增辅助模块/测试可继续修正，必要重测有新日志，不覆盖旧测试输出。

需要真实运行核实：手动STARTUP的阶段依赖、生产录包实际TF/初值/终态与停止覆盖、两种控制器、FAULT_ACTIVATED后不发初值/目标、全过程正常清理。未实测前均不写通过。

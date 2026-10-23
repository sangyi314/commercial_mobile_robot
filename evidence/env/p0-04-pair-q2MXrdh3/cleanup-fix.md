Gazebo 关闭诊断：线程退出的 TSD 析构地址落在已经卸载的 Boost thread 库中。仅为 Gazebo 子进程设置 LD_PRELOAD，保持该既有库的加载引用；关闭检查实际退出 0。导航配置、物理几何、核心和系统库均未改。旧结果保持 cleanup FAILED。

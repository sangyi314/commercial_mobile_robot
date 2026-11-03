# P1 Reliability v1

2026-11-03。新增可靠性验证层，Mission/Store/Action/Gate单写者和安全阈值保持原合同。真正结果在本轮verification和evidence内；设计不等于通过。

准入：本人最新明确允许AI代理交接作为P1-05仿真准入，见ADR p1-delegated-handover-20261103。G01D必须真实运动和source=synthetic；原human G01不得改记通过。正式长时还需短时资格和R01-R05当前候选兼容。

L0纯核/SQLite、L1真实ROS协议+明确synthetic运动sink、L2真实Gazebo/Nav2+Oracle停车辅助、L3未实施。I1–I10沿用Mission合同，unknown/旧Goal保持槽、期限不跨进程继承，不把取消ACK当Goal终态或停车。

候选：HEAD加工作树完整内容、SHA256源码快照、patch、实际依赖、ELF和配置；修复生成新候选。--check不能建运行目录/DB/节点/进程/迁移。profile5仅从已审核profile4 G05派生，增加reliability_v1和work/finalize预算，不改运动链。原Runner仍唯一拥有所有进程，ReliabilitySession仅业务编排，无ActionClient或另一个supervisor。

默认连续负载：每60秒记录机会；20单固定分布前中后，首两单间隔1s验证FIFO；其余每180s，预留Gate64条Begin历史中的60个attempt额度（每单最多2正常+1额外）。不清历史、不重启Gate扩容。原60单计划可能超过门控容量，因此采用明确的本机有限负载，不声称60单吞吐。短时300秒3单；600秒4单；正式3600秒20单，最低10单同时要求全部已接纳单正常终态。source=synthetic。未选机会记录为冻结计划空机会，不追赶补单。

计时：系统READY/初始停止/录制就绪后独立steady t0；严格连续3600s，不计启动。最后接单3420s，drain180s；任务240s/Goal150s/共享retry1保持；长时work=duration+380包含startup180、drain180和监督开销20，finalize180。每秒资源/进程/状态采样，gap<=3s、RSS<=6GiB、累计CPU<=700%、空闲磁盘>=8GiB；文件录制进展gap<=3s。资源阈值先在短会话验证，正式后不得修改。仅声明软实时观测。

健康：真实owner唯一Goal；Gate/observer/source receipt健康和最终速度源唯一；异常立即停止新调度并委托原安全收尾；不能清未知状态继续跑。租约300ms、候选/停车freshness及3s停车窗不变。Gate/Bridge死亡缺下游保护不在运动中注入。Oracle辅助仅仿真停车，C ACK UNKNOWN、collision=null。

完整原始记录落盘；仅长时在线观察缓存有限，离线审计按SQLite确切消费窗口的预定超集流式选取，仍扫描全部MCAP计数/UUID/时钟/地图及非法Gate输出。不能按是否通过选窗口。全原始文件hash逐条核对，缺证据拒绝。

R01–R16冻结表见experiments/mission/reliability-v1.yaml。每个率明确分母；接纳=完成+失败+取消+非终态活动+WAITING，PAUSED/UNCERTAIN只算活动一次。预期业务失败可使故障用例通过，不能计配送完成。

正式一小时须实际监督安排与明确中止方法，不能伪造supervision记录。S10本人理解/练习/最终放行独立待签；只可准备材料。无自动commit/push/tag/release或P2。

## ExecutionPort续租回执容量修复

既有effect ID为mission:已提交Record版本:kind。高频RenewLease改用当前attempt上下文内uint64最后版本，严格单调并核对mission/operation及规范ID；不建立新上下文，撤销后旧上下文不恢复。旧/重复版本不再产生续租请求。Send/Begin/Revoke/Reset等一次性effect仍留永久去重set及4096上限，contexts64和Gate Begin64上限不变。不扩大队列、租约、期限或安全阈值。5000条续租API回归与格式反例单列，随后对实际协议及L2重测。

## Delivery API协议容量

D06十一请求同时在途发现默认服务QoS KEEP_LAST depth10可能在executor读取前丢首个u0。Delivery服务显式Services QoS可靠/volatile，history depth64与API pending64匹配；并发测试客户端同为64。保持owner邮箱16、等待容量4、并发11和8秒期限；过载返回OVERLOADED，DDS送达不等于事务提交。原失败保留并重测。

P1-05 r07录制修复：本机rosbag2/ros2bag/mcap_vendor0.26.11，官方ros2/rosbag2 tag0.26.11 commit cb8fa198b93bfed90e2417b52b8645fb9ef89c2b（Apache2），研究SequentialWriter、mcap_storage.cpp的YAML转换及实际test_mcap_storage.cpp。默认ROS双缓存100MiB和MCAP chunk使文件mtime不代表订阅健康；仅profile5显式max-cache-size=0、MCAP noChunking=true、enableDataCRC=true，不降低三秒文件进展门槛，P0及profile4录制不变。数据CRC保留，文件可见增长不等于掉电fsync保证。短测r06失败与原MCAP保留；新候选重跑。窗口失败也单独记录已开始/中止和真实steady时长，不拼接。成熟组件直接使用，CMR参数和监控独立适配，无上游实现复制。源码/测试/许可在evidence/env/p1-05-dev-b1sg_5gd/upstream-recording-r07。

录制进展仅用监督器本地steady时间观测MCAP总字节增长，拒绝截断、超过3秒无增长；t0前必须在初始停止证明期间看到实际字节增加。文件mtime不作单调期限或fsync证明。失败窗口保存真实已运行时长并标aborted；不因日志不足释放未知Goal。

独立长时审计额外要求逐样本ROS时钟推进；按原实际消费窗口，使用包围其边界的独立Gazebo pose全窗复核，不丢弃短运动前缀或挑后续静止后缀。Recover重新发Goal也绑定当次实际消费的停止窗。此为审计增强，不改Motion/lease/到达/交接或短会话实际负载；重新审计原闭合数据，原证据和失败不改写。

验收审计同时核对启动manifest.inputs冻结引用与最终result.artifacts引用；materialized mission-events.jsonl必须逐条等于SQLite重建的事件/队列时间线。不能仅重签最终hash就把实验输入或导出日志改写成新事实。负例只在新独立副本内注入，保留原始回合所有字节，副本明确标测试，不用于通过真实验收。

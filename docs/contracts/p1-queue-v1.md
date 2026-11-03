# P1 Queue v1 — 持久有限FIFO

2026-11-02；P1-04增量合同。Store/Runtime单写者不变。默认等待容量4，不含活动Mission；容量0兼容BUSY。WAITING仅登记，不是新FSM状态，也不持有Goal或租约。

## 事务与身份

独立registration表持久保存规范MissionRequest及冻结站点/地图/BT快照；queue_seq为SQLite单调唯一号；queue_state=WAITING/ACTIVE/CANCELED/FINISHED。同ID检查在容量检查之前，同内容EXISTING、异内容CONFLICT；created_at仅审计。满队列QUEUE_FULL不得丢既有幂等决定。取消后登记和审计不删除，ID不能重新执行。

BEGIN IMMEDIATE内查ID/槽/队首/容量→登记或激活→写审计→COMMIT后回复。仅无槽、无等待且调度健康时新单直接激活。已有等待时新单只能尾部入队，不能趁空槽插队。升队单事务核对最早WAITING与版本、原冻结运行资格、占槽、建立Record、写事件。提交后才允许Prepare/授权/Send。

队中取消先提交则不提升；提升先提交则按活动取消，不直接释放未知运动。取消等待项无当前其他任务Goal/租约效果。COMPLETED完整收尾且健康才自动升下一项；FAILED/CANCELED/存储/授权/定位异常默认锁存调度暂停。暂停/恢复调度与旧运动解析是独立条件，恢复不能代替Goal终态/D。

重启恢复顺序、锁存调度暂停、活动PAUSED/UNCERTAIN；不自动升队或重放Goal。任务级总时限从激活开始，返程/重试不重置，等待无自动过期。

## 版本与兼容

配送配置只使用新schema2开发DB；旧P1-01/02/03 fixture默认仍schema1和容量0。既有v1数据库未经明确迁移授权不得自动升级；历史evidence永不迁移。复制样本迁移必须排他owner锁、SQLite backup一致副本、事务升级、旧ID/终态/未决槽保留；未知版本拒绝、不删除重建。没有指定当前开发DB时交付使用新DB。

必须测试容量0/4、满队列重复/冲突、活动+4等待、第5拒绝、空槽有队列不插队、并发唯一排序、取消/升队屏障竞态、提交前后崩溃、重启暂停、错资源、DB提交失败。独立核对max活动Mission/未决Goal≤1。实现/运行状态单列，不把本合同当通过证据。

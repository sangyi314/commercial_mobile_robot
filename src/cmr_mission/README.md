# cmr_mission

C++17 persistent Mission core and ROS 2 Jazzy Delivery + Gate integration.

- CMR_MISSION_BUILD_ROS=OFF builds the FSM, SQLite Store, owner Runtime and offline tests.
- CMR_MISSION_BUILD_ROS=ON builds the ament package and Mission Node. BUILD_TESTING also builds the isolated fake Nav2 server.
- Submit, cancel, handover and queue operations use the namespaced DeliveryCommand service. MissionCommand remains available for queries. The owner commits business decisions before dispatching effects.
- The node always creates DeliveryMonitor, NavigationAdapter and ExecutionPort. Stop and arrival evidence goes through the observer and monitor. The isolated test setup supplies synthetic ROS inputs to those components.

| Parameter | Purpose |
| --- | --- |
| test_mode | Enables the isolated domain 72 setup with synthetic inputs and a private Action name. The default uses domain 71 with /navigate_to_pose. |
| station_catalog | Path to the station catalog and its referenced map and behavior tree resources. |
| queue_capacity | Number of waiting orders. Zero enables active-slot admission only. |
| task_ms, handover_ms | Total task and handover deadlines in milliseconds. |
| action_name | NavigateToPose Action endpoint. |
| database, process_epoch, mailbox_capacity | SQLite path, current owner identity, and bounded owner queue capacity. |
| execution_run_id, execution_namespace | Execution run identity and Gate namespace. The namespace must match the node namespace and defaults to it. |
| readiness_ms, accept_ms, execute_ms, cancel_ms, collect_ms | Action readiness, acceptance, execution, cancel response and collection deadlines in milliseconds. |
| use_sim_time | Standard ROS parameter. Mission requires false and uses the explicit source clock for evidence. |

Parameters are frozen after startup. Startup overrides use ROS 2 parameter declaration rules.

See [Delivery integration](../../docs/guides/delivery-mission.md) and [Action protocol tests](../../docs/guides/mission-action-tests.md).

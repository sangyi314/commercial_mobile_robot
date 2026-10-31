# cmr_mission

C++17 persistent Mission core and optional thin Jazzy NavigateToPose adapter.

- CMR_MISSION_BUILD_ROS=OFF: FSM, SQLite, owner Runtime and offline tests; no ROS lookup/linking.
- CMR_MISSION_BUILD_ROS=ON: ament package, mission_node, and BUILD_TESTING-only fake_nav2_server.
- Business services return only after owner transactions. Synthetic evidence exists only in explicit isolated test mode; production is refused until a real ExecutionPort exists.

See ../../docs/contracts/p1-action-v1.md, ../../docs/design/p1-action-design.md and ../../docs/guides/mission-action-tests.md.
No Gazebo/Nav2 stack, actual motion gate or physical stop guarantee is supplied by this package.

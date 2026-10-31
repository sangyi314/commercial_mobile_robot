# cmr_mission offline core

C++17 FSM, system SQLite C API store, bounded single-owner Runtime and synthetic Port tests.
No ROS package/runtime, no Gazebo, no physical stop claim. S1 admission implementation unchanged.

See ../../docs/guides/mission-offline.md and ../../docs/design/p1-mission-tests.md.
Build and test in a new directory. SQLite3 development headers and POSIX flock are required.
Runtime drains accepted mailbox work on close; a blocked user Port cannot be forcefully preempted.
The offline probe uses Runtime/Store, records synthetic=true and requires new DBs for scenarios.

source /opt/ros/jazzy/setup.bash
export CMR_WS=/home/robot/commercial_mobile_robot
export CMR_RUN=/home/robot/commercial_mobile_robot/evidence/runs/p0-03-0pl9lbsf
export CMR_RUN_ID=p0-03-0pl9lbsf
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=cmr_p0-03-0pl9lbsf
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
export CMR_SIM_SHARE="$(ros2 pkg prefix --share nav2_minimal_tb3_sim)"
export CMR_NAV_SHARE="$(ros2 pkg prefix --share nav2_bringup)"
export GZ_SIM_RESOURCE_PATH="$CMR_SIM_SHARE/models:$(dirname "$CMR_SIM_SHARE")"
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
cd "$CMR_WS"

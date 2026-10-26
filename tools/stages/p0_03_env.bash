# Source AFTER the real simulation session.bash in every P0-03 terminal.
# Changes only this shell and its children; does not edit global ROS settings.
export ROS_DOMAIN_ID=71
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)/src/cmr_bringup/config/fastdds_loopback_udp.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"

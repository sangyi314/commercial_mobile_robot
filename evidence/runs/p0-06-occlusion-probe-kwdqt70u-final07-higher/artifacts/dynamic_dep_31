from copy import deepcopy
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[1]
folder = root / "src/cmr_bringup/config"
base = yaml.safe_load((folder / "nav2_installed.yaml").read_text())
assert "controller_server" in base, "Unexpected installed YAML structure"

mppi = {
    "plugin": "nav2_mppi_controller::MPPIController",
    "time_steps": 50, "model_dt": 0.05, "batch_size": 1000,
    "vx_std": 0.15, "vy_std": 0.15, "wz_std": 0.35,
    "vx_max": 0.3, "vx_min": -0.1, "vy_max": 0.0, "wz_max": 1.0,
    "ax_max": 1.0, "ax_min": -1.0, "ay_max": 0.0, "az_max": 2.0,
    "iteration_count": 1, "temperature": 0.3, "gamma": 0.015,
    "motion_model": "DiffDrive", "visualize": False,
    "prune_distance": 1.7, "transform_tolerance": 0.1,
    "critics": ["ConstraintCritic", "CostCritic", "GoalCritic",
                "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
                "PathAngleCritic", "PreferForwardCritic"],
}
# Critic default parameters come from the installed plugin version.
# Save that package version; these values are only a smoke-test starting point.
dwb = {
    "plugin": "dwb_core::DWBLocalPlanner",
    "debug_trajectory_details": False,
    "min_vel_x": -0.1, "min_vel_y": 0.0,
    "max_vel_x": 0.3, "max_vel_y": 0.0, "max_vel_theta": 1.0,
    "min_speed_xy": 0.0, "max_speed_xy": 0.3, "min_speed_theta": 0.0,
    "acc_lim_x": 1.0, "acc_lim_y": 0.0, "acc_lim_theta": 2.0,
    "decel_lim_x": -1.0, "decel_lim_y": 0.0, "decel_lim_theta": -2.0,
    "vx_samples": 20, "vy_samples": 1, "vtheta_samples": 30,
    "sim_time": 2.5, "linear_granularity": 0.05,
    "angular_granularity": 0.025, "transform_tolerance": 0.1,
    "xy_goal_tolerance": 0.25, "trans_stopped_velocity": 0.02,
    "short_circuit_trajectory_evaluation": True, "stateful": True,
    "critics": ["RotateToGoal", "Oscillation", "BaseObstacle",
                "GoalAlign", "PathAlign", "PathDist", "GoalDist"],
    "BaseObstacle.scale": 0.02, "PathAlign.scale": 24.0,
    "GoalAlign.scale": 16.0, "PathDist.scale": 24.0,
    "GoalDist.scale": 16.0, "RotateToGoal.scale": 24.0,
    "PathAlign.forward_point_distance": 0.1,
    "GoalAlign.forward_point_distance": 0.1,
    "RotateToGoal.slowing_factor": 5.0,
    "RotateToGoal.lookahead_time": -1.0,
}
for method, plugin in [("mppi", mppi), ("dwb", dwb)]:
    config = deepcopy(base)
    ctl = config["controller_server"]["ros__parameters"]
    ctl["controller_frequency"] = 20.0
    ctl["controller_plugins"] = ["FollowPath"]
    ctl["FollowPath"] = plugin
    smooth = config.get("velocity_smoother", {}).get("ros__parameters")
    if smooth is not None:
        smooth.update(max_velocity=[0.3, 0.0, 1.0],
                      min_velocity=[-0.1, 0.0, -1.0],
                      max_accel=[1.0, 0.0, 2.0],
                      max_decel=[-1.0, 0.0, -2.0])
    (folder / f"nav2_{method}.yaml").write_text(
        yaml.safe_dump(config, sort_keys=False))
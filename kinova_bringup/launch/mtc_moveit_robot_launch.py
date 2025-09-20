#!/usr/bin/env python3
"""
Merged launch file: combines the robustness of `moveit_robot_launch.py` with the
MTC capability setup from `mtc_demo.launch.py`.

Features:
- Supports selectable Kinova robot type (default j2s6s300).
- Loads URDF via xacro and full MoveIt configuration (SRDF, kinematics, limits, OMPL).
- Normalizes ROS1-style controllers.yaml into MoveIt2 Simple Controller Manager format.
- Adds conservative default per-joint constraints if absent.
- Optionally enables ExecuteTaskSolutionCapability for MoveIt Task Constructor (on by default).
- Optional RViz (standard visualization or MTC layout if available).
- Optional fallback joint_state_publisher when hardware driver is not running.
- TRAC-IK fallback to KDL if the plugin is not installed.

This launch DOES NOT start the MTC task node itself. You can launch your MTC
pick/place task separately (e.g. using your modified `pick_place_demo.launch.py`
or a dedicated MTC node) and it will be able to execute via move_group because
of the added capability.

Example usage:
  ros2 launch kinova_bringup mtc_moveit_robot_launch.py
  ros2 launch kinova_bringup mtc_moveit_robot_launch.py kinova_robotType:=j2s6s300 with_rviz:=true use_mtc_capabilities:=true
"""

import os
import pathlib
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
    PackageNotFoundError,
)
import xacro

PACKAGE_NAME = "kinova_bringup"


def generate_launch_description():
    # Launch arguments
    declare_robot_type = DeclareLaunchArgument(
        "kinova_robotType",
        default_value="j2s6s300",
        description="Kinova robot type, e.g. j2s6s300 or j2n6s300",
    )
    # Match execution/tolerance params to moveit_robot_launch.py
    declare_goal_joint_tol = DeclareLaunchArgument(
        "goal_joint_tolerance",
        default_value="0.001",
        description="Joint-space goal tolerance (rad).",
    )
    declare_goal_pos_tol = DeclareLaunchArgument(
        "goal_position_tolerance",
        default_value="0.001",
        description="Cartesian position goal tolerance (m).",
    )
    declare_goal_ori_tol = DeclareLaunchArgument(
        "goal_orientation_tolerance",
        default_value="0.001",
        description="Cartesian orientation goal tolerance (rad).",
    )
    declare_allowed_start_tol = DeclareLaunchArgument(
        "allowed_start_tolerance",
        default_value="0.001",
        description="Allowed deviation between current state and trajectory start (rad).",
    )
    declare_allowed_goal_duration_margin = DeclareLaunchArgument(
        "allowed_goal_duration_margin",
        default_value="2.0",
        description="Extra time (s) past trajectory end before failure.",
    )
    declare_allowed_execution_duration_scaling = DeclareLaunchArgument(
        "allowed_execution_duration_scaling",
        default_value="10.0",
        description="Scaling of planned duration allowed for execution before timeout.",
    )
    # TOTG / resampling parameters (used by AddTimeOptimalParameterization)
    declare_path_tolerance = DeclareLaunchArgument(
        "path_tolerance",
        default_value="0.001",
        description="move_group.path_tolerance (rad).",
    )
    declare_resample_dt = DeclareLaunchArgument(
        "resample_dt",
        default_value="0.05",
        description="move_group.resample_dt (s).",
    )
    declare_min_angle_change = DeclareLaunchArgument(
        "min_angle_change",
        default_value="0.0005",
        description="move_group.min_angle_change (rad).",
    )
    declare_with_rviz = DeclareLaunchArgument(
        "with_rviz", default_value="true", description="Start RViz2"
    )
    declare_use_mtc_cap = DeclareLaunchArgument(
        "use_mtc_capabilities",
        default_value="true",
        description="Add ExecuteTaskSolutionCapability so external MTC tasks can execute solutions",
    )
    declare_publish_fallback_jsp = DeclareLaunchArgument(
        "publish_fallback_joint_state",
        default_value="false",
        description="Start joint_state_publisher to provide fake joint states if no driver is running",
    )

    def yaml_to_dict(path_to_yaml):
        if not os.path.exists(path_to_yaml):
            return {}
        try:
            return yaml.safe_load(pathlib.Path(path_to_yaml).read_text()) or {}
        except Exception:
            return {}

    def strip_none(o):
        if isinstance(o, dict):
            return {k: strip_none(v) for k, v in o.items() if v is not None}
        if isinstance(o, list):
            return [strip_none(v) for v in o if v is not None]
        return o

    def setup(context, *args, **kwargs):
        nodes = []
        lc = lambda name: LaunchConfiguration(name).perform(context)

        robot_type = lc("kinova_robotType")
        with_rviz = lc("with_rviz").lower() in ("1", "true", "yes", "on")
        use_mtc_cap = lc("use_mtc_capabilities").lower() in ("1", "true", "yes", "on")
        use_fallback_jsp = (
            lc("publish_fallback_joint_state").lower() in ("1", "true", "yes", "on")
        )

        package_dir = get_package_share_directory(PACKAGE_NAME)

        # Validate MoveIt installation
        installed_pkgs = get_packages_with_prefixes().keys()
        if "moveit_ros_move_group" not in installed_pkgs:
            nodes.append(
                LogInfo(
                    msg="'moveit_ros_move_group' package not found. Install MoveIt2 to use this launch."
                )
            )
            return nodes

        # URDF (xacro)
        xacro_file = os.path.join(
            get_package_share_directory("kinova_description"),
            "urdf",
            f"{robot_type}_standalone.xacro",
        )
        if not os.path.exists(xacro_file):
            nodes.append(LogInfo(msg=f"URDF not found: {xacro_file}"))
            return nodes
        doc = xacro.process_file(xacro_file)
        description = {"robot_description": doc.toprettyxml(indent="  ")}

        # MoveIt config
        moveit_config_dir = os.path.join(
            package_dir, "moveit_resource", f"{robot_type}_moveit_config", "config"
        )
        if not os.path.isdir(moveit_config_dir):
            nodes.append(
                LogInfo(
                    msg=f"MoveIt config directory not found for {robot_type}: {moveit_config_dir}"
                )
            )
            return nodes

        srdf_path = os.path.join(moveit_config_dir, f"{robot_type}.srdf")
        if not os.path.exists(srdf_path):
            nodes.append(LogInfo(msg=f"SRDF not found: {srdf_path}"))
            return nodes
        description_semantic = {
            "robot_description_semantic": pathlib.Path(srdf_path).read_text()
        }

        kin_path = os.path.join(moveit_config_dir, "kinematics.yaml")
        kin_cfg = yaml_to_dict(kin_path)
        if kin_cfg is None:
            kin_cfg = {}

        # Fallback to KDL if TRAC-IK requested but not available
        uses_trac_ik = False
        if isinstance(kin_cfg, dict):
            for grp in kin_cfg.values():
                if (
                    isinstance(grp, dict)
                    and "kinematics_solver" in grp
                    and "trac_ik" in str(grp["kinematics_solver"]).lower()
                ):
                    uses_trac_ik = True
                    break
        if uses_trac_ik:
            try:
                get_package_share_directory("trac_ik_kinematics_plugin")
            except PackageNotFoundError:
                # Fallback generic kinematics config with KDL solver
                fb_kin_path = os.path.join(package_dir, "moveit_resource", "kinematics.yaml")
                fb_cfg = yaml_to_dict(fb_kin_path)
                if isinstance(fb_cfg, dict):
                    merged = dict(kin_cfg)
                    for group_name, cfg in fb_cfg.items():
                        base = dict(merged.get(group_name, {}))
                        base.update(cfg or {})
                        merged[group_name] = base
                    kin_cfg = merged
        description_kinematics = {"robot_description_kinematics": strip_none(kin_cfg)}

        jl_path = os.path.join(moveit_config_dir, "joint_limits.yaml")
        description_joint_limits = {
            "robot_description_planning": strip_none(yaml_to_dict(jl_path) or {})
        }

        # OMPL planning config
        ompl_planning_pipeline_config = {
            "move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": " ".join(
                    [
                        "default_planner_request_adapters/AddTimeOptimalParameterization",
                        "default_planner_request_adapters/FixWorkspaceBounds",
                        "default_planner_request_adapters/FixStartStateBounds",
                        "default_planner_request_adapters/FixStartStateCollision",
                        "default_planner_request_adapters/FixStartStatePathConstraints",
                    ]
                ),
                "start_state_max_bounds_error": 0.001,
            }
        }
        ompl_yaml_path = os.path.join(moveit_config_dir, "ompl_planning.yaml")
        if os.path.exists(ompl_yaml_path):
            ompl_yaml = yaml_to_dict(ompl_yaml_path)
            if isinstance(ompl_yaml, dict):
                ompl_planning_pipeline_config["move_group"].update(strip_none(ompl_yaml))

        # Controllers
        controllers_path = os.path.join(moveit_config_dir, "controllers.yaml")
        controllers_yaml = yaml_to_dict(controllers_path) or {}
        controllers_yaml = strip_none(controllers_yaml)

        # Normalize ROS1-style schema
        if "controller_list" in controllers_yaml and isinstance(
            controllers_yaml["controller_list"], list
        ):
            controller_names = []
            converted = {}
            for ctrl in controllers_yaml["controller_list"]:
                name = ctrl.get("name")
                if not name:
                    continue
                controller_names.append(name)
                converted[name] = {
                    k: v
                    for k, v in ctrl.items()
                    if k in ("action_ns", "type", "default", "joints", "constraints")
                }
            controllers_yaml = {"controller_names": controller_names, **converted}

        # Inject default constraints if missing (match moveit_robot_launch.py)
        for cname in list(controllers_yaml.get("controller_names", [])):
            cfg = controllers_yaml.get(cname, {}) or {}
            if "constraints" not in cfg:
                joints = cfg.get("joints", []) or []
                per_joint = {jn: {"goal": 0.0005, "trajectory": 0.0005} for jn in joints}
                cfg["constraints"] = {
                    "goal_time": 2.0,
                    "stopped_velocity_tolerance": 0.1,
                    **per_joint,
                }
                controllers_yaml[cname] = cfg

        moveit_controllers = strip_none(
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager": controllers_yaml,
            }
        )

        # Execution tuning (match moveit_robot_launch.py, from launch args)
        execution_tuning = {
            "goal_joint_tolerance": float(lc("goal_joint_tolerance")),
            "goal_position_tolerance": float(lc("goal_position_tolerance")),
            "goal_orientation_tolerance": float(lc("goal_orientation_tolerance")),
            "trajectory_execution": {
                "allowed_start_tolerance": float(lc("allowed_start_tolerance")),
                "allowed_goal_duration_margin": float(lc("allowed_goal_duration_margin")),
                "allowed_execution_duration_scaling": float(lc("allowed_execution_duration_scaling")),
                "execution_duration_monitoring": True,
            },
        }

        # Parameters used by AddTimeOptimalParameterization (match moveit_robot_launch.py)
        topp_params = {
            "move_group.path_tolerance": float(lc("path_tolerance")),
            "move_group.resample_dt": float(lc("resample_dt")),
            "move_group.min_angle_change": float(lc("min_angle_change")),
        }

        # MTC capability (optional)
        extra_caps = {}
        if use_mtc_cap:
            extra_caps = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

        sim_time = {"use_sim_time": False}

        # Robot state publisher (TF)
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[description],
            )
        )

        # Optional fallback joint_state_publisher
        if use_fallback_jsp:
            nodes.append(
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                    output="log",
                )
            )

        # move_group
        nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    description_joint_limits,
                    ompl_planning_pipeline_config,
                    moveit_controllers,
                    execution_tuning,
                    topp_params,
                    extra_caps,
                    sim_time,
                ],
                remappings=[
                    ("joint_states", f"{robot_type}_driver/out/joint_state"),
                    ("/joint_states", f"/{robot_type}_driver/out/joint_state"),
                ],
            )
        )

        # RViz (optional) – prefer MTC layout if available, else package default
        if with_rviz:
            rviz_config_file = None
            try:
                rviz_config_file = os.path.join(
                    get_package_share_directory("moveit2_tutorials"),
                    "launch",
                    "mtc.rviz",
                )
                if not os.path.exists(rviz_config_file):
                    rviz_config_file = None
            except Exception:
                rviz_config_file = None
            if rviz_config_file is None:
                rviz_config_file = os.path.join(
                    package_dir, "moveit_resource", "visualization.rviz"
                )
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="log",
                    arguments=["-d", rviz_config_file],
                    parameters=[
                        description,
                        description_semantic,
                        description_kinematics,
                        description_joint_limits,
                        sim_time,
                    ],
                    remappings=[
                        ("joint_states", f"{robot_type}_driver/out/joint_state"),
                        ("/joint_states", f"/{robot_type}_driver/out/joint_state"),
                    ],
                )
            )

        return nodes

    return LaunchDescription(
        [
            declare_robot_type,
            declare_goal_joint_tol,
            declare_goal_pos_tol,
            declare_goal_ori_tol,
            declare_allowed_start_tol,
            declare_allowed_goal_duration_margin,
            declare_allowed_execution_duration_scaling,
            declare_path_tolerance,
            declare_resample_dt,
            declare_min_angle_change,
            declare_with_rviz,
            declare_use_mtc_cap,
            declare_publish_fallback_jsp,
            OpaqueFunction(function=setup),
        ]
    )

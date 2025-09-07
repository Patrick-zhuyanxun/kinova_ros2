#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Kinova robot in MoveIt2 with selectable robot type."""

import os
import pathlib
import yaml
from launch.actions import LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import xacro


PACKAGE_NAME = 'kinova_bringup'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)

    # Declare the robot type so we can switch between JACO variants
    declare_robot_type = DeclareLaunchArgument(
        'kinova_robotType', default_value='j2s6s300', description='Kinova robot type, e.g. j2s6s300 or j2n6s300'
    )

    def yaml_to_dict(path_to_yaml):
        return yaml.safe_load(pathlib.Path(path_to_yaml).read_text())

    def strip_none(o):
        if isinstance(o, dict):
            return {k: strip_none(v) for k, v in o.items() if v is not None}
        if isinstance(o, list):
            return [strip_none(v) for v in o if v is not None]
        return o

    def setup_nodes(context, *args, **kwargs):
        nodes = []

        # Check if moveit is installed
        if 'moveit' not in get_packages_with_prefixes():
            nodes.append(LogInfo(msg='"moveit" package is not installed, please install it to run this launch.'))
            return nodes

        robot_type = LaunchConfiguration('kinova_robotType').perform(context)

        # URDF (xacro)
        xacro_file = os.path.join(
            get_package_share_directory('kinova_description'), 'urdf', f'{robot_type}_standalone.xacro'
        )
        if not os.path.exists(xacro_file):
            nodes.append(LogInfo(msg=f'URDF not found for robot_type={robot_type}: {xacro_file}'))
            return nodes
        doc = xacro.process_file(xacro_file)
        description = {'robot_description': doc.toprettyxml(indent='  ')}

        # MoveIt config paths based on robot_type
        moveit_config_dir = os.path.join(
            package_dir, 'moveit_resource', f'{robot_type}_moveit_config', 'config'
        )
        if not os.path.isdir(moveit_config_dir):
            nodes.append(LogInfo(msg=f'MoveIt config directory not found for {robot_type}: {moveit_config_dir}'))
            return nodes

        srdf_name = f'{robot_type}.srdf'
        srdf_path = os.path.join(moveit_config_dir, srdf_name)
        if not os.path.exists(srdf_path):
            nodes.append(LogInfo(msg=f'SRDF not found for {robot_type}: {srdf_path}'))
            return nodes

        description_semantic = {'robot_description_semantic': pathlib.Path(srdf_path).read_text()}
        # Kinematics: prefer robot-specific config, but fall back to KDL if TRAC-IK plugin is unavailable
        kin_file = os.path.join(moveit_config_dir, 'kinematics.yaml')
        kin_cfg = yaml_to_dict(kin_file) if os.path.exists(kin_file) else {}
        if kin_cfg is None:
            kin_cfg = {}

        # If robot-specific config requests TRAC-IK but the plugin isn't installed in this environment (common on Humble),
        # override with the package's default KDL config to ensure RViz interactive marker and IK work.
        try:
            available_pkgs = set(get_packages_with_prefixes().keys())
        except Exception:
            available_pkgs = set()
        uses_trac_ik = False
        if isinstance(kin_cfg, dict):
            for group_cfg in kin_cfg.values():
                if isinstance(group_cfg, dict) and 'kinematics_solver' in group_cfg:
                    if 'trac_ik' in str(group_cfg['kinematics_solver']).lower():
                        uses_trac_ik = True
                        break

        if uses_trac_ik and 'trac_ik_kinematics_plugin' not in available_pkgs:
            # Load fallback KDL settings from the generic config bundled with this package
            fallback_kin_path = os.path.join(package_dir, 'moveit_resource', 'kinematics.yaml')
            if os.path.exists(fallback_kin_path):
                fallback_kin = yaml_to_dict(fallback_kin_path) or {}
                # Merge/override per-group settings with fallback (ensures KDL solver is used)
                merged = dict(kin_cfg) if isinstance(kin_cfg, dict) else {}
                for group_name, fb_cfg in (fallback_kin.items() if isinstance(fallback_kin, dict) else []):
                    base = dict(merged.get(group_name, {}))
                    base.update(fb_cfg or {})
                    merged[group_name] = base
                kin_cfg = merged
        description_kinematics = {'robot_description_kinematics': strip_none(kin_cfg)}
        jl = yaml_to_dict(os.path.join(moveit_config_dir, 'joint_limits.yaml'))
        if jl is None:
            jl = {}
        description_joint_limits = {'robot_description_planning': strip_none(jl)}
        sim_time = {'use_sim_time': False}

        # Rviz node
        rviz_config_file = os.path.join(package_dir, 'moveit_resource', 'visualization.rviz')
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    description_joint_limits,
                    sim_time,
                ],
                remappings=[
                    ('joint_states', f'{robot_type}_driver/out/joint_state'),
                    ('/joint_states', f'/{robot_type}_driver/out/joint_state'),
                ],
            )
        )

        # Planning Configuration
        ompl_planning_pipeline_config = {
            'move_group': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1,
            }
        }
        ompl_planning_yaml_path = os.path.join(moveit_config_dir, 'ompl_planning.yaml')
        if os.path.exists(ompl_planning_yaml_path):
            ompl_planning_yaml = yaml_to_dict(ompl_planning_yaml_path) or {}
            ompl_planning_pipeline_config['move_group'].update(strip_none(ompl_planning_yaml))

        # Execution and goal tolerance tuning: ensure we reach the goal in one go
        execution_tuning = {
            # Tighten goal tolerances for joint and pose goals
            'goal_joint_tolerance': 0.00005,
            'goal_position_tolerance': 0.00005,
            'goal_orientation_tolerance': 0.003,
            # Make execution robust to small timing issues and allow enough time to settle
            'trajectory_execution': {
                'allowed_start_tolerance': 0.005,
                'allowed_goal_duration_margin': 2.0,
                'allowed_execution_duration_scaling': 1.5,
                'execution_duration_monitoring': True,
            },
        }

        # Load controllers and normalize schema
        controllers_path = os.path.join(moveit_config_dir, 'controllers.yaml')
        controllers_yaml = yaml_to_dict(controllers_path)
        if controllers_yaml is None:
            controllers_yaml = {}
        controllers_yaml = strip_none(controllers_yaml)

        # Convert ROS1-style 'controller_list' to MoveIt2 Simple Controller Manager format if needed
        if 'controller_list' in controllers_yaml and isinstance(controllers_yaml['controller_list'], list):
            controller_names = []
            converted = {}
            for ctrl in controllers_yaml['controller_list']:
                name = ctrl.get('name')
                if not name:
                    continue
                controller_names.append(name)
                # Copy supported keys
                converted[name] = {
                    k: v for k, v in ctrl.items()
                    if k in ('action_ns', 'type', 'default', 'joints', 'constraints')
                }
            controllers_yaml = {
                'controller_names': controller_names,
                **converted,
            }

        # Inject conservative default constraints if missing to improve goal attainment
        for cname in list(controllers_yaml.get('controller_names', [])):
            cfg = controllers_yaml.get(cname, {}) or {}
            if 'constraints' not in cfg:
                joints = cfg.get('joints', []) or []
                per_joint = {jn: {'goal': 0.001, 'trajectory': 0.005} for jn in joints}
                cfg['constraints'] = {
                    'goal_time': 2.0,
                    'stopped_velocity_tolerance': 0.0001,
                    **per_joint,
                }
                controllers_yaml[cname] = cfg

        moveit_controllers = strip_none({
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': controllers_yaml,
        })

        nodes.append(
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[
                    description,
                    description_semantic,
                    description_kinematics,
                    execution_tuning,
                    moveit_controllers,
                    ompl_planning_pipeline_config,
                    description_joint_limits,
                    sim_time,
                ],
                # Remap both relative and absolute forms to be robust to different name resolution behaviors
                remappings=[
                    ('joint_states', f'{robot_type}_driver/out/joint_state'),
                    ('/joint_states', f'/{robot_type}_driver/out/joint_state'),
                ],
            )
        )

        return nodes

    return LaunchDescription([
        declare_robot_type,
        OpaqueFunction(function=setup_nodes),
    ])
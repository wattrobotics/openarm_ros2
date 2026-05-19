# Copyright 2025 Enactic, Inc.
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

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


CONFIG_DIR = "openarm_v1.0"
XACRO_REL_PATH = ("urdf", "robot", "v10.urdf.xacro")


def _xacro_path(description_package_str):
    return os.path.join(
        get_package_share_directory(description_package_str),
        *XACRO_REL_PATH,
    )


def generate_robot_description(context, description_package, use_fake_hardware,
                               right_can_interface):
    description_package_str = context.perform_substitution(description_package)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)

    return xacro.process_file(
        _xacro_path(description_package_str),
        mappings={
            "arm_type": "v10",
            "bimanual": "true",
            "include_left": "false",
            "include_right": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
        },
    ).toprettyxml(indent="  ")


def robot_nodes_spawner(context, description_package, use_fake_hardware,
                        controllers_file, right_can_interface):
    robot_description = generate_robot_description(
        context, description_package, use_fake_hardware, right_can_interface,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description_param],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description_param, controllers_file_str],
        ),
    ]


def controller_spawner(context, robot_controller):
    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[right, "-c", "/controller_manager"],
        )
    ]


def effort_controller_spawner():
    """Spawn the right_forward_effort_controller used for gravity feedforward."""
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_forward_effort_controller",
                "-c", "/controller_manager",
            ],
        )
    ]


def gravity_comp_node_launcher(context, description_package, use_fake_hardware,
                                right_can_interface):
    """Dump the active robot_description to /tmp and start gravity_comp_node.

    KDL parses URDF from a file path, so the xacro-expanded robot description
    is written to a temp file mirroring exactly what controller_manager sees.
    """
    robot_description = generate_robot_description(
        context, description_package, use_fake_hardware, right_can_interface,
    )
    urdf_path = "/tmp/openarm_v10_right_gravity.urdf"
    with open(urdf_path, "w") as f:
        f.write(robot_description)

    return [
        Node(
            package="openarm_gravity_comp",
            executable="gravity_comp_node",
            name="gravity_comp_node",
            output="screen",
            parameters=[{
                # Required: KDL loads from this file
                "urdf_path": urdf_path,
                # Start at 0.0 — operator ramps up with `ros2 param set
                # /gravity_comp_node g_scale <value>` after verifying sign.
                "g_scale": 0.0,
                "enable_right": True,
                "enable_left": False,
                "enable_compensation": True,
                "verbose": True,
            }],
        )
    ]


def moveit_nodes_spawner(context, description_package, use_fake_hardware):
    description_package_str = context.perform_substitution(description_package)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)

    moveit_pkg_path = get_package_share_directory("openarm_bimanual_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path=_xacro_path(description_package_str),
            mappings={
                "arm_type": "v10",
                "bimanual": "true",
                "include_left": "false",
                "include_right": "true",
                "use_fake_hardware": use_fake_hardware_str,
                "ros2_control": "true",
            },
        )
        .robot_description_semantic(
            file_path=f"config/{CONFIG_DIR}/openarm_right_arm.srdf")
        .robot_description_kinematics(
            file_path=f"config/{CONFIG_DIR}/kinematics.yaml")
        .joint_limits(file_path=f"config/{CONFIG_DIR}/joint_limits.yaml")
        .trajectory_execution(
            file_path=f"config/{CONFIG_DIR}/moveit_controllers_right.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()

    pilz_cartesian_limits_path = os.path.join(
        moveit_pkg_path, "config", CONFIG_DIR, "pilz_cartesian_limits.yaml"
    )
    if os.path.exists(pilz_cartesian_limits_path):
        import yaml
        with open(pilz_cartesian_limits_path, 'r') as f:
            config_data = yaml.safe_load(f)
            if "cartesian_limits" in config_data:
                moveit_params.setdefault(
                    "robot_description_planning", {}).update(config_data)

    rviz_cfg = os.path.join(moveit_pkg_path, "config", CONFIG_DIR, "moveit.rviz")

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_cfg],
            parameters=[moveit_params],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package",
                              default_value="openarm_description"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
        ),
        DeclareLaunchArgument("runtime_config_package",
                              default_value="openarm_bringup"),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_right_arm_moveit_controllers.yaml"),
        DeclareLaunchArgument(
            "enable_gravity_comp",
            default_value="false",
            description=(
                "Enable gravity compensation: spawns right_forward_effort_controller "
                "and starts gravity_comp_node. Operator must ramp g_scale via "
                "`ros2 param set /gravity_comp_node g_scale ...` (starts at 0.0)."),
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    right_can_interface = LaunchConfiguration("right_can_interface")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, use_fake_hardware, controllers_file,
              right_can_interface],
    )

    moveit_nodes_func = OpaqueFunction(
        function=moveit_nodes_spawner,
        args=[description_package, use_fake_hardware],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner, args=[robot_controller]
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "-c", "/controller_manager"],
    )

    enable_gravity_comp = LaunchConfiguration("enable_gravity_comp")

    effort_spawner_func = OpaqueFunction(
        function=lambda context: effort_controller_spawner()
    )
    gravity_comp_func = OpaqueFunction(
        function=gravity_comp_node_launcher,
        args=[description_package, use_fake_hardware, right_can_interface],
    )

    # spawner waits for controller_manager on its own; gravity_comp_node idles
    # until the first /joint_states arrives. No explicit delay needed.
    forward_effort_group = GroupAction(
        condition=IfCondition(enable_gravity_comp),
        actions=[effort_spawner_func, gravity_comp_func],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_spawner_func,
            moveit_nodes_func,
            TimerAction(period=2.0, actions=[jsb_spawner]),
            TimerAction(period=1.0, actions=[controller_spawner_func]),
            TimerAction(period=1.0, actions=[gripper_spawner]),
            forward_effort_group,
        ]
    )

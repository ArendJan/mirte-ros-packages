# Copyright 2023 ros2_control Development Team
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
import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "machine_namespace",
                default_value=TextSubstitution(
                    text=platform.node().replace("-", "_").lower()
                ),
                description="The namespace containing all Robot specific ROS communication",
            ),
            DeclareLaunchArgument(
                "hardware_namespace",
                default_value="io",
                description="The namespace for the Telemetrix Node and the hardware peripherals",
            ),
        ],
    )

    machine_namespace = LaunchConfiguration("machine_namespace")

    frame_prefix = LaunchConfiguration(
        "_frame_prefix", default=[machine_namespace, "/"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mirte_master_arm_control"),
                    "urdf",
                    "arm.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {
        "robot_description": robot_description_content,
        "frame_prefix": frame_prefix,
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mirte_master_arm_control"),
            "config",
            "mirte_master_arm_control.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ParameterFile(robot_controllers, allow_substs=True)],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
            ("~/tf_odometry", "/tf"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            # "pid_wheels_controller",
            # "mirte_master_arm_controller",
            "joint_follower_controller",
            # "position_trajectory_controller",
        ],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    # # Delay start of joint_state_broadcaster after `robot_controller`
    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        namespace="mirte_base_controller",
        remappings=[
            ("cmd_vel_out", "cmd_vel"),
            ("cmd_vel_in", "cmd_vel_unstamped"),
        ],
        parameters=[
            {
                "frame_id": (
                    frame_prefix,
                    TextSubstitution(text="base_link"),
                )
            }
        ],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        twist_stamper,
    ]
    ld.add_action(
        GroupAction(
            [
                # PushRosNamespace(machine_namespace),
                control_node,
                robot_state_pub_node,
                robot_controller_spawner,
                joint_state_broadcaster_spawner,
                # twist_stamper,
            ],
            launch_configurations={
                arg.name: LaunchConfiguration(arg.name)
                for arg in ld.get_launch_arguments()
            },
            forwarding=False,
        )
    )
    return ld

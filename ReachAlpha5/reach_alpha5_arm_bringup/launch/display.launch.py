# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to run the BlueROV2 Heavy Alpha system.

    Returns:
        The BlueROV2 Heavy Alpha ROS 2 launch description.
    """
    # Set some constants for this configuration
    description_package = "angler_description"
    configuration_type = "bluerov2_heavy_alpha"
    description_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "xacro",
            configuration_type,
            "config.xacro",
        ]
    )


    args = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "The prefix of the system. This is useful for multi-robot setups."
                " Expected format '<prefix>/'."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description=(
                "The namespace of the launched nodes. This is useful for multi-robot"
                " setups. If the namespace is changed, then the namespace in the"
                " controller configuration must be updated. Expected format '<ns>/'."
            ),
        ),

        DeclareLaunchArgument(
            "rviz_config",
            default_value="view_bluerov2_heavy_alpha.rviz",
            description="The RViz2 configuration file.",
        ),

        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="bluerov2_heavy_alpha_underwater.world",
            description="The Gazebo world file to launch.",
        ),
    ]

    use_sim = LaunchConfiguration("use_sim")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    prefix = LaunchConfiguration("prefix")
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")

    # Generate the robot description using xacro

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        description_file,
                    ]
                ),
                " ",
                "prefix:=",
                prefix,
                " ",
                "namespace:=",
                namespace,
                " ",
                "use_sim:=",
                use_sim,
            ]
        )
    }

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        namespace=namespace,
        parameters=[robot_description],
    )


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "rviz",
                    LaunchConfiguration("rviz_config"),
                ]
            ),
        ],
        parameters=[{"use_sim_time": use_sim, "robot_description": robot_description}],
        condition=IfCondition(use_rviz),
    )

    nodes = [
        robot_state_pub_node,
        rviz,
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{"use_sim_time": use_sim}],
        )
    ]

    return LaunchDescription(args + nodes)
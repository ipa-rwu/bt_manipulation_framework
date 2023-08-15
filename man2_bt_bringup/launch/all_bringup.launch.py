# Copyright (c) 2023 Ruichao Wu
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("man2_bt_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")

    lifecycle_nodes = [
        "bt_operator",
        "detect_aruco_marker_action_server",
        "moveit_skill_server",
        "set_path_constrains_server",
    ]

    moveit_config = (
        MoveItConfigsBuilder("ur5e_workcell", package_name="ur5e_cell_moveit_config")
        .robot_description(file_path="config/ur5e_workcell.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("man2_bt_bringup") + "/config/moveitcpp.yaml"
        )
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_dir, "config", "params.yaml"),
                description="Full path to the ROS2 parameters file to use",
            ),
            DeclareLaunchArgument(
                "default_bt_xml_filename",
                default_value=os.path.join(
                    get_package_share_directory("man2_bt_operator"),
                    "trees",
                    "simple_tree.xml",
                ),
                description="Full path to the behavior tree xml file to use",
            ),
            Node(
                package="man2_bt_operator",
                executable="bt_operator",
                name="bt_operator",
                output="screen",
                parameters=[
                    {"default_bt_xml_filename": default_bt_xml_filename},
                    {"use_sim_time": use_sim_time},
                    params_file,
                ],
            ),
            Node(
                package="detect_aruco_marker_skill",
                executable="detect_aruco_marker_action_server",
                name="detect_aruco_marker_action_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    params_file,
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
            Node(
                package="moveit_skills",
                executable="moveit_skill_server_node",
                name="moveit_skill_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    params_file,
                    moveit_config.to_dict(),
                ],
                # prefix=["xterm -e gdb -ex run --args"],
            ),
            Node(
                package="moveit_skills",
                executable="set_path_constrains",
                name="set_path_constrains_server",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
                # prefix=["xterm -e gdb -ex run --args"],
            ),
        ]
    )

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("man2_bt_operator")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    params_file = LaunchConfiguration("params_file")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("man2_bt_operator"),
            "trees",
            "simple_tree.xml",
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # Specify the actions
    start_bt_operator = Node(
        package="man2_bt_operator",
        executable="bt_operator",
        name="bt_operator",
        output="screen",
        parameters=[
            {"default_bt_xml_filename": default_bt_xml_filename},
            {"use_sim_time": use_sim_time},
            params_file,
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(start_bt_operator)

    return ld

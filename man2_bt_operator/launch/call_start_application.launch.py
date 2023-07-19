from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import FindExecutable

import yaml
from yaml import SafeLoader

document = """
    behavior_tree_filename: "simple_tree.xml" # absolute path
    run_in_loop: true
    sleep: 2000
    """


def generate_launch_description():
    ld = LaunchDescription()

    req = yaml.load(document, Loader=SafeLoader)

    cmd_str = 'action send_goal --feedback /start_application man2_msgs/action/RunApplication "{}"'.format(
        str(req)
    )

    ld.add_action(
        ExecuteProcess(
            cmd=[[FindExecutable(name="ros2"), " {}".format(cmd_str)]], shell=True
        )
    )
    return ld

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import FindExecutable

import yaml
from yaml import SafeLoader

document = """
    header:
      stamp:
        sec: 0
      frame_id: test
    markers:
      - header:
          stamp:
            sec: 0
          frame_id: test
        id: 42
        pose:
          pose:
            position:
              x: 1
              y: 1
              z: 1
            orientation:
              x: 1
              y: 1
              z: 1
              w: 1
        confidence: 100.0
    """


def generate_launch_description():
    ld = LaunchDescription()

    req = yaml.load(document, Loader=SafeLoader)

    cmd_str = 'topic pub -r 10 /marker aruco_msgs/msg/MarkerArray "{}"'.format(str(req))

    ld.add_action(
        ExecuteProcess(
            cmd=[[FindExecutable(name="ros2"), " {}".format(cmd_str)]], shell=True
        )
    )
    return ld

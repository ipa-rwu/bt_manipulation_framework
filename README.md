# Framework for Composing Applications for Manipulation from Skills

## Install
### Pre-requirement
* Install [groot2](https://www.behaviortree.dev/groot/)
* Get source code from BehahiorTree.CPP V4
    ```
    # in workspace folder
    vcs import < src/bt_manipulation_framework/upstream.repos
    ```

### Build
* use `rosdep` get depencices and `colcon`

### Configuration

```yaml
/bt_operator:
  ros__parameters:
    # default_bt_xml_filename: "simple_tree.xml" # define default BT tree that is saved in "bt_manipulation_framework/man2_bt_operator/trees"
    default_plugin_lib_names:
      - util_plugin_print_value # plugins created in "man2_bt_operator" package
    customized_plugin_lib_names: # Plugins created in other packages
      man2_bt_skills:  # package name
        - detect_aruco_marker_skill # plugin name

```

### Run
* Start Behavior Tree operator
```
ros2 launch man2_bt_operator bringup_tree.launch.py
```
* Configure lifecycle node
```
ros2 lifecycle set /bt_operator 1
ros2 lifecycle set /bt_operator 3
```

* Start an application
```
# start action client from launch file
ros2 launch man2_bt_operator call_start_application.launch.py
```
## Contributing
This repository uses pre-commit for code formatting. This program has to be setup locally and installed inside the repository. For this execute in the repository folder following commands:
```bash
sudo apt install -y pre-commit
pre-commit install
```
The check are automatically executed before each commit. This helps you to always commit well formatted code. To run all the checks manually use ``pre-commit run -a`` command. For the other options check ``pre-commit --help``.

In a case of an "emergency" you can avoid execution of pre-commit hooks by adding ``-n`` flag to git commit command - this is NOT recommended to do if you don't know what are you doing!

## Reference
The architecture is similar to [navigation2](https://github.com/ros-planning/navigation2)
It uses C++ template class from [nav2_behavior_tree](https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree) module for easily integrating ROS2 actions and services into Behavior Trees.

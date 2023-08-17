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
* get dependencies
```
rosdep install --from-paths src --ignore-src -r -y
```
* build
```
colcon build
```

### Usage and Contributing
[bt_manipulation_framework_guide](http://ipa326.pages.fraunhofer.de/demonstrator/bt_based_application_framework)

## Contributing
This repository uses pre-commit for code formatting. This program has to be setup locally and installed inside the repository. For this execute in the repository folder following commands:
```bash
sudo apt install -y pre-commit
pre-commit install
```
The check are automatically executed before each commit. This helps you to always commit well formatted code. To run all the checks manually use ``pre-commit run -a`` command. For the other options check ``pre-commit --help``.

In a case of an "emergency," you can avoid execution of pre-commit hooks by adding ``-n`` flag to git commit command - this is NOT recommended doing if you don't know what are you doing!

## Reference
The architecture is similar to [navigation2](https://github.com/ros-planning/navigation2)
It uses C++ template class from [nav2_behavior_tree](https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree) module for easily integrating ROS2 actions and services into Behavior Trees.

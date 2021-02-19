#man_behavior_tree_nodes
## description:
* action client or sercie client, rap with behavior tree action

## available skill nodes

### Service Node
If bt node provides service_name, it should be same as service server name

| BTAction ID   |      C++ lib name      | Service name |  parameters | description |
|----------|:-------------|------|------|------|
| UpdateParameter  | man_update_param_service_client_node | skill_update_param | data_type="double, bool, int", topic="dynmiac param topic" | get dynamic param, write into blackboard |
| UpdateGoalForArm  | man_update_goal_for_arm_service_client_node | update_arm_goal | initial_pose="seq;timestamp; frameid;pose"(must provide), step="task step"(optional), param="update y"(optional), goal_frame_id="for arm"(optional, if not provide will use world_fram_id)  | if step is defined, will read param of this step; if param is defined, will update z based on initial param, write into recovery_arm_parameter. If param is defined, z will keep adding this param |
| SetParameter  | man_set_parameter_node | skill_set_parameter_server | value="new param", label="param label", param_topic="dynmiac param topic" data_type="double, bool"| if not provide value, will use recovery_arm_parameter from blackboard|
| CallExternHelp | man_call_extern_help_service_client_node | call_extern_help | | |
### Action Nodes
| BTAction ID   |      C++ lib name      | default action name |  parameters | description |
|----------|:-------------|------|------|------|
|ComputePathArm |man_compute_path_action_client_node| compute_path | target_type="Name; Cartesian; Pose", server_name=""(default), allowed_planning_time="60.0(default)", replan_times="2(default)", if target_type=="Cartesian":{jump_threshold="0.0(default)" eef_step="0.01(default)"}  if target_type=="Pose/Name"{goal_name="Home(if Name)", goal="PoseStamped(if Pose)", end_effector="", max_velocity_scaling_factor="1.0(default)", max_acceleration_scaling_factor="1.0(default)", planner_id="RRTConnectkConfigDefault(default)"} | |
|ComputePathArm |man_compute_path_action_client_node| compute_path | | |
|ComputePathArm |man_compute_path_action_client_node| compute_path | | |
|ComputePathArm |man_compute_path_action_client_node| compute_path | | |
### Condition Nodes
| BTAction ID   |      C++ lib name  |  parameters | description |
|----------|:-------------|------|------|
| CheckTaskResult  | man_check_task_result_condition_node | task_name="Help" | check task result |

[man_update_param_service_client_node, 
                    man_find_objects_action_client_node,
                    ,
                    ,
                    man_execute_trajectory_arm_action_client_node,
                    man_execute_gripper_trajectory_action_client_node,
                    ,
                    ,
                    ]


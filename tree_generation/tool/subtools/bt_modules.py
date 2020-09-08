#! /usr/bin/env python
from read_task import GetTaskInfo
from read_plugin import GetPlugins
import xml.etree.ElementTree as ET
import os
class Create_BT_Module():
    def __init__(self):
        get_plugins = GetPlugins()
        self.plugin_data_loaded = get_plugins.load_plugin()
    
    def find_plugin(self, words=[]):
        data_loaded = self.plugin_data_loaded
        plugin_names = data_loaded["plugin_lib"]
        for plugin_name in plugin_names:
            for word in words:
                if word not in data_loaded[plugin_name]["name"]:
                    break
                if word == words[-1]:
                    return (True, plugin_name)
        return (False, None)

# Todo: auto generate code according to plugin
    # <SetBlackboard name="setTaskName" output_key="task_name" value="task2pick"/>
    def set_balkboard_module(self, name, output_key, value):
        attrib = {"name": name, "output_key": output_key, "value": value}
        tag = "SetBlackboard"
        return tag, attrib


    # <Condition ID="CheckTaskResult" task_name="FHelp"/>
    def check_result_module(self, task_name):    
        data_loaded = self.plugin_data_loaded
        (result, plugin_name) = self.find_plugin(["Check", "Result"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False
        
        attrib = {"ID": plugin_info["name"], "task_name": task_name}
      
        tag = "Condition"
        return tag, attrib

        # <Condition ID="SetFlagTask" success="true" task_name=""/>
    def set_flag_condition_module(self, task_name, is_succeeded):    
        data_loaded = self.plugin_data_loaded
        (result, plugin_name) = self.find_plugin(["Set", "Flag"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False
        
        attrib = {"ID": plugin_info["name"], "task_name": task_name, "success": is_succeeded}
      
        tag = "Condition"
        return tag, attrib
    
    
    # <UpdateParameter service_name="update_param" topic="/arm_param_server" data_type="double"/>
    def update_parameter_module(self, topic, data_type, service_name):
        data_loaded = self.plugin_data_loaded
        (result, plugin_name) = self.find_plugin(["Update", "Parameter"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False
        
        if service_name == None:
            attrib = {"ID": plugin_info["name"], "topic": topic, "data_type": data_type}
        else:
            attrib = {"ID": plugin_info["name"], 'service_name': service_name , "topic": topic, "data_type": data_type}
        tag = "Action"
        print(tag, attrib)
        return tag, attrib

    # <UpdateGoalForArm service_name="update_arm_goal" step="T2S1arm" goal_frame_id="world" based_target="{container}" goal="{arm_goal}"/>
    def update_goal_arm_module(self, step, goal_frame_id, based_on_pose, goal, service_name = None):
        data_loaded = self.plugin_data_loaded
        result, plugin_name = self.find_plugin(["Goal", "Arm"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False

        if service_name == None:
            attrib = {"ID": plugin_info["name"], "step": step, "goal_frame_id": goal_frame_id, "based_on_pose": based_on_pose, "goal": goal}
        else:
            attrib = {"ID": plugin_info["name"], 'service_name': service_name , "step": step, "goal_frame_id": goal_frame_id, "based_on_pose": based_on_pose, "goal": goal}
        
        tag = "Action"
        #print(tag, attrib)
        return tag, attrib
    
    # <Action ID="ComputePath" goal="" plan="" replan_times="" target_type=""/>
    def compute_path_module(self, goal, plan, replan_times, target_type):
        data_loaded = self.plugin_data_loaded
        result, plugin_name = self.find_plugin(["Compute", "Path"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False
        
        attrib = {"ID": plugin_info["name"], "goal": goal, "plan": plan, "replan_times":"1", "target_type": target_type}
        tag = "Action"

        return tag, attrib

    # for arm 
    # <Action ID="ExecuteTrajectoryArm" plan="" result=""/>
    # <Action ID="ExecuteTrajectoryArm" plan=""/>
    def execute_trajectory_arm_module(self, plan, is_succeeded = None):
        data_loaded = self.plugin_data_loaded
        result, plugin_name = self.find_plugin(["Execute", "Arm"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False

        if is_succeeded == None:
            attrib = {"ID": plugin_info["name"], "plan": plan}
        else:
            attrib = {"ID": plugin_info["name"], "plan": plan, "result": is_succeeded}
        
        tag = "Action"
        print(tag, attrib)
        #print(tag, attrib)
        return tag, attrib
    
    # for gripper
    # <Action ID="ExecuteGripperTrajectory" action_name="" result="" step=""/>
    def execute_trajectory_gripper_module(self, action_name = None, step = None, is_succeeded = None):
        data_loaded = self.plugin_data_loaded
        result, plugin_name = self.find_plugin(["Execute", "Gripper"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False

        if is_succeeded == None:
            if step == None:
                attrib = {"ID": plugin_info["name"], "action_name": action_name}
            else:
                attrib = {"ID": plugin_info["name"], "step": step}
        else:
            if step == None:
                attrib = {"ID": plugin_info["name"], "action_name": action_name, "result": is_succeeded}
            else:
                attrib = {"ID": plugin_info["name"], "step": step, "result": is_succeeded}
        
        tag = "Action"
        #print(tag, attrib)
        return tag, attrib

    # <Action ID="FindObjects" container="{container}" 
    #container_A="-0.35;0.55;0.78;0;0.707;0;0.707;" 
    #container_B="0.35;0.55;0.78;0;0.707;0;0.707" 
    #frame_id="world" marker="{marker}" marker_id="2"/>
    def find_object_module(self, container, marker_id, frame_id, container_A, container_B, marker):
        data_loaded = self.plugin_data_loaded
        result, plugin_name = self.find_plugin(["Find", "Object"])
        if result:
            plugin_info = data_loaded[plugin_name]
        
        attrib = {"ID": plugin_info["name"],
                    "container": container, 
                    "marker_id": marker_id, 
                    "frame_id": frame_id, 
                    "container_A": container_A, 
                    "container_B": container_B, 
                    "marker" : marker}
        tag = "Action"
        return tag, attrib

    def create_execution_module(self, single_step):
        for step_tuple in single_step:
                for num in step_tuple[2]:
                    print()
                    # single_action = step_tuple[2][num]
                    #print(step_tuple[0], step_tuple[1], step_tuple[2][num])
    
    # <SubTree ID="T1Detect" __shared_blackboard="true"/>
    def subtree_module(self, subtree_name, share = "true", ports={}):
        if not bool(ports):
            attrib = {"ID": subtree_name, " __shared_blackboard": share}
        else:
            for key, value in ports.items():
                attrib[key] = value
        tag = "SubTree"
        return tag, attrib


if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"

    read_task = GetTaskInfo()
    task_spec = read_task.load_task_specification(task_file)
    single_step = read_task.get_single_step(task_spec)
    
    bt_module = Create_BT_Module()
    parameter_module = bt_module.update_parameter_module(topic="arm_param_server", data_type="double", service_name=None)
    print(parameter_module)
    bt_module.create_execution_module(single_step)

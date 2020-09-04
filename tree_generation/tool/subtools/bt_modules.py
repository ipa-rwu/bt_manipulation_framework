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
    

    # <UpdateParameter service_name="update_param" topic="/arm_param_server" data_type="double"/>
    def parameter_module(self, topic, data_type, service_name):
        data_loaded = self.plugin_data_loaded
        (result, plugin_name) = self.find_plugin(["Update", "Parameter"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False
        
        if service_name == None:
            attrib = {"topic": topic, "data_type": data_type}
        else:
            attrib = {'service_name': service_name , "topic": topic, "data_type": data_type}
        tag = plugin_info["name"]
        print(tag, attrib)
        return tag, attrib

    # <UpdateGoalForArm service_name="update_arm_goal" step="T2S1arm" goal_frame_id="world" based_target="{container}" goal="{arm_goal}"/>
    def goal_module_arm(self, step, goal_frame_id, based_target, goal):
        data_loaded = self.plugin_data_loaded
        result, plugin_name = self.find_plugin(["Goal", "Arm"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False

        if service_name == None:
            attrib = {"step": step, "goal_frame_id": goal_frame_id, "based_target": based_target, "goal": goal}
        else:
            attrib = {'service_name': service_name , "step": step, "goal_frame_id": goal_frame_id, "based_target": based_target, "goal": goal}
            tag = plugin_info["name"]
        print(tag, attrib)
        return tag, attrib

    def arm_module(self, action):
        if action == "move":
            #  create compute path module
            # <ComputePath goal_name="home" target_type="Name" replan_times="3" plan="{plan}"/>
            # <ComputePath goal="{arm_goal}" target_type="Pose" replan_times="3" plan="{plan}"/>
            (result, plugin_name) = self.find_plugin(["Compute", "Arm"])
            if result:
                print(plugin_name)


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
    parameter_module = bt_module.parameter_module(topic="arm_param_server", data_type="double", service_name=None)
    print(parameter_module)
    bt_module.create_execution_module(single_step)

#! /usr/bin/env python
from read_task import GetSingleStep
from read_plugin import GetPlugins
import xml.etree.ElementTree as ET

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

    # <UpdateParameter service_name="update_param" topic="/arm_param_server" data_type="double"/>
    def parameter_module(self, topic, data_type, service_name):
        data_loaded = self.plugin_data_loaded
        (result, plugin_name) = self.find_plugin(["Update", "Parameter"])
        if result:
            plugin_info = data_loaded[plugin_name]
        else:
            return False
        
        if service_name == None:
            service_name = plugin_info["input"]["service_name"]  
        attrib = {'service_name': service_name , "topic": topic, "data_type": data_type}
        title = plugin_info[name]
        print(attrib)
        return title, attrib

    def goal_module_arm(self):
            (result, plugin_name) = self.find_plugin(["Compute", "Arm"])
            if result:
                print(plugin_name)

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

if __name__ == "__main__":
    read_task = GetSingleStep()
    single_step = read_task.get_single_step(read_task.load_task_specification())
    
    bt_module = Create_BT_Module()
    arm_module = bt_module.parameter_module(topic="arm_param_server", data_type="double", service_name=None)
    bt_module.create_execution_module(single_step)

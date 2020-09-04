#! /usr/bin/env python
import os 
import xml.etree.ElementTree as ET
from bt_modules import Create_BT_Module
from read_task import GetTaskInfo

class CreateBTStructure():
    def __init__(self, application_name, template_path = None, created_bt_xml_path = None):
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if template_path == None :
            template_path = "/template/bt_template.xml"
        self.bt_template_path = BASE_DIR + template_path
        rm_size = len("/tool")
        bt_xml_dir = BASE_DIR[:-rm_size]
        if created_bt_xml_path == None:
            self.created_bt_xml = bt_xml_dir + "/examples/pick_and_place/generated_file/bt_xml/" + application_name 
        else:
            self.created_bt_xml_path = created_bt_xml_path
        self.bt_modules = Create_BT_Module()
    
    def get_bt_template(self, bt_template_path = None, created_bt_xml_path = None):
        if bt_template_path == None:
            bt_template_path = self.bt_template_path
        if created_bt_xml_path == None:
            created_bt_xml_path = self.created_bt_xml
        tree = ET.parse(bt_template_path)
        root = tree.getroot()
        print(root[0].tag,   root[0].attrib)
        tree.write(created_bt_xml_path, xml_declaration=True, encoding='utf-8')
    
    def update_created_bt_xml(self, created_bt_xml_path = None):
        if created_bt_xml_path == None:
            created_bt_xml_path = self.created_bt_xml
        tree = ET.parse(created_bt_xml_path)
        root = tree.getroot()
        self.indent(root)
        # print(root[0].tag,   root[0].attrib)
        return tree, root

    def indent(self, elem, level=0):
        i = "\n" + level*"  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.indent(elem, level+1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def maintree(self, task_names, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]
        
        # add tasks or task
        counter_root = 0
        counter_maintree = 1
        for lev_1 in root:
            if lev_1.attrib["ID"] == "MainTree":
                for name in task_names:
                    # add task <Fallback name="Task1">
                    attrib = {"name": name[0]}
                    element = root[counter_root][0].makeelement('Fallback', attrib)
                    root[counter_root][0].append(element)
                    
                    # <SetBlackboard name="setTaskName" output_key="task_name" value="task2pick"/>
                    set_balkboard_module = self.bt_modules.set_balkboard_module( \
                        name = "setTaskName", output_key = "current_task_name", value = name[0]+name[1])
                    root[counter_root][0][counter_maintree].makeelement(set_balkboard_module[0], set_balkboard_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree], set_balkboard_module[0], set_balkboard_module[1])

                    # <Action ID="CheckTaskResult" task_name="FHelp" />
                    check_result_module = self.bt_modules.check_result_module(task_name = "{current_task_name}")
                    root[counter_root][0][counter_maintree].makeelement(check_result_module[0], check_result_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree], check_result_module[0], check_result_module[1])
                    # <SubTree  __shared_blackboard="true" ID="task2pick" />
                    subtree_module = self.bt_modules.subtree_module(subtree_name =  name[0]+name[1])
                    root[counter_root][0][counter_maintree].makeelement(subtree_module[0], subtree_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree], subtree_module[0], subtree_module[1])
                    # <SubTree ID="NeedHelp" __shared_blackboard="true"/>
                    subtree_module = self.bt_modules.subtree_module(subtree_name = "NeedHelp")
                    root[counter_root][0][counter_maintree].makeelement(subtree_module[0], subtree_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree], subtree_module[0], subtree_module[1])
                    counter_maintree += 1
            else:
                counter_root += 1

        tree.write(self.created_bt_xml, xml_declaration=True, encoding='utf-8')

    # param = (topic, data_type, service_name)
    # <UpdateParameter service_name="update_param" topic="/arm_param_server" data_type="double"/>
    def initial_subtree(self, param_info={}, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]
        
        update_parameter_module = self.bt_modules.update_parameter_module(topic = param_info["topic"], \
                                                            data_type = param_info["data_type"], \
                                                            service_name = param_info["service_name"])
        counter = 0
        for lev_1 in root:
            if lev_1.attrib["ID"] == "Initialization":
                root[counter][0].makeelement(update_parameter_module[0], update_parameter_module[1])
                ET.SubElement(root[counter][0], update_parameter_module[0], update_parameter_module[1])
                break
            else:
                counter += 1

        tree.write(self.created_bt_xml, xml_declaration=True, encoding='utf-8')

    #  <BehaviorTree ID="NeedHelp">
    #     <Condition ID="SetFlagTask" success="true" task_name=""/>
    # </BehaviorTree>   
    def need_help_subtree(self, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]

        # append NeedHep subtree
        attrib = {"ID": "NeedHelp"}
        element = root.makeelement('BehaviorTree', attrib)
        root.append(element)

        set_flag_condition_module = self.bt_modules.set_flag_condition_module(task_name = "{current_task_name}", issuccess="false")
        print(set_flag_condition_module)

        counter_root = 0
        for lev_1 in root:
            print(lev_1.attrib)    
            if lev_1.attrib["ID"] == "NeedHelp":               
                root[counter_root].makeelement(set_flag_condition_module[0], set_flag_condition_module[1])
                ET.SubElement(root[counter_root], set_flag_condition_module[0], set_flag_condition_module[1])
                break
            else:
                counter_root += 1
        
        tree.write(self.created_bt_xml, xml_declaration=True)
    
    #     <BehaviorTree ID="T2Pick">
    #       <Sequence name="T2Steps">       
    #         <Sequence name="T2Step1">
    #             <Action ID="UpdateGoalForArm" step="T2S1arm" goal_frame_id="world" target="{container}" goal="{arm_goal}"/>
    #             <Action ID="ComputePath" goal_name="" plan="" replan_times="" target_type=""/>
    #             <SubTree ID="Execution" __shared_blackboard="true"/>
    #         </Sequence>
    #         <Sequence name="T2Step2">
    #             <Action ID="UpdateGoalForArm" step="T2S1arm" goal_frame_id="world" target="{container}" goal="{arm_goal}"/>
    #             <Action ID="ComputePath" goal_name="" plan="" replan_times="" target_type=""/>
    #             <SubTree ID="Execution" __shared_blackboard="true"/>
    #         </Sequence>
    #         <Sequence name="T2Finish">
    #             <Condition ID="SetFlagTask" success="true" task_name=""/>
    #         </Sequence>
    #       </Sequence>
    #    </BehaviorTree>
    def task_subtree(self, task_name, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]

        # append task subtree
        attrib = {"ID": task_name}
        element = root.makeelement('BehaviorTree', attrib)
        root.append(element)



    # <Sequence name="T2Step2">
    #     <Action ID="UpdateGoalForArm" step="T2S1arm" goal_frame_id="world" target="{container}" goal="{arm_goal}"/>
    #     <Action ID="ComputePath" goal_name="" plan="" replan_times="" target_type=""/>
    #     <SubTree ID="Execution" __shared_blackboard="true"/>
    # </Sequence>
    def step_module(self, step_name, ):





if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"

    create_bt = CreateBTStructure(application_name = "pick_and_place.xml")
    create_bt.get_bt_template(bt_template_path = None, created_bt_xml_path = None)

    read_task = GetTaskInfo()

    # first write Initialization
    params = [  {"topic": "/arm_param_server", \
                "data_type": "double", \
                "service_name": None},
                {"topic": "/gripper_param_server", \
                "data_type": "string", \
                "service_name": None},
                {"topic": "/flag_param_server", \
                "data_type": "bool", \
                "service_name": None}]
    for param_info in params:
        create_bt.initial_subtree(param_info = param_info, created_bt_xml_path = None)

    # start from "job" or "task"
    # add tasks in maintree
    task_spec = read_task.load_task_specification(task_file)
    tasks = read_task.get_tasks(task_spec)
    task_name = read_task.get_task_name(tasks)
    create_bt.maintree(task_names = task_name, created_bt_xml_path = None)

    #add subtree
    create_bt.need_help_subtree(created_bt_xml_path = None)
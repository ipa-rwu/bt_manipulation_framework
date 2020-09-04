#! /usr/bin/env python
import os 
import xml.etree.ElementTree as ET
from bt_modules import Create_BT_Module
from read_task import GetTaskInfo

class CreateBTStructure():
    def __init__(self, application_name, template_path = None ):
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if template_path == None :
            template_path = "/template/bt_template.xml"
        self.bt_template_path = BASE_DIR + template_path
        rm_size = len("/tool")
        bt_xml_dir = BASE_DIR[:-rm_size]
        self.created_bt_xml = bt_xml_dir + "/examples/pick_and_place/generated_file/bt_xml/" + application_name 

        self.bt_modules = Create_BT_Module()
    
    def get_bt_template(self):
        tree = ET.parse(self.bt_template_path)
        root = tree.getroot()
        print(root[0].tag,   root[0].attrib)
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

    def maintree(self,template_xml, task_names):
        tree = template_xml[0]
        root = template_xml[1]
        
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
                    # <Action ID="CheckTaskResult" task_name="FHelp" />
                    check_result_module = self.bt_modules.check_result_module(task_name = name[0]+name[1])
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

    # param = (topic, data_type, service_name)
    # <UpdateParameter service_name="update_param" topic="/arm_param_server" data_type="double"/>
    def initial_subtree(self, template_xml, param_info={}):
        tree = template_xml[0]
        root = template_xml[1]
        parameter_module = self.bt_modules.parameter_module(topic = param_info["topic"], \
                                                            data_type = param_info["data_type"], \
                                                            service_name = param_info["service_name"])
        counter = 0
        for lev_1 in root:
            if lev_1.attrib["ID"] == "Initialization":
                root[counter][0].makeelement(parameter_module[0], parameter_module[1])
                ET.SubElement(root[counter][0], parameter_module[0], parameter_module[1])
                break
            else:
                counter += 1
        self.indent(root)
        tree.write(self.created_bt_xml, xml_declaration=True, encoding='utf-8')
    
    #def need_help_subtree(self):


if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"

    create_bt = CreateBTStructure(application_name = "pick_and_place.xml")
    bt_template = create_bt.get_bt_template()

    read_task = GetTaskInfo()
    # start from "job" or "task"
    task_spec = read_task.load_task_specification(task_file)
    tasks = read_task.get_tasks(task_spec)
    task_name = read_task.get_task_name(tasks)
    create_bt.maintree(bt_template, task_name)
    

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
        create_bt.initial_subtree(bt_template, param_info)

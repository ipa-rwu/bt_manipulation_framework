# generate cfg based on task specification
import os
import subprocess
import xml.etree.ElementTree as ET
from read_task import GetTaskInfo


class ParameterGenerator():
    def __init__(self):
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        print(BASE_DIR)
        self.template_path = BASE_DIR + "/template/dynamic_parameter_template.ros_package"
        if BASE_DIR.endswith('/manipulator_bt/tree_generation/tool'):
            rm_size = len("/manipulator_bt/tree_generation/tool")
            self.workspace_dir = BASE_DIR[:-rm_size]
    
    def modify_template(self, template_path, workspace_dir, package_name, author_name, author_email, param_dic):
        if template_path == None:
            template_path = self.template_path
        if workspace_dir == None:
            workspace_dir = self.workspace_dir
        
        package_template_path = workspace_dir + "/{}.ros_package".format(package_name)
        
        tree = ET.parse(template_path)
        root = tree.getroot()
        #print (root.tag, root.attrib)
        for x in root:
            print (x.tag, x.attrib)
            #self.print_subtree(x.getchildren())
        for package_meta_info in root.iter('package'):
            package_meta_info.set("author", author_name)
            package_meta_info.set("author_email", author_email)
            package_meta_info.set("name", package_name)
            package_meta_info.set("description", package_name)
        
        for component in root.iter('component'):
            component.set("name", package_name)
    
        
        # adding an element to the seconditem node
        for key in param_dic.keys():
            if type(param_dic[key]) is float:
                param_type = "double"
            if type(param_dic[key]) is str:
                param_type = "string"
            if type(param_dic[key]) is bool:
                param_type = "bool"
            attrib = {'description': package_name, "name": key, "type": param_type, "value": "{}".format(param_dic[key])}
            print(attrib)
            subelement = root[0].makeelement('dynParameter', attrib)
            ET.SubElement(root[0], 'dynParameter', attrib)

        self.indent(root)
        tree.write(package_template_path)

    def print_subtree(self, subtree):
        for y in subtree:
            print ("\t", y.tag, ":", y.text)

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

    
    def generate(self, template_name):
        subprocess.run(["rosrun", "package_generator", "generate_package", template_name])
    
if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"

    read_task = GetTaskInfo()
    job_spec = read_task.load_task_specification(task_file)
    steps = read_task.get_single_step(job_spec)

    parameter_generator = ParameterGenerator()

    arm_param_dic = read_task.get_param(steps= steps, param_object="arm")
    parameter_generator.modify_template(template_path = None,
                                        workspace_dir = None,
                                        package_name = "arm_parameter_server", 
                                        author_name="RWU", 
                                        author_email="rwu@ipa.fraunhofer.de", 
                                        param_dic = arm_param_dic)

    gripper_param_dic = read_task.get_param(steps= steps, param_object="gripper")
    parameter_generator.modify_template(template_path = None,
                                        workspace_dir = None,
                                        package_name = "gripper_parameter_server", 
                                        author_name="RWU", 
                                        author_email="rwu@ipa.fraunhofer.de", 
                                        param_dic = gripper_param_dic)

    task_names = read_task.get_task_name(data_loaded = job_spec)
    task_param_dic = {}
    for task in task_names.keys():
        task_param_dic[task.capitalize()+task_names[task].capitalize()] = False
    task_param_dic["Help"] = False

    parameter_generator.modify_template(template_path = None,
                                        workspace_dir = None,
                                        package_name = "task_flag_parameter_server", 
                                        author_name="RWU", 
                                        author_email="rwu@ipa.fraunhofer.de", 
                                        param_dic = task_param_dic)
    


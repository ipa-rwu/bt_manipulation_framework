# generate cfg based on task specification
import os
import subprocess
import xml.etree.ElementTree as ET


class parameter_generator():
    def __init__(self, package_name, author_name, author_email, target, template_path = None):
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        print(BASE_DIR)
        template_path = BASE_DIR + "/template/dynamic_parameter_template.ros_package"
        if BASE_DIR.endswith('/manipulator_bt/tree_generation/tool'):
            rm_size = len("/manipulator_bt/tree_generation/tool")
            workspace_dir = BASE_DIR[:-rm_size]
        package_path = workspace_dir + "/arm_parameter_server.ros_package"
        self.modify_template(template_path, package_path, package_name, author_name, author_email, target)

    # def get_parameter(self, target):
    
    def modify_template(self,template_path, package_path, package_name, author_name, author_email, target):
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
    
        print(root[0].attrib)
        
        # adding an element to the seconditem node
        attrib = {'description': target, "name": "T2S1", "type": "double", "value": "0.1"}
        subelement = root[0].makeelement('dynParameter', attrib)
        ET.SubElement(root[0], 'dynParameter', attrib)
        ET.SubElement(root[0], 'dynParameter', attrib)
        tree.write(package_path)

    def print_subtree(self, subtree):
        for y in subtree:
            print ("\t", y.tag, ":", y.text)



    
    def generate(self, template_name):
        subprocess.run(["rosrun", "package_generator", "generate_package", template_name])
    
if __name__ == "__main__":
    parameter_generator = parameter_generator("arm_parameter_server", "RWU", "rwu@ipa.fraunhofer.de", "arm")

    


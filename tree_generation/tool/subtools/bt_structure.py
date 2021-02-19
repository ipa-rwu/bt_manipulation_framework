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
                for label in task_names.keys():
                    # add task <Fallback name="Task1">
                    attrib = {"name": label.capitalize()}
                    element = root[counter_root][0].makeelement('Fallback', attrib)
                    root[counter_root][0].append(element)
                    
                    attrib = {}
                    root[counter_root][0][counter_maintree].makeelement("Inverter", attrib)
                    ET.SubElement(root[counter_root][0][counter_maintree], "Inverter", attrib)
                    # <SetBlackboard name="setTaskName" output_key="task_name" value="Task2Pick"/>
                    set_balkboard_module = self.bt_modules.set_balkboard_module( \
                        name = "setTaskName", output_key = "current_task_name", value = label.capitalize() + task_names[label].capitalize())
                    root[counter_root][0][counter_maintree][0].makeelement(set_balkboard_module[0], set_balkboard_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree][0], set_balkboard_module[0], set_balkboard_module[1])

                    # <Action ID="CheckTaskResult" task_name="FHelp" />
                    check_result_module = self.bt_modules.check_result_module(task_name = "{current_task_name}")
                    root[counter_root][0][counter_maintree].makeelement(check_result_module[0], check_result_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree], check_result_module[0], check_result_module[1])
                    # <SubTree  __shared_blackboard="true" ID="Task2Pick" />
                    subtree_module = self.bt_modules.subtree_module(subtree_name =  label.capitalize() + task_names[label].capitalize())
                    root[counter_root][0][counter_maintree].makeelement(subtree_module[0], subtree_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree], subtree_module[0], subtree_module[1])

                    attrib = {}
                    root[counter_root][0][counter_maintree].makeelement("Inverter", attrib)
                    ET.SubElement(root[counter_root][0][counter_maintree], "Inverter", attrib)
                    # <SubTree ID="NeedHelp" __shared_blackboard="true"/>
                    subtree_module = self.bt_modules.subtree_module(subtree_name = "NeedHelp")
                    root[counter_root][0][counter_maintree][3].makeelement(subtree_module[0], subtree_module[1])
                    ET.SubElement(root[counter_root][0][counter_maintree][3], subtree_module[0], subtree_module[1])
                    counter_maintree += 1
                
                # add Finish
                # <Sequence name="Finish">
                #     <Action ID="ASetFlag" flagupdate="{Param}" name="ASetFlagTask1" param="{Param}" task="1" value="false"/>
                #     <Action ID="ASetFlag" flagupdate="{Param}" name="ASetFlagTask2" param="{Param}" task="2" value="false"/>
                #     <Action ID="ASetFlag" flagupdate="{Param}" name="ASetFlagTask3" param="{Param}" task="3" value="false"/>
                # </Sequence>
                attrib = {"name": "AllFinish"}
                element = root[counter_root][0].makeelement('Sequence', attrib)
                root[counter_root][0].append(element)

                counter_lev3 = 0
                for lev3 in root[counter_root][0]:
                    if lev3.attrib["name"] == "AllFinish":
                        for label in task_names.keys():
                            set_flag_condition_module = self.bt_modules.set_flag_condition_module(task_name = label.capitalize() + task_names[label].capitalize(), 
                                                                                        is_succeeded="false")
                            root[counter_root][0][counter_lev3].makeelement(set_flag_condition_module[0], set_flag_condition_module[1])
                            ET.SubElement(root[counter_root][0][counter_lev3], set_flag_condition_module[0], set_flag_condition_module[1])
                    else:
                        counter_lev3 += 1
            else:
                counter_root += 1
        

        tree.write(self.created_bt_xml, xml_declaration=True, encoding='utf-8')

    def solution_maintree(self, task_names, step_name, retry_times, param, sub_steps, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]
        
        # add tasks or task
        counter_root = 0
        counter_maintree = 1
        for lev_1 in root:
            if lev_1.attrib["ID"] == "MainTree":
                for label in task_names.keys():
                    for lev2 in root[counter_root]:
                        lev2.set('name', task_names[label])
                    for lev3 in root[counter_root][0]:
                        if lev3.tag == "RetryUntilSuccesful":
                            lev3.set('num_attempts', str(retry_times))
                            for steps_level2 in sub_steps.keys():
                                info = sub_steps[steps_level2]
                                if info["object"] == "arm":
                                    update_goal_arm_module = self.bt_modules.update_goal_arm_module(step = step_name, 
                                                                    #goal_frame_id = "world", 
                                                                    goal_frame_id = None,
                                                                    initial_pose = "{" + info["base"] + "}", 
                                                                    goal="{arm_goal}",
                                                                    param = str(param),
                                                                    service_name = None)
                                    break
                            root[counter_root][0][1][0].makeelement(update_goal_arm_module[0], update_goal_arm_module[1])
                            ET.SubElement(root[counter_root][0][1][0], update_goal_arm_module[0], update_goal_arm_module[1])

                            compute_path_module = self.bt_modules.compute_path_module(replan_times = "1", 
                                                    target_type = "Cartesian", 
                                                    plan = "{plan}", 
                                                    goal="{arm_goal}",
                                                    planner_id = None)
                            root[counter_root][0][1][0].makeelement(compute_path_module[0], compute_path_module[1])
                            ET.SubElement(root[counter_root][0][1][0], compute_path_module[0], compute_path_module[1])

                            execute_trajectory_arm_module = self.bt_modules.execute_trajectory_arm_module(
                                plan = "{plan}")
                            # print(execute_trajectory_arm_module[0], execute_trajectory_arm_module[1])
                            root[counter_root][0][1][0].makeelement(execute_trajectory_arm_module[0], execute_trajectory_arm_module[1])
                            ET.SubElement(root[counter_root][0][1][0], execute_trajectory_arm_module[0], execute_trajectory_arm_module[1])

                    # <RetryUntilSuccesful num_attempts="3">
                    #     <Sequence>
                    #         <Action ID="UpdateGoalForArm" initial_pose="{marker}" goal="" step="task2step1"/>
                    #         <Action ID="ComputePathArm" goal="" plan="" replan_times="" target_type=""/>
                    #         <Action ID="ExecuteTrajectoryArm" plan="" topic_name=""/>
                    #     </Sequence>
                    # </RetryUntilSuccesful>
                    # add task <Fallback name="Task1">


            else:
                counter_root += 1
        
        self.indent(root)
        tree.write(self.created_bt_xml, xml_declaration=True, encoding='utf-8')

    def update_solution_initialization(self,created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1] 
        counter = 0
        for lev_1 in root:
            if lev_1.attrib["ID"] == "Initialization":
                find_object_module = self.bt_modules.find_object_module(marker_id = "1", \
                                    frame_id="world", \
                                    marker="{marker}", \
                                    container_A="-0.35;0.55;0.78;0;0.707;0;0.707;",\
                                    container_B="0.35;0.55;0.78;0;0.707;0;0.707",\
                                    container="{container}")
                root[counter][0].makeelement(find_object_module[0], find_object_module[1])
                ET.SubElement(root[counter][0], find_object_module[0], find_object_module[1])
                break
            else:
                counter += 1
        self.indent(root)
        tree.write(self.created_bt_xml, xml_declaration=True)

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
    #     <Condition ID="SetFlagTask" success="true" task_name="Help"/>
    # </BehaviorTree>   
    def need_help_subtree(self, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]

        # append NeedHep subtree
        attrib = {"ID": "NeedHelp"}
        element = root.makeelement('BehaviorTree', attrib)
        root.append(element)



        set_flag_condition_module = self.bt_modules.set_flag_condition_module(task_name = "{current_task_name}", is_succeeded="false")
        #print(set_flag_condition_module)

        counter_root = 0
        for lev_1 in root:
            #print(lev_1.attrib)    
            if lev_1.attrib["ID"] == "NeedHelp":  
                attrib = {"name": "setHelpsteps"}
                element = root[counter_root].makeelement('Sequence', attrib)
                root[counter_root].append(element)     

                root[counter_root][0].makeelement(set_flag_condition_module[0], set_flag_condition_module[1])
                ET.SubElement(root[counter_root][0], set_flag_condition_module[0], set_flag_condition_module[1])

                set_flag_condition_module = self.bt_modules.set_flag_condition_module(task_name = "Help", is_succeeded="true")
                root[counter_root][0].makeelement(set_flag_condition_module[0], set_flag_condition_module[1])
                ET.SubElement(root[counter_root][0], set_flag_condition_module[0], set_flag_condition_module[1])
                break
            else:
                counter_root += 1
        
        tree.write(self.created_bt_xml, xml_declaration=True)

    # <BehaviorTree ID="Execution">
    #    <Sequence>
    #         <Action ID="ExecuteTrajectory" plan="" result=""/>
    #        <Action ID="ExecuteGripperTrajectory" action_name="" result="" step=""/>
    #    </Sequence>
    # </BehaviorTree>
    def execution_subtree(self, plan = None, action_name = None, step = None, sequence_name = None, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]

        # append NeedHep subtree
        attrib = {"ID": "subExecution"}
        element = root.makeelement('BehaviorTree', attrib)
        root.append(element)

        if sequence_name == None:
            attrib = {"name": "Sequence"}  
        else:
            attrib = {"ID": sequence_name}  

        counter_root = 0
        for lev_1 in root:
            print(lev_1.attrib)    
            if lev_1.attrib["ID"] == "subExecution":       
                root[counter_root].makeelement("Sequence", attrib)
                ET.SubElement(root[counter_root], "Sequence", attrib)

                # add action
                execute_trajectory_arm_module = self.bt_modules.execute_trajectory_arm_module(
                                                plan = plan)
                # print(execute_trajectory_arm_module[0], execute_trajectory_arm_module[1])
                root[counter_root][0].makeelement(execute_trajectory_arm_module[0], execute_trajectory_arm_module[1])
                ET.SubElement(root[counter_root][0], execute_trajectory_arm_module[0], execute_trajectory_arm_module[1])
                execute_trajectory_gripper_module = self.bt_modules.execute_trajectory_gripper_module(
                                                        action_name = action_name,
                                                        step = step
                                                    )
                root[counter_root][0].makeelement(execute_trajectory_gripper_module[0], execute_trajectory_gripper_module[1])
                ET.SubElement(root[counter_root][0], execute_trajectory_gripper_module[0], execute_trajectory_gripper_module[1])                          
                break
            else:
                counter_root += 1
                
        self.indent(root)
        tree.write(self.created_bt_xml, xml_declaration=True)

    
    def create_subtree(self, subtree_name, sequence_name = None, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]

        # append task subtree
        attrib = {"ID": subtree_name}
        element = root.makeelement('BehaviorTree', attrib)
        root.append(element)

        if sequence_name != None:
            counter_root = 0
            for lev_1 in root:
                if lev_1.attrib["ID"] == subtree_name:     
                    attrib = {"name": sequence_name}  
                    root[counter_root].makeelement("Sequence", attrib)
                    ET.SubElement(root[counter_root], "Sequence", attrib)
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
    def task_step(self, task_name, step_name, sub_steps, created_bt_xml_path = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]  

        counter_root = 0
        for lev_1 in root:
            if lev_1.attrib["ID"] == task_name:    
                # append step subtree
                # <Sequence name="T2Step1">
                attrib = {"name": step_name}
                root[counter_root][0].makeelement("Sequence", attrib)
                ET.SubElement(root[counter_root][0], "Sequence", attrib)

                counter_step = 0 
                for lev_3 in root[counter_root][0]:
                    if lev_3.attrib["name"] != step_name:
                        counter_step += 1
                    else:
                        break

                # <Action ID="UpdateGoalForArm" step="T2S1arm" goal_frame_id="world" target="{container}" goal="{arm_goal}"/>
                # <Action ID="ComputePath" goal="" plan="" replan_times="" target_type=""/>
                # <SubTree ID="Execution" __shared_blackboard="true"/>
                for steps_level2 in sub_steps.keys():
                    info = sub_steps[steps_level2]
                    
                    if info["object"] == "arm":
                        update_goal_arm_module = self.bt_modules.update_goal_arm_module(step = step_name, 
                                                            #goal_frame_id = "world", 
                                                            goal_frame_id = None,
                                                            initial_pose = "{" + info["base"] + "}", 
                                                            goal="{arm_goal}",
                                                            service_name = None,
                                                            param = None)
                        root[counter_root][0][counter_step].makeelement(update_goal_arm_module[0], update_goal_arm_module[1])
                        ET.SubElement(root[counter_root][0][counter_step], update_goal_arm_module[0], update_goal_arm_module[1])
                        
                        compute_path_module = self.bt_modules.compute_path_module(replan_times = "1", 
                                                            target_type = "Cartesian", 
                                                            plan = "{plan}", 
                                                            goal="{arm_goal}",
                                                            planner_id = None)
                        root[counter_root][0][counter_step].makeelement(compute_path_module[0], compute_path_module[1])
                        ET.SubElement(root[counter_root][0][counter_step], compute_path_module[0], compute_path_module[1])

                        execution_subtree = self.bt_modules.subtree_module(subtree_name="subExecution", share="true")
                        root[counter_root][0][counter_step].makeelement(execution_subtree[0], execution_subtree[1])
                        ET.SubElement(root[counter_root][0][counter_step], execution_subtree[0], execution_subtree[1])

                   # <Action ID="FindObjects" container="{container}" 
                    #container_A="-0.35;0.55;0.78;0;0.707;0;0.707;" 
                     #container_B="0.35;0.55;0.78;0;0.707;0;0.707" 
                        #frame_id="camera" marker="{marker}" marker_id="2"/>
                    if info["object"] == "camera":
                        find_object_module = self.bt_modules.find_object_module(marker_id = "{}".format(info["marker_id"]), 
                                                            frame_id="world", 
                                                            marker="{marker}", 
                                                            container_A="-0.35;0.55;0.78;0;0.707;0;0.707;",
                                                            container_B="0.35;0.55;0.78;0;0.707;0;0.707",
                                                            container="{container}")

                        root[counter_root][0][0].makeelement(find_object_module[0], find_object_module[1])
                        ET.SubElement(root[counter_root][0][0], find_object_module[0], find_object_module[1])
            else:
                    counter_root += 1

        tree.write(self.created_bt_xml, xml_declaration=True)


    #         <Sequence name="T2Finish">
    #             <Condition ID="SetFlagTask" success="true" task_name=""/>
    #         </Sequence>
    def add_finish_step(self, task_name, task_num, created_bt_xml_path  = None):
        created_bt_xml = self.update_created_bt_xml(created_bt_xml_path)
        tree = created_bt_xml[0]
        root = created_bt_xml[1]  

        counter_root = 0
        for lev_1 in root:
            if lev_1.attrib["ID"] == task_name:    
                # append step subtree
                # <Sequence name="T2Step1">
                attrib = {"name": task_num+"finish"}
                root[counter_root][0].makeelement("Sequence", attrib)
                ET.SubElement(root[counter_root][0], "Sequence", attrib)

                counter_lev3 = 0
                for lev_3 in root[counter_root][0]:
                    if lev_3.attrib["name"] == task_num+"finish":
                        set_flag_condition_module = self.bt_modules.set_flag_condition_module(
                            task_name = "{current_task_name}", 
                            is_succeeded = "true")

                        root[counter_root][0][counter_lev3].makeelement(set_flag_condition_module[0], set_flag_condition_module[1])
                        ET.SubElement(root[counter_root][0][counter_lev3], set_flag_condition_module[0], set_flag_condition_module[1])
                    else:
                        counter_lev3 += 1
            else:
                counter_root += 1

            tree.write(self.created_bt_xml, xml_declaration=True)





if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"

    create_bt = CreateBTStructure(application_name = "pick_and_place.xml")
    create_bt.get_bt_template(bt_template_path = None, created_bt_xml_path = None)

    read_task = GetTaskInfo()

    # first write Initialization
    params = [  {"topic": "/arm_parameter_server", \
                "data_type": "double", \
                "service_name": None},
                {"topic": "/gripper_parameter_server", \
                "data_type": "string", \
                "service_name": None},
                {"topic": "/task_flag_parameter_server", \
                "data_type": "bool", \
                "service_name": None}]
    for param_info in params:
        create_bt.initial_subtree(param_info = param_info, created_bt_xml_path = None)

    # start from "job" or "task"
    # add tasks in maintree
    task_spec = read_task.load_task_specification(task_file)
    # task_name: {'task1': 'detect', 'task2': 'pick', 'task3': 'place'}
    task_names = read_task.get_task_name(task_spec)

    create_bt.maintree(task_names = task_names, created_bt_xml_path = None)

    steps = read_task.get_single_step(task_spec)
    old_step=[]
    for step in steps:
        if len(old_step) == 0 or step[0] != old_step[0]:
            create_bt.create_subtree(subtree_name = step[0].capitalize() + task_names[step[0]].capitalize(), 
                                    sequence_name = "steps",
                                    created_bt_xml_path = None)
        create_bt.task_step(task_name = step[0].capitalize() + task_names[step[0]].capitalize(), 
                            step_name = step[0] + step[1],
                            sub_steps = step[2])
        old_step = step

    for task_num in task_names.keys():
        create_bt.add_finish_step(task_name = task_num.capitalize() + task_names[task_num].capitalize(),
                                        task_num= task_num)
    
    for step in steps:
        print(step)
        print(step[0], step[1])
        for item in step[2]:
            print(step[2][item])

    #add subtree
    # Needhelp
    create_bt.need_help_subtree(created_bt_xml_path = None)

    # Execution
    create_bt.execution_subtree(created_bt_xml_path = None, sequence_name = None, plan = "{plan}", step = "{current_step}")
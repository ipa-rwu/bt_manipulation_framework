#! /usr/bin/env python

import os
from bt_structure import CreateBTStructure
from read_task import GetTaskInfo

if __name__ == "__main__":
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"

    create_bt = CreateBTStructure(application_name = "pick_and_place_2.xml")
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
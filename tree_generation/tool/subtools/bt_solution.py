#! /usr/bin/env python

import os
from bt_structure import CreateBTStructure
from read_task import GetTaskInfo

import argparse


def main(task_file, application_name, current_step, bt_template_path = None, created_bt_xml_path = None):
    create_bt = CreateBTStructure(application_name)
    create_bt.get_bt_template(bt_template_path, created_bt_xml_path = None)

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

    create_bt.update_solution_initialization()
    # start from "job" or "task"
    # add tasks in maintree
    task_spec = read_task.load_task_specification(task_file)
    print("task_spec: ",task_spec)
    # task_name: {'task1': 'detect', 'task2': 'pick', 'task3': 'place'}
    task_names = read_task.get_task_name(task_spec)
    print("task_name: ", task_names)

    steps = read_task.get_single_step(task_spec)
    for step in steps:
        create_bt.solution_maintree(sub_steps = step[2], task_names = task_names, retry_times = 3, param = 0.1, step_name = "task2step1", created_bt_xml_path = None)


    # steps = read_task.get_single_step(task_spec)
    # old_step=[]
    # for step in steps:
    #     if len(old_step) == 0 or step[0] != old_step[0]:
    #         create_bt.create_subtree(subtree_name = step[0].capitalize() + task_names[step[0]].capitalize(), 
    #                                 sequence_name = "steps",
    #                                 created_bt_xml_path = None)
    #     create_bt.task_step(task_name = step[0].capitalize() + task_names[step[0]].capitalize(), 
    #                         step_name = step[0] + step[1],
    #                         sub_steps = step[2])
    #     old_step = step

    # for task_num in task_names.keys():
    #     create_bt.add_finish_step(task_name = task_num.capitalize() + task_names[task_num].capitalize(),
    #                                     task_num= task_num)
    
    # for step in steps:
    #     print(step)
    #     print(step[0], step[1])
    #     for item in step[2]:
    #         print(step[2][item])

    #add subtree


if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description='Process some integers.')
    # parser.add_argument('task', metavar='N',
    #                help='an task description file')
    # parser.add_argument('bt', metavar='N',
    #                help='an behavior tree file')
    # args = parser.parse_args()
    # print(args.bt)

    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/solution/description/arm_change_parameter.yaml"
    bt_template_path = BASE_DIR + "/template/bt_solution.xml"

    main(task_file=task_file, application_name="www.xml", current_step = "task2step1", bt_template_path = bt_template_path, created_bt_xml_path = None)
    






    

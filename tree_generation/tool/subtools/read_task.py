#! /usr/bin/env python
import yaml
import os
import numpy as np
class GetTaskInfo():
    def __init__(self, task_file = None):
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if  task_file == None:
            rm_size = len("/tool")
            example_dir = BASE_DIR[:-rm_size] + "/examples"
            self.task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"
        else:
            self.task_file = task_file
    
    def load_task_specification(self, load_file):
        with open(load_file, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
        return data_loaded
    
    # only start with job or task
    def get_task_name(self, data_loaded):
        flag_job = 0
        tasks = []
        task_names = {}
        for first_word, left in data_loaded.items():
            #print(first_word)
            if first_word == "step":
                return None 
            if first_word == "job":
                break
            if first_word == "task":
                task_names["task"] = data_loaded["task"]["name"]
                print(task_names)
                return task_names
        
        # save all tasks 
        for task_name in data_loaded["job"]:
            task = data_loaded["job"][task_name]["name"]
            task_names[task_name] = task
            #tasks.append((task_name,task))
        
        print(task_names)

        return task_names
    
    # def get_task_name(self, tasks):
    #     # task_names = []
        
    #     # for task_tuple in tasks:
    #     #     task_names.append((task_tuple[0],task_tuple[1]["name"]))
    #     # #print(task_names)
    #     # return task_names
        
    #     task_names = {}
        
    #     for task_tuple in tasks:
    #         task_names[task_tuple[0]] = task_tuple[1]["name"]
    #     print("!!!", task_names)
    #     return task_names


    def get_single_step(self, data_loaded):
        flag_job = 0
        flag_task = 0
        flag_step = 0
        tasks = []
        steps = []

        for first_word, left in data_loaded.items():
            #print(first_word)
            if first_word == "job":
                flag_job = 1
            if first_word == "task":
                flag_task = 1
                tasks.append(("task",data_loaded["task"]))
            if first_word == "step":
                steps.append(("task", "step", data_loaded["step"]))
                flag_step = 1
        
        
        # save all tasks 
        if flag_job == 1:
            for task_name in data_loaded["job"]:
                task = data_loaded["job"][task_name]
                tasks.append((task_name,task))
            flag_task = 1
         #print(tasks)
        
        task_names = []

        if flag_task == 1:
            for task_tuple in tasks:
                task_names.append((task_tuple[0],task_tuple[1]["name"]))
                for title in sorted(task_tuple[1]["steps"]):
                    #print(title)
                    steps.append((task_tuple[0], title, task_tuple[1]["steps"][title]))
        #print("????", steps)
        return steps
    
    def get_param(self, steps, param_object):
        param_dic = {}
        for step in steps:
            for key in step[2].keys():
                if step[2][key]["object"] == param_object:
                    param_dic[step[0] + step[1]] = step[2][key]["param"]

        return param_dic
            





if __name__ == "__main__":

    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rm_size = len("/tool")
    example_dir = BASE_DIR[:-rm_size] + "/examples"
    task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"
    
    read_task = GetTaskInfo()

    job_spec = read_task.load_task_specification(task_file)
    steps = read_task.get_single_step(job_spec)


    task_names = read_task.get_task_name(job_spec)

    task_spec = read_task.load_task_specification(
        "/home/rachel/kogrob/manbt_ws/src/manipulator_bt/tree_generation/examples/pick_and_place/application_specification/task.yaml")
    read_task.get_single_step(task_spec)
    task_name = read_task.get_task_name(task_spec)
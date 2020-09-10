#! /usr/bin/env python
import yaml
import os
import numpy as np
class GetSingleStep():
    def __init__(self, task_file = None):
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if  task_file == None:
            rm_size = len("/tool")
            example_dir = BASE_DIR[:-rm_size] + "/examples"
            self.task_file = example_dir + "/pick_and_place/application_specification/job_pickplace.yaml"
    
    def load_task_specification(self):
        with open(self.task_file, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
        return data_loaded

    def get_single_step(self, data_loaded):
        flag_job = 0
        flag_task = 0
        flag_step = 0
        tasks = []
        single_step = []

        for first_word, left in data_loaded.items():
            #print(first_word)
            if first_word == "job":
                flag_job = 1
            if first_word == "task":
                flag_task = 1
                tasks.append(("task1",data_loaded["task"]))
            if first_word == "step":
                single_step.append(("task1", "step1", data_loaded["step"]))
                flag_step = 1
        
        
        # save all tasks 
        if flag_job == 1:
            for task_name in data_loaded["job"]:
                task = data_loaded["job"][task_name]
                tasks.append((task_name,task))
            flag_task = 1
            for first_word, left in data_loaded.items():
                data_loaded = left
                #print(data_loaded)
        #print(tasks)
        
        task_names = []

        if flag_task == 1:
            for task_tuple in tasks:
                task_names.append((task_tuple[0],task_tuple[1]["name"]))
                for title in task_tuple[1]["steps"]:
                    single_step.append((task_tuple[0], title, task_tuple[1]["steps"][title]))
                    #print(single_step)
 
        return single_step


if __name__ == "__main__":
    read_task = GetSingleStep()
    read_task.get_single_step(read_task.load_task_specification())
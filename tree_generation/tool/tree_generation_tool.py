#! /usr/bin/env python
import yaml
import os

class generate_tree():
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    def __init__(self, plugin_file = None):
        if plugin_file == None:
            yaml_path = self.BASE_DIR + "/bt_nodes/man_skill_plugin.yaml"
        else:
            yaml_path = self.BASE_DIR + plugin_file

        with open(yaml_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)

        for plugin in data_loaded["plugin_lib"]:
            print(data_loaded[plugin]["name"])

if __name__ == "__main__":
    generate_tree = generate_tree()

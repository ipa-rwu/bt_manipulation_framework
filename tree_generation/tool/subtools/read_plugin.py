#! /usr/bin/env python
import yaml
import os
import numpy as np

class GetPlugins():
    def __init__(self, plugin_file = None):
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if  plugin_file == None:
            self.plugin_file = BASE_DIR + "/provided_plugin.yaml"
    
    def load_plugin(self):
        with open(self.plugin_file, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
        return data_loaded

    def get_plugin_info(self, data_loaded):
        plugin_names = data_loaded["plugin_lib"]
        #print(plugin_names)
        for plugin_name in plugin_names:
            print(data_loaded[plugin_name])


if __name__ == "__main__":
    get_plugins = GetPlugins()
    get_plugins.get_plugin_info(get_plugins.load_plugin())
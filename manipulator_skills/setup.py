#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

CONFIG = generate_distutils_setup(
    packages=['help_extern_server'],
    package_dir={'': 'src/skills/service_server'}
)

setup(**CONFIG)

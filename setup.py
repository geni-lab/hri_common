#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hri_common', 'hri_common.drivers', 'hri_common.hri_api', 'hri_common.hri_framework'],
    package_dir={'': 'src/'},
)

setup(**d)

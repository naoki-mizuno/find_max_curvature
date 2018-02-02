#!/usr/bin/env python2

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['find_max_curvature'],
    package_dir={'': 'src'},
)

setup(**d)

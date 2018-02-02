cmake_minimum_required(VERSION 2.8.3)
project(find_max_curvature)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
    nodes/find_max_curvature_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

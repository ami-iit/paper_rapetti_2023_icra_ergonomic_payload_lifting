
# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Lorenzo Rapetti <lorenzo.rapetti@iit.it>

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(ergonomy-controllers)

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)

add_subdirectory(app)
add_subdirectory(modules)
add_subdirectory(whole-body-controllers)
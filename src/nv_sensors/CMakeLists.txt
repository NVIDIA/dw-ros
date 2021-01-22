# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 2.8.3)
project(nv_sensors)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
)

add_service_files(DIRECTORY srv FILES
    camera_start.srv
    camera_stop.srv
    )

generate_messages()

catkin_package(
    CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/local/driveworks/targets/aarch64-Linux/include
    /usr/local/cuda-10.2/targets/aarch64-linux/include
    $ENV{HOME}/nvidia/nvidia_sdk/DRIVE_OS_5.2.0.0_SDK_Linux_OS_DDPX/DRIVEOS/drive-t186ref-linux/include
)

add_executable(nv_sensors_producer
    src/camera.cpp
    src/nv_sensors_producer.cpp
)

target_link_libraries(nv_sensors_producer
    ${catkin_LIBRARIES}
    driveworks.so
    rt
)

add_dependencies(nv_sensors_producer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS nv_sensors_producer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

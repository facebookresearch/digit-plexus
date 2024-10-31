
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


cmake_minimum_required(VERSION 3.5)
project(plexus_pkg)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PowerStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

install(PROGRAMS
  plexus_pkg/allegro_info_node.py
  plexus_pkg/allegro_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

# Copyright 2026 amr-sim-lab contributors
# Licensed under the Apache License, Version 2.0. See LICENSE.
cmake_minimum_required(VERSION 3.8)
project(scene_spawner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnLayout.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

#
# Copyright 2022 Xilinx, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

cmake_minimum_required(VERSION 3.8)
project(lidar_and_cam_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vitis_ai_library REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(../common/include/)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(std_msgs)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(OpenCV)
ament_export_dependencies(vitis_ai_library)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${vitis_ai_library_INCLUDE_DIRS}
	../common/include
	../lidar_detection/include
	../cam_detection/include
)

add_library(${PROJECT_NAME}
	../common/src/process_image.cpp
	../common/src/display_transform_matrices.cpp
	../common/src/convert_pointcloud.cpp
	../common/src/process_pointcloud.cpp
	../lidar_detection/src/lidar_detection.cpp
	../cam_detection/src/cam_detection.cpp
)

add_executable(${PROJECT_NAME}_node
	src/${PROJECT_NAME}_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
	${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBRARIES}
	${vitis_ai_library_LIBRARIES}
	vitis_ai_library-pointpillars
	vitis_ai_library-yolov3
	glog
)

target_link_libraries(${PROJECT_NAME}_node
	${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
	sensor_msgs
	cv_bridge
	OpenCV
	vitis_ai_library
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ../common/include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


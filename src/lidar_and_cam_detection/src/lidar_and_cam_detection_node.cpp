/*
 * Copyright 2022 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rclcpp/rclcpp.hpp>
#include "lidar_detection/lidar_detection.hpp"
#include "cam_detection/cam_detection.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;

	auto cam_node =  std::make_shared<cam_detection>();
	auto pointcloud_converter_node = std::make_shared<convert_pointcloud>();
	auto lidar_node = std::make_shared<lidar_detection>();
	executor.add_node(cam_node);
	executor.add_node(pointcloud_converter_node);
	executor.add_node(lidar_node);

	executor.spin();

	rclcpp::shutdown();
	return 0;
}



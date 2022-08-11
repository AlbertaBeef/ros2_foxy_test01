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
 
#include "process_pointcloud.hpp"

namespace vitis
{
	namespace ai
	{
		extern cv::Mat bev_preprocess(const V1F& PointCloud);
	}
}

process_pointcloud::process_pointcloud(const rclcpp::Clock::SharedPtr clk)
{
	_clk = clk;
	_ml_task = vitis::ai::PointPillars::create("pointpillars_kitti_12000_0_pt", "pointpillars_kitti_12000_1_pt");
}

void process_pointcloud::run(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
	_converted_pointcloud = msg->data;
	detect_objects();
	generate_display();
}

void process_pointcloud::detect_objects()
{
	rclcpp::Time begin = _clk->now();
	_result = _ml_task->run(_converted_pointcloud);
	rclcpp::Time end = _clk->now();
	
	_ml_task_fps = 1/((end-begin).seconds());
}

void process_pointcloud::generate_display()
{
	vitis::ai::ANNORET annoret;
	cv::Mat rgbmat;
	
	_bevmat = vitis::ai::bev_preprocess(_converted_pointcloud);
	_ml_task->do_pointpillar_display(_result, vitis::ai::E_BEV, _display_transform_matrices.get_display_data(), rgbmat, _bevmat, 1242, 375, annoret);
}

vitis::ai::PointPillarsResult* process_pointcloud::get_result()
{
	return &_result;
}

cv::Mat process_pointcloud::get_bevmat()
{
	return _bevmat;
}

float process_pointcloud::get_fps()
{
	return _ml_task_fps;
}


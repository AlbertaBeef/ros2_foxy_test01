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

#include "convert_pointcloud.hpp"

using namespace std::placeholders;

convert_pointcloud::convert_pointcloud() : Node("convert_pointcloud")
{
	_sub_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10, std::bind(&convert_pointcloud::recv_pointcloud_callback, this,_1));
	_pub_v1f = this->create_publisher<std_msgs::msg::Float32MultiArray>("/v1f_points", 10);
}

void convert_pointcloud::recv_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// Convert to V1F format V1F is defined in <vitis/ai/pointpillars.hpp> and is only a vector of floats
{
	const float uchar_scale_factor = 1.0/255.0;

	std_msgs::msg::Float32MultiArray _converted_pointcloud;

	// First time we call the function, do stuff only needed once
	static bool first = true;	
	static msg_offsets_t msg_offsets;
	if (first)
	{
		get_msg_offsets(msg, msg_offsets);
		first = false;
	}
	else
	{
		for (unsigned int ii = 0; ii < msg->width*msg->height; ii++)
		{
			float x, y, z, intensity;
			x = *(float*)(&msg->data[0] + (msg->point_step*ii) + msg_offsets.x);
			y = *(float*)(&msg->data[0] + (msg->point_step*ii) + msg_offsets.y);
			z = *(float*)(&msg->data[0] + (msg->point_step*ii) + msg_offsets.z);
			intensity = *(float*)(&msg->data[0] + (msg->point_step*ii) + msg_offsets.intensity);
			
			_converted_pointcloud.data.push_back(x);
			_converted_pointcloud.data.push_back(y);
			_converted_pointcloud.data.push_back(z);
			_converted_pointcloud.data.push_back(intensity*uchar_scale_factor); // HDL64E is different from VLP32C. HDL64E is a float value. VLP32C is int8
		}
	}
	_pub_v1f->publish(_converted_pointcloud);
}

void convert_pointcloud::get_msg_offsets(const sensor_msgs::msg::PointCloud2::SharedPtr msg, msg_offsets_t& msg_offsets)
{
	for (unsigned int ii = 0; ii < msg->fields.size(); ii++)
	{		
		if (msg->fields[ii].name == "x")
		{
			msg_offsets.x = msg->fields[ii].offset;
		}
		if (msg->fields[ii].name == "y")
		{
			msg_offsets.y = msg->fields[ii].offset;
		}
		if (msg->fields[ii].name == "z")
		{
			msg_offsets.z = msg->fields[ii].offset;
		}
		if (msg->fields[ii].name == "intensity" || msg->fields[ii].name == "i") // HDL64E is different from VLP32C
		{
			msg_offsets.intensity = msg->fields[ii].offset;
		}
	}
}

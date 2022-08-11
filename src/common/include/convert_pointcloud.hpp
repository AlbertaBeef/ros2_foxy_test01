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

#ifndef _convert_pointcloud_hpp_
#define _convert_pointcloud_hpp_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

typedef struct msg_offsets
{
	int x;
	int y;
	int z;
	int intensity;
} msg_offsets_t;

class convert_pointcloud : public rclcpp::Node
{
	private:
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_pointcloud;
		rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _pub_v1f;

		void recv_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void get_msg_offsets(const sensor_msgs::msg::PointCloud2::SharedPtr msg, msg_offsets_t& msg_offsets);

	public:
		convert_pointcloud();
		~convert_pointcloud() { }
};

#endif

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

#ifndef _display_fps_hpp_
#define _display_fps_hpp_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

class display_fps : public rclcpp::Node
{
	private:
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_fps;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_fps_marker;

		void recv_fps_callback(const std_msgs::msg::Float32::SharedPtr msg);

	public:
		display_fps();
		~display_fps() { }
};

#endif

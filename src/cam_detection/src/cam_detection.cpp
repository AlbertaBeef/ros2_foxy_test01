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

#include "cam_detection/cam_detection.hpp"

using namespace std::placeholders;

cam_detection::cam_detection() : Node("cam_detection")
{
	_image_processor = new process_image(this->get_clock());
	init_cam_subscriber();
	_pub_output_image = this->create_publisher<sensor_msgs::msg::Image>("/cam_object_img", 10);
	_pub_ml_task_fps = this->create_publisher<std_msgs::msg::Float32>("/cam_fps", 10);
}

void cam_detection::init_cam_subscriber()
{
	std::string cam_name;
	this->declare_parameter(std::string("cam"), rclcpp::ParameterValue("cam"));
	cam_name = this->get_parameter("cam").as_string();
	
	_sub_image = this->create_subscription<sensor_msgs::msg::Image>(std::string("/"+cam_name+"/image_raw"), 10, std::bind(&cam_detection::recv_image_callback, this, _1));
}

void cam_detection::recv_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	_image_processor->run(msg);
	publish_output_image(msg->header, _image_processor->get_output_image());
	publish_fps(_image_processor->get_fps());
}

void cam_detection::publish_output_image(const std_msgs::msg::Header header, cv::Mat img)
{
	sensor_msgs::msg::Image::SharedPtr output_img_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
	_pub_output_image->publish(*output_img_msg.get()); // TODO: Check if this will have performance impact
}

void cam_detection::publish_fps(float processing_fps)
{
	std_msgs::msg::Float32 fps_msg;
	fps_msg.data = processing_fps;
	_pub_ml_task_fps->publish(fps_msg);
}

cam_detection::~cam_detection()
{
	delete _image_processor;
}


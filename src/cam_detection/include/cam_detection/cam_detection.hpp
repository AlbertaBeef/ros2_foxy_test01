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

#ifndef _cam_detection_hpp_
#define _cam_detection_hpp_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "../../common/include/process_image.hpp"

class cam_detection : public rclcpp::Node
{
	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_image;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_output_image;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_ml_task_fps;
		process_image* _image_processor;
		
		void recv_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
		void init_cam_subscriber();
		void publish_output_image(const std_msgs::msg::Header header, cv::Mat img);
		void publish_fps(float processing_fps);

	public:
		cam_detection();
		~cam_detection();
};

#endif

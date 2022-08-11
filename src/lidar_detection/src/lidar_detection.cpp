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

#include "lidar_detection/lidar_detection.hpp"

using namespace std::placeholders;

lidar_detection::lidar_detection() : Node("lidar_detection")
{
	_pointcloud_processor = new process_pointcloud(this->get_clock());
	_sub_pointcloud = this->create_subscription<std_msgs::msg::Float32MultiArray>("/v1f_points", 10, std::bind(&lidar_detection::recv_pointcloud_callback, this, _1));
	_pub_pointcloud_2d_projection_image = this->create_publisher<sensor_msgs::msg::Image>("/lidar_object_img", 10);
	_pub_ml_task_fps = this->create_publisher<std_msgs::msg::Float32>("/lidar_fps", 10);
}

void lidar_detection::recv_pointcloud_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
	std_msgs::msg::Header header;
	header.stamp = this->get_clock()->now();
	header.frame_id = "v1f_pointcloud";
	_pointcloud_processor->run(msg);
	publish_boxes(header, _pointcloud_processor->get_result());
	publish_output_image(header, _pointcloud_processor->get_bevmat());
	publish_fps(_pointcloud_processor->get_fps());
}

void lidar_detection::publish_boxes(const std_msgs::msg::Header header, vitis::ai::PointPillarsResult* result)
{
	/*jsk_recognition_msgs::BoundingBoxArray boxes;
	boxes.header = header;
	for (int ii = 0; ii < result->ppresult.final_box_preds.size(); ii++)
	{
		jsk_recognition_msgs::BoundingBox box;
		box.header = header;		
		box.pose.position.x = result->ppresult.final_box_preds[ii][0];
		box.pose.position.y = result->ppresult.final_box_preds[ii][1];
		box.pose.position.z = result->ppresult.final_box_preds[ii][2];
		box.pose.orientation.w = 1.0;
		box.dimensions.x = result->ppresult.final_box_preds[ii][3];
		box.dimensions.y = result->ppresult.final_box_preds[ii][4];
		box.dimensions.z = result->ppresult.final_box_preds[ii][5];
		box.value = result->ppresult.final_scores[ii];
		box.label = ii;

		boxes.boxes.push_back(box);
	}
	
	_pub_boxes.publish(boxes);*/
}

void lidar_detection::publish_output_image(const std_msgs::msg::Header header, cv::Mat bevmat)
{
	sensor_msgs::msg::Image::SharedPtr bevmat_msg = cv_bridge::CvImage(header, "bgr8", bevmat).toImageMsg();
	_pub_pointcloud_2d_projection_image->publish(*bevmat_msg.get());  // TODO: Check if this will have performance impact
}

void lidar_detection::publish_fps(float processing_fps)
{
	std_msgs::msg::Float32 fps_msg;
	fps_msg.data = processing_fps;
	_pub_ml_task_fps->publish(fps_msg);
}

lidar_detection::~lidar_detection()
{
	delete _pointcloud_processor;
}


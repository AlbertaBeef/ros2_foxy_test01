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

import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetRemap

def generate_launch_description():
    omit_sensor = LaunchConfiguration('omit_sensor')
    arg_omit_sensor = DeclareLaunchArgument("omit_sensor", default_value=TextSubstitution(text="1"))
    
    # Launch Camera node only if omit_sensor is set to 0. If not, play rosbag (.bag) file
    cv_cam_node = GroupAction(
        actions = [
            PushRosNamespace('cam'),
            Node(
                package='cv_camera',
                executable='cv_camera_node',
                name='cam',
                output='screen',
                parameters = [{
                    'image_width': 640,
                    'image_height': 480
                }],
                condition=IfCondition(PythonExpression([omit_sensor, ' == 0']))
            )
        ]
    )
    
    # Static TF transforms
    tf2_ros_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'world']
    )

    # Launch Camera processing node 
    cam_detection_node = Node(
        package='cam_detection',
        executable='cam_detection_node',
        name='cam_detection',
        output='screen'
    )
    
    # Launch ML task FPS Display node
    display_fps_node = GroupAction(
        actions = [
            PushRosNamespace('fps_cam'),
            SetRemap(
                '/fps', '/cam_fps',
            ),
            SetRemap(
                '/fps_marker', '/cam_fps_marker',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('display_fps'),
                        'launch/display_fps.launch.py'
                    )
                )
            )
        ]
    )
    
    return LaunchDescription([
        arg_omit_sensor,
        cv_cam_node,
        tf2_ros_node,
        cam_detection_node,
        display_fps_node
    ])

if __name__ == '__main__':
    generate_launch_description()

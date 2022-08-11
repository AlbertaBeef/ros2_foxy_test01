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
    
    # Launch VLP32C node only if omit_sensor is set to 0. If not, play rosbag (.bag) file
    velodyne_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne'),
                'launch/velodyne-all-nodes-VLP32C-launch.py'
            )
        ),
        condition=IfCondition(PythonExpression([omit_sensor, ' == 0']))
    )

    # Static TF transforms
    tf2_ros_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_velodyne',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'velodyne']
    )
    
    # Launch LiDAR processing node
    lidar_detection_node = Node(
        package='lidar_detection',
        executable='lidar_detection_node',
        output='screen'
    )
    
    # Launch ML task FPS Display node
    display_fps_node = GroupAction(
        actions=[
            PushRosNamespace('fps_lidar'),
            SetRemap(
                '/fps', '/lidar_fps',
            ),
            SetRemap(
                '/fps_marker', '/lidar_fps_marker',
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
        velodyne_node,
        tf2_ros_node,
        lidar_detection_node,
        display_fps_node
    ])


if __name__ == '__main__':
    generate_launch_description()

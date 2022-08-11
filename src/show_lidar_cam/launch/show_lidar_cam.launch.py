import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world',
            parameters=[
                {
                    '/cam/image_width': '640'
                },
                {
                    '/cam/image_height': '480'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_velodyne',
            parameters=[
                {
                    '/cam/image_width': '640'
                },
                {
                    '/cam/image_height': '480'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='cv_camera',
            executable='cv_camera_node',
            name='cam',
            output='screen',
            parameters=[
                {
                    '/cam/image_width': '640'
                },
                {
                    '/cam/image_height': '480'
                }
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'velodyne_pointcloud'), 'launch/VLP-32C_points.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

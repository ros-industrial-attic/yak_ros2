from os import path

from launch import LaunchDescription

from launch_ros import get_default_launch_description
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription([
        launch_ros.actions.Node(
            node_name='yak_ros2_node', package='yak_ros2', node_executable='yak_ros2_node', output='screen',
            parameters=[{
                         'tsdf_frame_id': 'tsdf_origin',
                         }]
            ),
        launch_ros.actions.Node(
            node_name='yak_ros2_image_simulator', package='yak_ros2', node_executable='yak_ros2_image_simulator', output='screen',
            parameters=[{
                         'base_frame': 'world',
                         'orbit_speed': 1.0,
                         'framerate': 30.0,
                         'mesh': path.join(get_package_share_directory('yak_ros2'), 'demo', 'bun_on_table.ply'),
                         }]
            ),
        launch_ros.actions.Node(
            node_name='static_tf_publisher', package='tf2_ros', node_executable='static_transform_publisher', output='screen',
            arguments=['-0.3', '-0.3', '-0.01', '0', '0', '0', '1', 'world', 'tsdf_origin'],
            ),
    ])
    return ld

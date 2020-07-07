from os import path

from launch import LaunchDescription

from launch_ros import get_default_launch_description
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription([
        launch_ros.actions.Node(
            node_name='yak_ros2_node', package='yak_ros2', node_executable='yak_ros2_node', output='screen',
            remappings=[('input_depth_image', '/image')],
            parameters=[{
                         'tsdf_frame_id': 'tsdf_origin',
                         'camera_intrinsic_params':
                           {
                             'fx': 550.0,
                             'fy': 550.0,
                             'cx': 320.0,
                             'cy': 240.0,
                           },
                         'cols': 640,
                         'rows': 480,
                         'volume_resolution': 0.001,
                         'volume_x': 640,
                         'volume_y': 640,
                         'volume_z': 192,
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

from os import path

from launch import LaunchDescription

from launch_ros import get_default_launch_description
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription([
        launch_ros.actions.Node(
            node_name='yak_ros_node', package='yak_ros', node_executable='yak_ros_node', output='screen',
            parameters=[{'tsdf_frame_id': 'tsdf_origin'}]
            ),
        launch_ros.actions.Node(
            node_name='mutable_tf_pub', package='mutable_transform_publisher', node_executable='mutable_transform_pub', output='screen',
            parameters=[{'yaml_path': path.join(get_package_share_directory('yak_ros'), 'config', 'mutable_tf_config.yaml')}]
            ),
    ])
    return ld

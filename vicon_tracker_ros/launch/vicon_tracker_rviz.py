import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_directory('vicon_tracker_ros'),'rviz','vicon_display.rviz')

    node_vicon_client = Node(
        package = 'vicon_tracker_ros',
        name = 'vicon_tracker_client',
        executable = 'vicon_tracker_client',
        parameters = [os.path.join(
            get_package_share_directory('vicon_tracker_ros'),
            'config',
            'vicon_tracker_client_params.yaml'
        )]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_path)],
    )

    # Run the nodes
    return LaunchDescription([
        node_vicon_client,
        rviz,
    ])


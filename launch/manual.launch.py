import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('hybrid_astar')
    rviz_config_dir = os.path.join(pkg_dir, 'launch', 'config.rviz')
    map_yaml_file = os.path.join(pkg_dir, 'maps', 'map.yaml')

    return LaunchDescription([
        Node(
            package='hybrid_astar',
            executable='hybrid_astar',
            name='hybrid_astar',
            output='screen'
        ),
        Node(
            package='hybrid_astar',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])

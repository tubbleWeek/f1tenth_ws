from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    costmap_config = os.path.join(
        get_package_share_directory('f1tenth_controllers'),
        'config',
        'costmap_params.yaml'
    )
    costmap_la = DeclareLaunchArgument(
        'costmap_config',
        default_value=costmap_config,
        description='Descriptions for costmap configs')
    
    ld = LaunchDescription([costmap_la])

    global_costmap = Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            parameters=[LaunchConfiguration('costmap_config')],
            output='screen'
            )
    
    local_costmap = Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            parameters=[LaunchConfiguration('costmap_config')],
            output='screen'
            )
    
    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['global_costmap', 'local_costmap']
            }]
            )
    
    ld.add_action(global_costmap)
    ld.add_action(local_costmap)
    ld.add_action(lifecycle_manager)

    return ld

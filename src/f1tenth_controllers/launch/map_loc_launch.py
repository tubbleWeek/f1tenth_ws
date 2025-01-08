from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    map_server_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'map_server_params.yaml'
    )
    amcl_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'amcl_params.yaml'
    )

    costmap_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'costmap_params.yaml'
    )

    wp_visual_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'waypoint_visual_config.yaml'
    )

    map_server_la = DeclareLaunchArgument(
        'map_server_config',
        default_value=map_server_config,
        description='Descriptions for map server configs')
    amcl_la = DeclareLaunchArgument(
        'amcl_config',
        default_value=amcl_config,
        description='Descriptions for amcl configs')

    costmap_la = DeclareLaunchArgument(
        'costmap_config',
        default_value=costmap_config,
        description='Descriptions for costmap configs')

    wp_visual_la = DeclareLaunchArgument(
        'wp_visual_config',
        default_value=wp_visual_config,
        description='Descriptions for waypoint visual configs')

    ld = LaunchDescription([map_server_la, amcl_la, costmap_la, wp_visual_la])

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('map_server_config')]
    )

    map_server_lifecyle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('amcl_config')]
    )

    amcl_lifecyle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['amcl']
        }]
    )

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='nav2_costmap_2d',
        output='screen',
        parameters=[LaunchConfiguration('costmap_config')]
    )

    costmap_lifecyle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['nav2_costmap_2d']
        }]
    )

    wp_vis_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[LaunchConfiguration('wp_visual_config')]
    )


    map_lc_ta = TimerAction(
        period = 1.0,
        actions=[map_server_lifecyle]
    )

    amcl_n_ta = TimerAction(
        period = 2.0,
        actions=[amcl_node]
    )

    amcl_lc_ta = TimerAction(
        period = 3.0,
        actions=[amcl_lifecyle]
    )

    wp_visual_ta = TimerAction(
        period = 3.5,
        actions=[wp_vis_node]
    )

    costmap_n_ta = TimerAction(
        period = 4.0,
        actions=[costmap_node]
    )

    costmap_lc_ta = TimerAction(
        period = 4.5,
        actions=[costmap_lifecyle]
    )
    # finalize
    ld.add_action(map_server_node)
    ld.add_action(wp_vis_node)
    ld.add_action(map_lc_ta)
    # ld.add_action(wp_visual_ta)
    ld.add_action(costmap_n_ta)
    ld.add_action(costmap_lc_ta)
    ld.add_action(amcl_n_ta)
    ld.add_action(amcl_lc_ta)

    return ld
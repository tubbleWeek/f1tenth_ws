import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the urdf/xacro file path
    path_to_urdf = '/home/jiang/Desktop/f1tenth_ws/src/f1tenth_controllers/launch/models/f1tenth_gazebo/f1tenth_robot.urdf'
    with open(path_to_urdf, 'r') as infp:
        robot_desc = infp.read()
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }]
    )

    # Use your custom Gazebo world (optional). Replace 'empty.sdf' with the word "world" (no "" needed)
    # world = os.path.join(
    #     get_package_share_directory(package_name), "worlds", "empty_world.sdf"
    # )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", 'empty.sdf']}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "robot1",
            "-topic",
            "/robot_description",
            "-robot_namespace",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "1.4",
        ],
        output="screen",
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher, gz_sim, spawn_entity
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def evaluate_joystick(context, *args, **kwargs):
    joystick_value = LaunchConfiguration('joystick').perform(context)
    
    if joystick_value == 'True':
        # Launch the joystick teleop node for manual control
        teleop_node = Node(
            package='turtlebot3_teleop_joystick',
            executable='teleop_joystick',
            name='teleop_joystick'
        )
        # Launch the joystick driver node
        joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.1,  # Deadzone for joystick, adjust as needed
            }]
        )
        return [teleop_node, joy_node]
    else:
        return []

def generate_launch_description():   
    # Declare arguments
    joystick_launch_arg = DeclareLaunchArgument(
        'joystick',
        default_value='False',
        description='Specify whether or not Joystick will be used for teleop'
    )

    # Get the path to your package's share directory
    package_share_dir = get_package_share_directory('bezobots')  # Replace with your package name
    
    # Declare the launch argument for selecting the turtlebot3_gazebo launch file
    turtlebot_launch_arg = DeclareLaunchArgument(
        'turtlebot_launch_file', 
        default_value='aws_world.launch.py',  # Default to this launch file
        description='Specify the TurtleBot3 Gazebo launch file to use'
    )

    # Launch the turtlebot3_gazebo with the selected launch file
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir, 'launch/'),
            LaunchConfiguration('turtlebot_launch_file')])
    )
    
    workspace_dir = os.getenv('COLCON_PREFIX_PATH', os.getcwd()).replace('/install','')
    map_file_path = os.path.join(workspace_dir, 'src', '41068_SPR2024_BezoBots', 'BezoBots_Main', 'maps', 'gazebo_map', 'map.yaml')

    # Declare the map file path argument
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=map_file_path,
        description='Full path to map file to load'
    )

    # Timer to delay the map server initialization after RViz2
    map_server_cmd = TimerAction(
        period=5.0,  # Delay for 5 seconds (adjust this as needed based on your system performance)
        actions=[Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map_yaml_file'),  # Use the launch argument for map file
                         'frame_id': 'odom'}],
            remappings=[('/map', '/gazebomap')]
        )]
    )

    # Delay lifecycle manager to give the map server time to start
    lifecycle_manager_cmd = TimerAction(
        period=10.0,  # Delay for 10 seconds (adjust as needed, depending on how long the map_server takes)
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Adjust for simulation time if necessary
                'autostart': True,  # Automatically configure and activate lifecycle nodes
                'node_names': ['map_server']
            }]
        )]
    )
    
    rviz_config_dir = os.path.join(
        package_share_dir,
        'config',
        'rviz_config_SLO_3_2.rviz'
    )
    
    cartographer_launch_arg = DeclareLaunchArgument(
        'cartographer_launch_file', 
        default_value='cartographer.launch.py',  # Default to this launch file
        description='Specify the launch file that starts cartographer'
    )
    
    # Include the Cartographer launch file
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_share_dir, 'launch/'),
            LaunchConfiguration('cartographer_launch_file')])
    )
    
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    return LaunchDescription([
        turtlebot_launch_arg,  # Include the argument for selecting the TurtleBot3 Gazebo launch file
        joystick_launch_arg,  # Joystick argument
        map_yaml_arg,  # Pass the map YAML argument
        turtlebot3_gazebo_launch,
        OpaqueFunction(function=evaluate_joystick),  # Evaluate joystick configuration
        rviz_launch,  # Launch RViz first
        map_server_cmd,  # Delayed map server launch
        lifecycle_manager_cmd,  # Delayed lifecycle manager launch
        cartographer_launch_arg,
        cartographer_launch,  # Pass the cartographer launch argument
    ])

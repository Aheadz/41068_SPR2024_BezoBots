import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_nav2_cmd(context, *args, **kwargs):
    # Perform the substitution for the map_dir argument within the context of the launch
    map_dir = LaunchConfiguration('map_dir').perform(context)
    
    # Now use the resolved map_dir in the command
    nav2_bringup_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', f'map:={map_dir}', 'use_sim_time:=True'],
        output='screen'
    )
    
    return [nav2_bringup_cmd]

def evaluate_joystick(context, *args, **kwargs):
    joystick_value = LaunchConfiguration('joystick').perform(context)
    
    # Ensure the LaunchConfiguration is evaluated here
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
    # Declare the launch argument for selecting the turtlebot3_gazebo launch file
    turtlebot_launch_arg = DeclareLaunchArgument(
        'turtlebot_launch_file', 
        default_value='turtlebot3_world.launch.py',  # Default to this launch file
        description='Specify the TurtleBot3 Gazebo launch file to use'
    )
    
    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value='/home/strangelove/robo1_ws/src/robo_S2/maps/map2/map2.yaml', 
        description='Specify the path of the map to be used'
    )
    
    joystick_launch_arg = DeclareLaunchArgument(
        'joystick',
        default_value='False',
        description='Specify whether or not Joystick will be used for teleop'
    )

   # Launch the turtlebot3_gazebo with the selected launch file
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch/'),
            LaunchConfiguration('turtlebot_launch_file')])
    )

    rqt_plot_accel = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        arguments=['topics','/imu/linear_acceleration/x','/imu/linear_acceleration/y']
    )
    
    rqt_plot_odom = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        arguments=['topics','/odom/twist/twist/linear/x','/odom/twist/twist/linear/y']
    )
      
    return LaunchDescription([
        turtlebot_launch_arg,
        map_dir_arg,  # Map directory argument
        joystick_launch_arg,
        turtlebot3_gazebo_launch,
        OpaqueFunction(function=evaluate_joystick),  # Evaluate joystick configuration
        OpaqueFunction(function=generate_nav2_cmd),  # Execute nav2_bringup with map_dir
        rqt_plot_accel,
        rqt_plot_odom,
    ])

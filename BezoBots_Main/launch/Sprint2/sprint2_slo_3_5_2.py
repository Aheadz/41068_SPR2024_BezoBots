import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument for selecting the turtlebot3_gazebo launch file
    turtlebot_launch_arg = DeclareLaunchArgument(
        'turtlebot_launch_file', 
        default_value='turtlebot3_world.launch.py',  # Default to this launch file
        description='Specify the TurtleBot3 Gazebo launch file to use'
    )
    
    joystick_launch_arg = DeclareLaunchArgument(
        'joystick',
        default_value='False',
        description='Specify whether or not Joystick will be used for teleop'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable sim time'
    )

    # Get the path to your package's share directory
    package_share_dir = get_package_share_directory('robo_s2')  # Replace with your package name

    # Launch the turtlebot3_gazebo with the selected launch file
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch/'),
            LaunchConfiguration('turtlebot_launch_file')])
    )
    
    turtlebot3_rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch/'),'rviz2.launch.py'])
    )
    
   
    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=['/home/strangelove/robo1_ws/src/robo_S2/config/ekf.yaml', {'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
      
    if LaunchConfiguration('joystick') == 'True':
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
        return LaunchDescription([
            turtlebot_launch_arg,  # Include the argument for selecting the TurtleBot3 Gazebo launch file
            turtlebot3_gazebo_launch,
            turtlebot3_rviz2_launch,
            teleop_node,
            robot_localization_node,
            joy_node,
            #rqt_plot_accel,
            #rqt_plot_odom,
        ])
    else:
        return LaunchDescription([
            turtlebot_launch_arg,
            turtlebot3_gazebo_launch,
            turtlebot3_rviz2_launch,
            robot_localization_node,
            #rqt_plot_accel,
            #rqt_plot_odom,
        ])
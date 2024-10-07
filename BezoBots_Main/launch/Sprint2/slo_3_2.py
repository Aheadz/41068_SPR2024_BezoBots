import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    
    joystick_launch_arg = DeclareLaunchArgument(
        'joystick',
        default_value='False',
        description='Specify whether or not Joystick will be used for teleop'
    )

    # Get the path to your package's share directory
    package_share_dir = get_package_share_directory('robo_s2')  # Replace with your package name

    # Ensure the map directory exists
    map_directory = os.path.join(package_share_dir, 'maps')
    if not os.path.exists(map_directory):
        os.makedirs(map_directory)

    # Launch the turtlebot3_gazebo with the selected launch file
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch/'),
            LaunchConfiguration('turtlebot_launch_file')])
    )
    # Execute the nav2_bringup launch command
    nav2_bringup_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=True'],
        output='screen'
    )

    # Launch the SLAM toolbox with simulation time enabled
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'),
            '/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # Launch RViz2 with a pre-configured view
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
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
        turtlebot_launch_arg,  # Include the argument for selecting the TurtleBot3 Gazebo launch file
        joystick_launch_arg,  # Joystick argument
        turtlebot3_gazebo_launch,
        OpaqueFunction(function=evaluate_joystick),  # Evaluate joystick configuration
        rqt_plot_accel,
        rqt_plot_odom,
        nav2_bringup_cmd,  # Execute nav2_bringup command
        slam_toolbox_launch,
        rviz_launch,
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Get the path to the 'bezobots' package's share directory
package_share_dir = get_package_share_directory('bezobots')

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')

# Function to evaluate joystick usage
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
                'deadzone': 0.1,  # Adjust deadzone for joystick
            }]
        )
        return [teleop_node, joy_node]
    else:
        return []

# Generate the unified launch description
def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    joystick_launch_arg = DeclareLaunchArgument(
        'joystick',
        default_value='False',
        description='Specify whether or not Joystick will be used for teleop'
    )
    turtlebot_launch_arg = DeclareLaunchArgument(
        'turtlebot_launch_file',
        default_value='aws_world.launch.py',
        description='Specify the TurtleBot3 Gazebo launch file to use'
    )
    
    workspace_dir = os.getenv('COLCON_PREFIX_PATH', os.getcwd()).replace('/install','')
    map_file_path = '/home/student/BezoBots_ws/src/41068_SPR2024_BezoBots/BezoBots_Main/maps/gazebo_map/map.yaml'
    
    map_dir = LaunchConfiguration(
        'map',
        default=map_file_path
    )
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(package_share_dir, 'config', 'nav2_params.yaml')
    )

    # Define paths for other files
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    rviz_nav2_config_dir = os.path.join(package_share_dir, 'config', 'nav2_default_view.rviz')

    # Include the TurtleBot3 Gazebo launch file
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_share_dir, 'launch/'),
            LaunchConfiguration('turtlebot_launch_file')
        ])
    )

    # Include the navigation2 launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir
        }.items(),
    )

    # RViz node for nav2 package
    rviz_nav2_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_nav2_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Custom ROS 2 node from Sprint4
    sprint4_node = ExecuteProcess(
        cmd=['ros2', 'run', 'sprint4_new', 'sprint4_new_node'],
        output='screen',
        cwd='/home/student/BezoBots_ws/src/41068_SPR2024_BezoBots/sprint4_new'  # Ensure it runs in the correct directory
    )

    # Return the full launch description
    return LaunchDescription([
        joystick_launch_arg,
        turtlebot_launch_arg,
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        turtlebot3_gazebo_launch,
        nav2_bringup_launch,
        OpaqueFunction(function=evaluate_joystick),
        rviz_nav2_launch,
        sprint4_node  # Add Sprint4 custom node at the end
    ])

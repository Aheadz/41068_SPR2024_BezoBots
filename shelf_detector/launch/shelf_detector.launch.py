from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the base map path
    map_dir = '/home/student/ros2_ws/src/41068_SPR2024_BezoBots/BezoBots_Main/maps/gazebo_map'
    
    return LaunchDescription([
        Node(
            package='shelf_detector',
            executable='shelf_detector',
            name='shelf_detector',
            parameters=[{
                'map_yaml_path': os.path.join(map_dir, 'map.yaml'),
                'binary_map_path': os.path.join(map_dir, 'map_binary.pgm')
            }]
        )
    ])
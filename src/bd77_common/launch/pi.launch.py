from launch import LaunchDescription
from launch.actions import Shutdown,IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():



    return LaunchDescription([
        Node(
            package='bd77_control',
            executable='sound_control',
            name='sound_control_node',
            output='screen'
        ),
        Node(
            package='bd77_control',
            executable='eye_control',
            name='eye_control_node',
            output='screen'
        ),
        Node(
            package='bd77_control',
            executable='ear_control',
            name='ear_control_node',
            output='screen'
        ),
        Node(
            package='bd77_control',
            executable='steamdeck_input',
            name='steamdeck_input_node',
            output='screen'
        ),
        Node(
            package='bd77_control',
            executable='head_movement_control',
            name='head_movement_control_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('foxglove_bridge'), 'launch/foxglove_bridge_launch.xml')
        )),
        # Add more nodes as needed
    ])

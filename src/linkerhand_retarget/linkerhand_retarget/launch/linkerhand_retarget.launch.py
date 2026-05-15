from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration',
            default_value='True',
            description='Enable Calibration'
        ),
        
        Node(
            package='linkerhand_retarget',
            executable='handretarget',
            name='handretarget',
            output='screen',
            parameters=[{
                'calibration': LaunchConfiguration('calibration'),
            }]
        ),
    ])

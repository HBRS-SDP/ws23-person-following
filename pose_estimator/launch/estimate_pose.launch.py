#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
    	[
        Node(
            package='pose_estimator',  # Package name
            executable='estimate_pose',  # Python script name
            name='PoseEstimationNode',
            output='screen',
            emulate_tty=True,
            remappings=[('/camera/image_raw', '/your/image_topic')]  # Update topic as needed
        )
    	]
    )

#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
    	[
        Node(
            package='pose_estimator',  # Package name
            executable='milestone_done',  # Python script name
            name='PoseEstimationNode',
            output='screen',
            emulate_tty=True
        )
    	]
    )

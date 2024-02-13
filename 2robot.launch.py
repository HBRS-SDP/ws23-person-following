#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    robotname = os.environ['ROBOT_NAME']

    smart_wheel_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("kelo_tulip"), 'launch'),
            '/example_joypad.launch.py'])
    )

    if robotname=='robile3_with_sick':
        print('[INFO] [launch] robile3: loading Sick-LMS1XX')
        laser_scanner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sick_scan'), 'launch'),
            '/sick_lms_1xx.launch.py']),
    )  
    elif robotname=='robile3':
        print('[INFO] [launch] robile4: loading Hukoyu-URG04LX-UG01')
        laser_scanner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('urg_node2'), 'launch'),
            '/urg_node2.launch.py']),
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "gazebo",
                    "gazebo_robile_laserscanner_camera.xacro"
                ]
            ),
            " ",
            "platform_config:=4_wheel_config",
            " ",
            "movable_joints:=False",
        ]
    )

    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description_content, value_type=str)
            }],            
    )  

    rviz_cmd = Node(package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    output='screen'
    )
    
    static_transform = Node(package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                arguments=["0", "0", "0", "0", "0",
                                           "0", "base_link", "base_footprint"]
    )

    static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_link_node',
        output='screen',
        arguments=["0.45", "0", "0.22", "0", "0",
                    "0", "base_link", "base_laser"]
    )                                
    
    rs_lidar3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("rslidar_sdk"), 'launch'),
            '/start.py'])
    )

    # realsense = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory("realsense2_camera"), 'launch'),
    #         '/rs_launch.py'])
    # )    

    depth_profile = DeclareLaunchArgument(
    'depth_profile',
    default_value=['1280', '720', '30'],
    description='Depth profile as "width height fps" (e.g., 1280 720 30)'
    )

    pointcloud_enable = DeclareLaunchArgument(
        'pointcloud_enable',
        default_value='true',
        description='Enable point cloud generation'
    )

    # ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true

    realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    realsense2_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense2_camera_dir, '/launch/rs_launch.py']),
            launch_arguments={'align_depth': 'true', 'depth_width':'1280', 'depth_height': '720',
                              'depth_fps':'10.0', 'color_width':'1280', 'color_height':'720', 'color_fps':'15.0', 
                              'pointcloud.enable':'true'}.items()
        )    

    static_transform_rslidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rslidar_link_node',
        output='screen',
        arguments=["0.18", "0.07", "0.8", "0", "0",
                    "0", "base_link", "rslidar"]
    )  

    static_transform_realsense = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_camera_link_node',
        output='screen',
        arguments=["0.22", "0.15", "0.65", "0", "0",
                    "0", "base_link", "camera_link"]
    )          


# ros2 launch rslidar_sdk start.py


    return LaunchDescription([
        laser_scanner,
        smart_wheel_driver,
        static_transform_cmd,
        joint_state_publisher,
        robot_state_publisher,
        rviz_cmd,
        static_transform,
        # rs_lidar3d,
        # static_transform_rslidar,
        static_transform_realsense,
        realsense2_camera
        # depth_profile,
        # pointcloud_enable,
    ])

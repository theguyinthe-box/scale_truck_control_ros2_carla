#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    ros_param_file = os.path.join(
        get_package_share_directory('scale_truck_control_ros2'),
        'config',
        'config.yaml')

    lane_param_file = os.path.join(
        get_package_share_directory('scale_truck_control_ros2'),
        'config',
        'FV1.yaml')

    # Node #
    # camera and lidar topics remapped from Carla ROS2 Bridge

    ##    laserfilter_node=Node(
    ##            package="laser_filters",
    ##            namespace='LV',
    ##            executable="scan_to_scan_filter_chain",
    ##            parameters=[
    ##              PathJoinSubstitution([
    ##                get_package_share_directory("laser_filters"),
    ##                "examples", "laserfilter_angle.yaml",
    ##                ])],
    ##            output='screen',)

    object_node=Node(
        package="object_detection_ros2",
        namespace='LV',
        executable="object_detection_ros2_node",
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        })

    lane_detection_node=Node(
        package='lane_detection_ros2',
        namespace='LV',
        name='LaneDetector', # .yaml에 명시.
        executable='lane_detect_node',
        output='screen',
        parameters = [lane_param_file])

    control_node=Node(
        package='scale_truck_control_ros2',
        namespace='LV',
        name='scale_truck_control_node',
        executable='control_node',
        output='screen',
        parameters = [ros_param_file])

    lrc_node=Node(
        package='scale_truck_control_ros2',
        namespace='LV',
        name='LRC',
        executable='lrc_node',
        parameters = [ros_param_file],
        output='screen')

    opencr_node=Node(
        package='scale_truck_control_ros2',
        name='carla_control',
        namespace='LV',
        executable='carla_control_node',
    )

    ld = LaunchDescription()

    #ld.add_action(usb_cam_node)
    ld.add_action(lane_detection_node)
    #ld.add_action(rplidarA3_node)
    #ld.add_action(laserfilter_node)
    ld.add_action(object_node)
    ld.add_action(control_node)
    ld.add_action(lrc_node)
    ld.add_action(opencr_node)

    return ld

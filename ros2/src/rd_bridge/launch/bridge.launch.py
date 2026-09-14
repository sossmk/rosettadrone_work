#!/usr/bin/env python3
"""브리지 두 노드를 한 번에 띄운다.

    ros2 launch rd_bridge bridge.launch.py                       # 실시간 영상 + cmd_vel
    ros2 launch rd_bridge bridge.launch.py video_source:=~/drone_ws/capture/mavic2-real-20260910.h264
    ros2 launch rd_bridge bridge.launch.py cmd_vel:=false        # 영상만 (안전)

⚠ cmd_vel 노드는 /cmd_vel 이 오기 전에는 한 발도 쏘지 않으므로 띄워 두는 것 자체는
  안전하다. 그래도 실기에서 영상만 볼 때는 cmd_vel:=false 로 빼 두는 편이 낫다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_source = LaunchConfiguration("video_source")
    video_port = LaunchConfiguration("video_port")
    mav_port = LaunchConfiguration("mav_port")
    cmd_vel = LaunchConfiguration("cmd_vel")

    return LaunchDescription([
        DeclareLaunchArgument("video_source", default_value="udp",
                              description="udp 또는 .h264 파일 경로"),
        DeclareLaunchArgument("video_port", default_value="5600"),
        DeclareLaunchArgument("mav_port", default_value="14550"),
        DeclareLaunchArgument("cmd_vel", default_value="true"),

        Node(package="rd_bridge", executable="video_node", name="rd_video",
             output="screen",
             parameters=[{"source": video_source, "port": video_port}]),

        Node(package="rd_bridge", executable="cmd_vel_node", name="rd_cmd_vel",
             output="screen", condition=IfCondition(cmd_vel),
             parameters=[{"port": mav_port}]),
    ])

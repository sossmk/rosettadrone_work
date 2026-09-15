#!/usr/bin/env python3
"""브리지를 한 번에 띄운다.

    ros2 launch rd_bridge bridge.launch.py                  # 영상 + 링크 + 항법
    ros2 launch rd_bridge bridge.launch.py nav:=false       # 항법 빼고 (수동 cmd_vel 만)
    ros2 launch rd_bridge bridge.launch.py link:=false      # 영상만 (기체를 안 건드린다)
    ros2 launch rd_bridge bridge.launch.py tag:=true link:=false nav:=false   # 영상 + AprilTag

노드 셋:
    video_node  영상 → /image_raw
    link_node   UDP 14550 독점 — 다운링크 → /rd/*, /cmd_vel → MAVLink #84
    nav_node    /nav/goal → 유도 루프(우리 것) → /cmd_vel

⚠ link_node 는 /cmd_vel 이 오기 전에는 한 발도 쏘지 않고, nav_node 는 /nav/goal 이
  오기 전에는 /cmd_vel 을 내지 않는다. 그래서 띄워 두는 것 자체는 기체를 건드리지 않는다.
⚠⚠ UDP 14550 은 link_node 가 독점한다 — QGC·decode_check.py 와 동시에 못 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_source = LaunchConfiguration("video_source")
    video_port = LaunchConfiguration("video_port")
    mav_port = LaunchConfiguration("mav_port")
    link = LaunchConfiguration("link")
    nav = LaunchConfiguration("nav")
    tag = LaunchConfiguration("tag")

    return LaunchDescription([
        DeclareLaunchArgument("video_source", default_value="udp",
                              description="udp 또는 .h264 파일 경로"),
        DeclareLaunchArgument("video_port", default_value="5600"),
        DeclareLaunchArgument("mav_port", default_value="14550"),
        DeclareLaunchArgument("link", default_value="true"),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("tag", default_value="false",
                              description="apriltag_ros 를 같이 띄운다 (apt 설치 필요)"),

        Node(package="rd_bridge", executable="video_node", name="rd_video",
             output="screen",
             parameters=[{"source": video_source, "port": video_port}]),

        Node(package="rd_bridge", executable="link_node", name="rd_link",
             output="screen", condition=IfCondition(link),
             parameters=[{"port": mav_port}]),

        Node(package="rd_bridge", executable="nav_node", name="rd_nav",
             output="screen", condition=IfCondition(nav)),

        # AprilTag → /tf (camera → tag36h11:<id>) · /detections
        Node(package="apriltag_ros", executable="apriltag_node", name="apriltag",
             output="screen", condition=IfCondition(tag),
             remappings=[("image_rect", "/image_raw"), ("camera_info", "/camera_info")],
             parameters=[os.path.join(get_package_share_directory("rd_bridge"),
                                      "config", "tags.yaml")]),
    ])

#!/usr/bin/env python3
"""브리지를 한 번에 띄운다.

    ros2 launch rd_bridge bridge.launch.py                  # 영상 + 링크 + 항법 (경유점)
    ros2 launch rd_bridge bridge.launch.py nav:=false       # 항법 빼고 (drive 등 수동)
    ros2 launch rd_bridge bridge.launch.py link:=false      # 기체를 안 건드린다
    ros2 launch rd_bridge bridge.launch.py tag:=true follow:=true nav:=false \
        camera_info_file:=$HOME/rosettadrone_work/ros2/src/rd_bridge/config/camera.yaml   # 태그 추종
    ros2 launch rd_bridge bridge.launch.py usv:=true nav:=false   # USV GPS 추종

    video_node       영상 → /image_raw · /camera_info
    link_node        UDP 14550 — 다운링크 → /rd/* · /cmd_vel → #84
    nav_node         경유점 → /cmd_vel                     (nav, 기본 켬)
    apriltag_node    영상 → /detections · /tf (camera→tag0)  (tag)
    tag_follow_node  /tf → /cmd_vel                        (follow)
    usv_follow_node  /usv/* → /cmd_vel · 짐벌 틸트           (usv)

띄우기만 해서는 기체가 안 움직인다 — 항법·추종은 목표나 /tag/follow · /usv/follow 를 받아야 명령을 낸다.
⚠ nav · follow · usv 중 하나만 켠다 (셋 다 /cmd_vel).
⚠ 14550 은 link_node 가 독점한다 — QGC·gps_capture.py 와 같이 못 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    video_source = LaunchConfiguration("video_source")
    video_port = LaunchConfiguration("video_port")
    mav_port = LaunchConfiguration("mav_port")
    link = LaunchConfiguration("link")
    nav = LaunchConfiguration("nav")
    tag = LaunchConfiguration("tag")
    follow = LaunchConfiguration("follow")
    usv = LaunchConfiguration("usv")
    cam_info = LaunchConfiguration("camera_info_file")
    rectify = LaunchConfiguration("rectify")

    return LaunchDescription([
        DeclareLaunchArgument("video_source", default_value="udp",
                              description="udp 또는 .h264 파일 경로"),
        DeclareLaunchArgument("video_port", default_value="5600"),
        DeclareLaunchArgument("mav_port", default_value="14550"),
        DeclareLaunchArgument("link", default_value="true"),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("tag", default_value="false",
                              description="apriltag_ros 를 같이 띄운다 (apt 설치 필요)"),
        DeclareLaunchArgument("follow", default_value="false",
                              description="태그 추종 노드. nav 와 같이 쓰지 말 것 (둘 다 /cmd_vel)"),
        DeclareLaunchArgument("usv", default_value="false",
                              description="USV GPS 추종 노드. nav · follow 와 같이 쓰지 말 것"),
        DeclareLaunchArgument("camera_info_file", default_value="",
                              description="보정 결과 YAML. 비우면 대략값(fx=fy=1250)을 쓴다"),
        DeclareLaunchArgument("rectify", default_value="false",
                              description="왜곡을 펴서 발행한다 (camera_info_file 이 있어야 뜻이 있다)"),

        Node(package="rd_bridge", executable="video_node", name="rd_video",
             output="screen",
             # 형을 못 박는다 — 빈 문자열이 None 으로, "false" 가 문자열로 넘어가면
             # 노드가 InvalidParameterTypeException 으로 죽는다 (tags.yaml refine 과 같은 함정)
             parameters=[{"source": video_source, "port": video_port,
                          "camera_info_file": ParameterValue(cam_info, value_type=str),
                          "rectify": ParameterValue(rectify, value_type=bool)}]),

        Node(package="rd_bridge", executable="link_node", name="rd_link",
             output="screen", condition=IfCondition(link),
             parameters=[{"port": mav_port}]),

        Node(package="rd_bridge", executable="nav_node", name="rd_nav",
             output="screen", condition=IfCondition(nav)),

        # AprilTag → /tf (camera → tag0) · /detections
        # ⚠ image_rect 라는 이름이지만 /image_raw 를 물린다. 왜곡 보정까지 하려면
        #    video_node 에 camera_info_file 과 rectify:=true 를 준다 (거기서 같이 편다)
        Node(package="apriltag_ros", executable="apriltag_node", name="apriltag",
             output="screen", condition=IfCondition(tag),
             remappings=[("image_rect", "/image_raw"), ("camera_info", "/camera_info")],
             parameters=[os.path.join(get_package_share_directory("rd_bridge"),
                                      "config", "tags.yaml")]),

        # 태그 추종 — 뜨자마자 움직이지 않는다. /tag/follow true 를 받아야 명령을 낸다
        Node(package="rd_bridge", executable="tag_follow_node", name="rd_tag_follow",
             output="screen", condition=IfCondition(follow)),

        # USV 추종 — /usv/follow true 를 받아야 /cmd_vel · 짐벌 명령을 낸다
        Node(package="rd_bridge", executable="usv_follow_node", name="rd_usv_follow",
             output="screen", condition=IfCondition(usv)),
    ])

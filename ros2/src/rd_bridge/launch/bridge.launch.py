#!/usr/bin/env python3
"""브리지를 한 번에 띄운다.

    ros2 launch rd_bridge bridge.launch.py                  # 영상 + 링크 + 항법 (경유점)
    ros2 launch rd_bridge bridge.launch.py nav:=false       # 항법 빼고 (drive 등 수동)
    ros2 launch rd_bridge bridge.launch.py link:=false      # 기체를 안 건드린다
    ros2 launch rd_bridge bridge.launch.py tag:=true follow:=true nav:=false \
        camera_info_file:=$HOME/rosettadrone_work/ros2/src/rd_bridge/config/camera.yaml   # 태그 추종
    ros2 launch rd_bridge bridge.launch.py usv:=true nav:=false   # USV GPS 추종
    ros2 launch rd_bridge bridge.launch.py usv:=true usv_gnss:=true nav:=false   # 진짜 USV GNSS 로

    video_node       영상 → /rc/image_raw · /rc/camera_info
    republish        /rc/image_raw → /gcs/daiv/cam/compressed (JPEG, 관제 PC 용)  (gcs, 기본 켬)
    link_node        UDP 14550 — 다운링크 → /rd/* · /cmd_vel → #84
    nav_node         경유점 → /cmd_vel                     (nav, 기본 켬)
    apriltag_node    영상 → /detections · /tf (camera→tag0)  (tag)
    tag_follow_node  /tf → /cmd_vel                        (follow)
    usv_follow_node  /usv/* → /cmd_vel · 짐벌 틸트           (usv)
    usv_gnss_node    USV 항법장비 raw 배열 → /usv/fix · heading · vel  (usv_gnss)

띄우기만 해서는 기체가 안 움직인다 — 항법·추종은 목표나 /tag/follow · /usv/follow 를 받아야 명령을 낸다.
⚠ nav · follow · usv 중 하나만 켠다 (셋 다 /cmd_vel).
⚠ 14550 은 link_node 가 독점한다 — QGC·gps_capture.py 와 같이 못 쓴다.
⚠ usv_gnss 와 tools/usv_sim.py 를 같이 띄우지 않는다 — /usv/fix 에 퍼블리셔가 둘이 된다.
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
    usv_gnss = LaunchConfiguration("usv_gnss")
    gnss_topic = LaunchConfiguration("gnss_topic")
    cam_info = LaunchConfiguration("camera_info_file")
    rectify = LaunchConfiguration("rectify")
    gcs = LaunchConfiguration("gcs")
    jpeg_quality = LaunchConfiguration("jpeg_quality")

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
        DeclareLaunchArgument("usv_gnss", default_value="false",
                              description="USV 항법장비 raw 배열 → /usv/*. usv_sim.py 와 같이 쓰지 말 것"),
        DeclareLaunchArgument("gnss_topic", default_value="/maiv/fog/raw_gnss",
                              description="USV 쪽 raw GNSS 토픽 (Float64MultiArray)"),
        DeclareLaunchArgument("camera_info_file", default_value="",
                              description="보정 결과 YAML. 비우면 대략값(fx=fy=1250)을 쓴다"),
        DeclareLaunchArgument("rectify", default_value="false",
                              description="왜곡을 펴서 발행한다 (camera_info_file 이 있어야 뜻이 있다)"),
        DeclareLaunchArgument("gcs", default_value="true",
                              description="관제 PC 용 JPEG 중계 (/gcs/daiv/cam/compressed)"),
        DeclareLaunchArgument("jpeg_quality", default_value="80",
                              description="JPEG 품질 1~100. 낮출수록 대역폭이 준다"),

        Node(package="rd_bridge", executable="video_node", name="rd_video",
             output="screen",
             # raw 는 /rc/* 로 낸다 — 관제 PC 가 /image_raw 라는 이름으로 구독하고 있어서
             # 그 이름으로 내면 5MiB 프레임이 랜선으로 빨려나가 회선이 죽는다 (1.2Gbps)
             remappings=[("image_raw", "/rc/image_raw"),
                         ("camera_info", "/rc/camera_info")],
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
             remappings=[("image_rect", "/rc/image_raw"), ("camera_info", "/rc/camera_info")],
             parameters=[os.path.join(get_package_share_directory("rd_bridge"),
                                      "config", "tags.yaml")]),

        # 태그 추종 — 뜨자마자 움직이지 않는다. /tag/follow true 를 받아야 명령을 낸다
        Node(package="rd_bridge", executable="tag_follow_node", name="rd_tag_follow",
             output="screen", condition=IfCondition(follow)),

        # USV 추종 — /usv/follow true 를 받아야 /cmd_vel · 짐벌 명령을 낸다
        Node(package="rd_bridge", executable="usv_follow_node", name="rd_usv_follow",
             output="screen", condition=IfCondition(usv)),

        # USV 항법장비(FOG/INS) 의 raw 배열 → /usv/fix · /usv/heading · /usv/vel
        # 이름만 다른 게 아니라 **타입이 다르다** (Float64MultiArray → NavSatFix) — remap 으로
        # 안 되는 자리라 노드를 둔다. 토픽 이름만 다르면 gnss_topic 으로 바꾼다
        Node(package="rd_bridge", executable="usv_gnss_node", name="rd_usv_gnss",
             output="screen", condition=IfCondition(usv_gnss),
             remappings=[("usv/raw_gnss", gnss_topic)]),

        # 관제 PC 전송용 JPEG — /rc/image_raw → /gcs/daiv/cam/compressed
        # raw 는 1.2Gbps 라 기가비트 회선에 안 들어간다. JPEG(q80) 이면 ~46Mbps.
        # out_transport 를 박으면 compressed 하나만 광고된다 (베이스 /gcs/daiv/cam 은 안 생긴다).
        # ⚠ 그래서 관제 PC 의 rqt_image_view 에서 /gcs/daiv/cam/compressed 를 직접 고르면 죽는다.
        #    그쪽에서는 republish 로 한 번 풀어서 보라 (README 참고).
        Node(package="image_transport", executable="republish",
             name="image_republisher", output="screen",
             condition=IfCondition(gcs),
             # ⚠ arguments=["raw","compressed"] 로 주면 launch 에서 out_transport 가
             #    빈 값으로 들어가 JPEG 이 안 나온다. 파라미터로 박아야 한다.
             parameters=[{"in_transport": "raw",
                          "out_transport": "compressed",
                          # 구독만 BEST_EFFORT 로 내린다 — video_node 는 RELIABLE 로 내므로
                          # (제공 ≥ 요구) 붙고, 늦은 프레임을 재전송받느라 밀리지 않는다.
                          # ⚠ 값은 소문자여야 한다. "BEST_EFFORT" 나 "sensor_data" 를 주면
                          #   unknown QoS policy reliability value 로 노드가 죽는다.
                          #   쓸 수 있는 값은 system_default · reliable · best_effort 뿐이다.
                          # ⚠ 이 override 는 republish 라서 되는 것이다. apriltag_node 는
                          #   qos_overrides 를 아예 선언하지 않아 같은 수를 못 쓴다.
                          "qos_overrides./rc/image_raw.subscription.reliability": "best_effort",
                          "out.compressed.jpeg_quality": ParameterValue(jpeg_quality,
                                                                        value_type=int)}],
             remappings=[("in", "/rc/image_raw"),
                         ("out", "/gcs/daiv/cam"),
                         ("out/compressed", "/gcs/daiv/cam/compressed")]),
    ])

#!/usr/bin/env python3
"""브리지를 한 번에 띄운다.

    ros2 launch rd_bridge bridge.launch.py                  # 영상 + 링크 + 항법 (경유점)
    ros2 launch rd_bridge bridge.launch.py nav:=false       # 항법 빼고 (drive 등 수동)
    ros2 launch rd_bridge bridge.launch.py publish_fps:=15  # 영상 발행 Hz (기본 3, 0=제한없음)
    ros2 launch rd_bridge bridge.launch.py scale:=1.0       # 영상 축소 배율 (기본 0.5=반)
    ros2 launch rd_bridge bridge.launch.py link:=false      # 기체를 안 건드린다
    ros2 launch rd_bridge bridge.launch.py tag:=true follow:=true nav:=false \
        camera_info_file:=$HOME/rosettadrone_work/ros2/src/rd_bridge/config/camera.yaml   # 태그 추종
    ros2 launch rd_bridge bridge.launch.py usv:=true nav:=false   # USV GPS 추종
    ros2 launch rd_bridge bridge.launch.py usv:=true usv_gnss:=true nav:=false   # 진짜 USV GNSS 로
    ros2 launch rd_bridge bridge.launch.py usv:=true usv_gnss:=true nav:=false usv_below:=6.0   # 그날 높이차가 다를 때 (기본 8.0)
    ros2 launch rd_bridge bridge.launch.py mission:=true          # 임무 시퀀서 (전부 띄운다)

    video_node       영상 → /gcs/daiv/cam/compressed (JPEG, 기본 3Hz·0.5배) · /gcs/daiv/camera_info
    link_node        UDP 14550 — 다운링크 → /gcs/daiv/rd/* · /gcs/daiv/cmd_vel → #84
    nav_node         경유점 → /gcs/daiv/cmd_vel                     (nav, 기본 켬)
    apriltag_node    영상 → /gcs/daiv/detections · /tf (camera→tag0)  (tag)
    tag_follow_node  /tf → /gcs/daiv/cmd_vel                        (follow)
    usv_follow_node  /usv/* → /gcs/daiv/cmd_vel · 짐벌 틸트           (usv)
    usv_gnss_node    USV 항법장비 raw 배열·euler → /usv/fix · heading · vel  (usv_gnss)
    mission_node     이륙→추종→복귀→착륙. /gcs/daiv/cmd_vel 은 안 낸다 (mission)

띄우기만 해서는 기체가 안 움직인다 — 항법·추종은 목표나 /gcs/daiv/tag/follow · /gcs/daiv/usv/follow 를 받아야 명령을 낸다.
⚠ nav · follow · usv 는 셋 다 /gcs/daiv/cmd_vel 이다. **손으로 켤 때는 하나만.**
   mission:=true 면 셋을 다 띄우고 rd_mission 이 한 번에 하나만 무장시킨다 — 배타가
   런치 파일의 구조적 보장에서 mission_node.arm_for() 의 런타임 불변식으로 옮겨간다.
   그래서 임무 모드로 띄웠으면 `ros2 topic hz /gcs/daiv/cmd_vel` 이 ≈20Hz 인지 꼭 본다
   (퍼블리셔 수는 이제 늘 3이다 — 개수가 아니라 주파수가 시험이다).
⛔ rd_cmd.py drive 는 임무 모드에서 절대 같이 쓰지 않는다 — 유일하게 /gcs/daiv/cmd_vel 을 직접 쏜다.
⚠ 14550 은 link_node 가 독점한다 — QGC·gps_capture.py 와 같이 못 쓴다.
⚠ usv_gnss 와 tools/usv_sim.py 를 같이 띄우지 않는다 — /usv/fix 에 퍼블리셔가 둘이 된다.
⚠ raw 영상 토픽은 없다. 1.2Gbps 라 누가 구독하면 회선이 죽어서 아예 안 낸다 (9/22 실제 발생).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, OrSubstitution
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
    euler_topic = LaunchConfiguration("euler_topic")
    usv_below = LaunchConfiguration("usv_below")
    mission = LaunchConfiguration("mission")
    cam_info = LaunchConfiguration("camera_info_file")
    rectify = LaunchConfiguration("rectify")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    publish_fps = LaunchConfiguration("publish_fps")
    scale = LaunchConfiguration("scale")

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
                              description="태그 추종 노드. nav 와 같이 쓰지 말 것 (둘 다 /gcs/daiv/cmd_vel)"),
        DeclareLaunchArgument("usv", default_value="false",
                              description="USV GPS 추종 노드. nav · follow 와 같이 쓰지 말 것"),
        DeclareLaunchArgument("usv_gnss", default_value="false",
                              description="USV 항법장비 raw 배열 → /usv/*. usv_sim.py 와 같이 쓰지 말 것"),
        DeclareLaunchArgument("gnss_topic", default_value="/maiv/fog/raw_gnss",
                              description="USV 쪽 raw GNSS 토픽 (Float64MultiArray)"),
        DeclareLaunchArgument("euler_topic", default_value="/maiv/fog/euler_orientation",
                              description="USV 쪽 자세 토픽 (Float32MultiArray) — 기수를 여기서 읽는다"),
        DeclareLaunchArgument("usv_below", default_value="8.0",
                              description="USV 가 이륙 지면보다 낮은 m — 틸트 보정. 현장(안벽) 값을 여기 기본값 "
                                          "한 곳에 둔다. 땅에서 이륙하면 0 (끔)"),
        DeclareLaunchArgument("mission", default_value="false",
                              description="임무 시퀀서. nav·tag·follow·usv 를 전부 같이 띄우고 "
                                          "rd_mission 이 한 번에 하나만 무장시킨다"),
        DeclareLaunchArgument("camera_info_file", default_value="",
                              description="보정 결과 YAML. 비우면 대략값(fx=fy=1250)을 쓴다"),
        DeclareLaunchArgument("rectify", default_value="false",
                              description="왜곡을 펴서 발행한다 (camera_info_file 이 있어야 뜻이 있다)"),
        DeclareLaunchArgument("jpeg_quality", default_value="80",
                              description="JPEG 품질 1~100. 낮출수록 대역폭이 준다"),
        DeclareLaunchArgument("publish_fps", default_value="3.0",
                              description="발행 상한 Hz. 0 이면 오는 대로 다 낸다(~30Hz)"),
        DeclareLaunchArgument("scale", default_value="0.5",
                              description="축소 배율 0<s<=1. 1.0 이면 원본. camera_info 도 같이 준다"),

        Node(package="rd_bridge", executable="video_node", name="rd_video",
             output="screen",
             # raw 토픽은 존재하지 않는다 — video_node 가 JPEG 으로 바로 낸다.
             # 이 이름이 곧 관제 PC 가 보는 토픽이고, apriltag 도 같은 걸 구독한다.
             # ⚠ camera_info 는 영상과 **같은 네임스페이스**에 둬야 한다. image_transport 는
             #    camera_info 이름을 베이스 토픽에서 유도한다 — 베이스가 /gcs/daiv/cam 이면
             #    /gcs/daiv/camera_info 를 찾는다. 다른 데 두면 apriltag 가 짝을 못 맞춰
             #    "do not appear to be synchronized ... Synchronized pairs: 0" 으로 검출이 0 이다.
             remappings=[("cam/compressed", "/gcs/daiv/cam/compressed"),
                         ("camera_info", "/gcs/daiv/camera_info")],
             # 형을 못 박는다 — 빈 문자열이 None 으로, "false" 가 문자열로 넘어가면
             # 노드가 InvalidParameterTypeException 으로 죽는다 (tags.yaml refine 과 같은 함정)
             parameters=[{"source": video_source, "port": video_port,
                          "camera_info_file": ParameterValue(cam_info, value_type=str),
                          "rectify": ParameterValue(rectify, value_type=bool),
                          "jpeg_quality": ParameterValue(jpeg_quality, value_type=int),
                          "publish_fps": ParameterValue(publish_fps, value_type=float),
                          "scale": ParameterValue(scale, value_type=float)}]),

        Node(package="rd_bridge", executable="link_node", name="rd_link",
             output="screen", condition=IfCondition(link),
             parameters=[{"port": mav_port}]),

        Node(package="rd_bridge", executable="nav_node", name="rd_nav",
             output="screen", condition=IfCondition(OrSubstitution(nav, mission))),

        # AprilTag → /tf (camera → tag0) · /gcs/daiv/detections
        # ⚠ image_rect 는 **베이스 이름**이다. image_transport:=compressed 를 주면
        #    실제로는 <베이스>/compressed 를 구독하고 JPEG 을 로컬에서 푼다.
        #    그래서 raw 토픽 없이도 검출이 된다. 왜곡 보정은 video_node 에서 한다
        #    (camera_info_file + rectify:=true).
        # ⚠ 태그 검출이 나쁘면 JPEG 손실을 의심할 것 — jpeg_quality:=95 로 올려 본다.
        Node(package="apriltag_ros", executable="apriltag_node", name="apriltag",
             output="screen", condition=IfCondition(OrSubstitution(tag, mission)),
             # camera_info 는 remap 하지 않는다 — 베이스에서 /gcs/daiv/camera_info 로 유도된다
             remappings=[("image_rect", "/gcs/daiv/cam"),
                         ("detections", "/gcs/daiv/detections")],
             parameters=[os.path.join(get_package_share_directory("rd_bridge"),
                                      "config", "tags.yaml"),
                         {"image_transport": "compressed"}]),

        # 태그 추종 — 뜨자마자 움직이지 않는다. /gcs/daiv/tag/follow true 를 받아야 명령을 낸다
        Node(package="rd_bridge", executable="tag_follow_node", name="rd_tag_follow",
             output="screen", condition=IfCondition(OrSubstitution(follow, mission))),

        # USV 추종 — /gcs/daiv/usv/follow true 를 받아야 /gcs/daiv/cmd_vel · 짐벌 명령을 낸다
        # usv_below 는 float 로 못박는다 — "6" 처럼 주면 int 로 읽혀 파라미터 타입이 안 맞는다
        Node(package="rd_bridge", executable="usv_follow_node", name="rd_usv_follow",
             output="screen", condition=IfCondition(OrSubstitution(usv, mission)),
             parameters=[{"usv_below": ParameterValue(usv_below, value_type=float)}]),

        # USV 항법장비(FOG/INS) 의 raw 배열 → /usv/fix · /usv/heading · /usv/vel
        # 이름만 다른 게 아니라 **타입이 다르다** (Float64MultiArray → NavSatFix) — remap 으로
        # 안 되는 자리라 노드를 둔다. 토픽 이름만 다르면 gnss_topic 으로 바꾼다
        Node(package="rd_bridge", executable="usv_gnss_node", name="rd_usv_gnss",
             output="screen", condition=IfCondition(usv_gnss),
             remappings=[("usv/raw_gnss", gnss_topic), ("usv/euler", euler_topic)]),

        # 임무 시퀀서 — /gcs/daiv/cmd_vel 을 내지 않는다. 위 셋을 하나씩 무장·해제할 뿐이다
        Node(package="rd_bridge", executable="mission_node", name="rd_mission",
             output="screen", condition=IfCondition(mission)),
    ])

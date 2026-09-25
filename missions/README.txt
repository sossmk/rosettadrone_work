미션 파일 — python3 tools/rd_cmd.py mission missions/<파일>

한 줄에 한 점. # 뒤는 주석.

    gps     <위도> <경도> [고도]
    offset  <앞 m> <오른쪽 m> [고도]

  · 고도는 상대고도(이륙 지점 기준) m. 빼거나 0 이하면 시작할 때 고도를 유지한다
  · offset 은 전부 시작하는 순간(space)의 위치·기수 기준 — 앞 점 기준이 아니다
  · gps 와 offset 을 한 파일에 섞어도 된다
  · gps 경로는 tools/gps_capture.py 로 만든다 — RC 로 날며 Enter 마다 이 형식의 줄이 쌓인다
        python3 tools/gps_capture.py missions/site_a.txt      (브리지를 끄고)

⚠ gps 줄은 손으로 옮겨 적을 때 한 자리만 틀려도 수백 m ~ km 가 된다
  (위도 0.001° ≈ 111m). 시작 전에 도구가 찍는 「구간」 거리를 꼭 본다.

파일 없이 바로 줄 때:

    ros2 topic pub --once /gcs/daiv/nav/mission std_msgs/msg/Float64MultiArray \
        '{data: [37.5665123, 126.9780456, 5.0,  37.5666000, 126.9781000, 5.0]}'
    ros2 topic pub --once /gcs/daiv/nav/mission_offset std_msgs/msg/Float64MultiArray \
        '{data: [10, 0, 0,  10, 10, 0,  0, 10, 0,  0, 0, 0]}'

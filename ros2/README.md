# ROS 2 브리지 (할일.md 의 L3)

앱과 ROS 2 사이의 다리. **영상을 `/image_raw` 로 내고, `/cmd_vel` 을 MAVLink #84 로 바꿔 쏜다.**

```
ros2/
  src/rd_bridge/
    rd_bridge/video_node.py     영상 → sensor_msgs/Image
    rd_bridge/cmd_vel_node.py   geometry_msgs/Twist → MAVLink #84 (20Hz)
    rd_bridge/mav.py            → ../../../tools/mav.py 심링크 (L1 링크 계층을 그대로 쓴다)
    launch/bridge.launch.py
```

`mav.py` 는 **심링크다.** 사본을 두면 갈라진다 — 9/11 의 `target_system` 수정 같은 게
한쪽에만 반영되면 조용히 안 먹는 버그로 돌아온다.

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ros2 && colcon build --symlink-install
source install/setup.bash
```

추가 apt 설치가 **없다.** `h264parse`(gstreamer1.0-plugins-bad) 는 필요 없다 —
실시간은 `udpsrc ! rtph264depay ! avdec_h264`(9/10 실기 확인), 파일은 OpenCV 자체
디코더(FFMPEG)가 raw Annex-B 를 바로 읽는다.

## 기체 없이 개발하기 ⭐

```bash
# 영상 — 9/10 에 받아 둔 진짜 스트림
ros2 run rd_bridge video_node --ros-args \
  -p source:=~/drone_ws/capture/mavic2-real-20260910.h264
ros2 topic hz /image_raw

# 업링크 — 앱 대역을 띄워 두고
python3 tools/fake_app.py          # 다른 터미널
ros2 run rd_bridge cmd_vel_node
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'
```

태그 검출(E)·추종(F)은 이 구성으로 전부 만들 수 있다.

## 실기

```bash
ros2 launch rd_bridge bridge.launch.py                 # 영상 + cmd_vel
ros2 launch rd_bridge bridge.launch.py cmd_vel:=false  # 영상만
```

## 2026-09-14 실측

| 무엇 | 결과 |
|---|---|
| 저장분 재생 | 555 프레임 · 1632x1088 · **31.9fps** · 놓친 read 0 |
| 파일 앞부분 | PPS 없는 20 프레임이 버려진다 — 중간부터 받은 캡처라 정상. 뒤는 깨끗 |
| cmd_vel 변환 | ROS `x .3 / y .2 / z .1 / ang.z .5` → 앱 `vx+0.30(앞) vy-0.20(왼쪽) vz+0.10(위) yawRate-28.6°/s(반시계)` |
| 발행률 | 20.2Hz (목표 20) |
| IDLE 구간 | 송신 **0발** — /cmd_vel 이 오기 전엔 한 발도 안 쏜다 |

⚠ 실기 검증은 아직 안 했다. 위는 전부 기체 없이(저장분·앱 대역) 얻은 결과다.

## 축·부호 — ROS 와 앱의 규약이 다르다

ROS REP-103 기체계는 **FLU**(x=앞, y=**왼쪽**, z=위, angular.z>0=**반시계**).
앱은 vx>0=앞, vy>0=**오른쪽**, vz>0=**위**, yawRate>0=**시계** (9/9 SIM 실측, 제약 #2).

| ROS | → 앱 | 반전? |
|---|---|---|
| `linear.x` | `vx` | 그대로 |
| `linear.y` | `vy` | **반전** (왼쪽→오른쪽) |
| `linear.z` | `vz` | **그대로** — 둘 다 위가 양수 |
| `angular.z` | `yawRate` | **반전** + rad/s→deg/s |

⚠ 할일.md 에 오래 있던 "ROS 브리지에서 vz 부호를 뒤집을 것" 은 **MAVLink NED 기준으로
쓴 메모**였다. NED 는 z 가 아래가 양수라 ROS(z=위)에서 오면 뒤집어야 맞지만, 이 앱은
그 필드를 이미 "위가 양수"로 읽는다 — 두 번 뒤집으면 도로 제자리다. **실제로 뒤집히는
축은 vy 와 yaw 이고 vz 는 아니다.** (문서는 2026-09-14 에 고쳤다.)

실기에서 다르면 코드를 고치지 말고 파라미터로: `-p scale_vy:=1.0`

## ⚠ 조종권 — 이 노드가 왜 "입을 닫는가"

앱은 #84 를 하나 받으면 `startMotion()` 에서 `setVirtualSticksEnabled(true)` 를 부른다
(`DroneModel.java:2257`). Virtual Stick 이 켜진 동안 DJI 는 **물리 RC 스틱을 무시한다.**

그래서 이 노드는 0 이라도 계속 쏘는 설계를 하지 않는다:

```
IDLE    /cmd_vel 이 온 적 없다 → 한 발도 안 쏜다 (Virtual Stick 을 미리 켜지 않는다)
ACTIVE  Twist 가 온다          → 20Hz 로 쏜다
BRAKE   Twist 가 끊겼다        → 0 을 1초 쏴서 세우고
IDLE    → 송신 정지
```

⚠⚠ **입을 닫아도 Virtual Stick 은 안 풀린다.** 2026-09-14 소스 확인 — 데드맨 만료가
부르는 `cancelMotion()` 은 타이머만 죽인다(`DroneModel.java:2279`). 조종권을 RC 로
되돌리는 경로는 실질적으로 **앱 화면의 Safety(자물쇠) 버튼** 하나뿐이다
(`MainActivity:824` → `setSafetyEnabled(true)` → `setVirtualSticksEnabled(false)`).

```
RC 로 되돌리는 순서:
  1. 노드를 끈다 (안 끄면 50ms 뒤에 다시 빼앗아 간다 — startMotion 이 매번 켠다)
  2. 스틱에서 손 떼고 5초 (timerIgnoreMavLink 만료)
  3. 앱 화면의 Safety(자물쇠) 버튼
  안 되면 S 모드 (9/11 에 실제로 이걸로 복구)
```

## 아직 없는 것

- **CameraInfo 발행** — 태그의 상대 위치(solvePnP)를 뽑으려면 내부파라미터 보정이
  먼저다(할일.md E). 보정 안 된 값을 그럴듯하게 흘리면 나중에 거리가 왜 안 맞는지
  찾기 어려워지므로 일부러 안 넣었다
- `/tag_pose` — E·F 의 몫
- 텔레메트리 토픽화 — 지금은 cmd_vel_node 안에서만 읽어 로그로 찍는다

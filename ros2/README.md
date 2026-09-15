# ROS 2 브리지 + 항법 (할일.md 의 L3)

```
ros2/src/rd_bridge/rd_bridge/
  video_node.py   영상 → /image_raw
  link_node.py    UDP 14550 독점 — 다운링크 → /rd/*, /cmd_vel → MAVLink #84
  nav_node.py     /nav/goal → 유도 루프(우리 것) → /cmd_vel      ★ flyTo() 대체
  mav.py          → ../../../tools/mav.py 심링크 (L1 링크 계층)
```

`mav.py` 는 **심링크다.** 사본을 두면 갈라진다 — 한쪽에만 남은 수정은 "조용히 안 먹는
버그"로 돌아온다(9/11 에 실제로 겪었다).

## 상태 — 무엇이 확인됐고 무엇이 안 됐나

⚠ **실기 검증은 하나도 안 됐다.** 아래 ✅ 는 "코드가 돌고 안전 게이트가 걸린다"는
크래시 점검이고, **검증이 아니다.** 기체에 붙여 봐야 아는 것들이 그대로 남아 있다:
핫스팟에서 20Hz 가 유지되는지, 영상 지연, 유도 이득이 실제 기체에서 맞는지, 타력
(0.6~0.8m)이 우리 감속 프로파일로 실제로 흡수되는지.

| | 크래시 점검 | 실기 |
|---|---|---|
| `video_node` 실시간 | ⬜ | ⬜ |
| `link_node` 다운링크 → 토픽 | ✅ | ⬜ |
| `link_node` /cmd_vel → #84 20Hz | ✅ | ⬜ |
| `nav_node` P 제어 (9/15 단순화) | ✅ | ⬜ |

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ros2 && colcon build --symlink-install
source install/setup.bash
```

`h264parse`(plugins-bad)는 필요 없다. AprilTag(`tag:=true`)를 쓸 때만:
```bash
sudo apt update && sudo apt install -y ros-jazzy-apriltag-ros ros-jazzy-camera-calibration
```

## 실행

```bash
ros2 launch rd_bridge bridge.launch.py              # 영상 + 링크 + 항법
ros2 launch rd_bridge bridge.launch.py nav:=false   # 항법 빼고 (수동 cmd_vel)
ros2 launch rd_bridge bridge.launch.py link:=false  # 영상만 (기체를 안 건드린다)
ros2 launch rd_bridge bridge.launch.py tag:=true link:=false nav:=false   # 영상 + AprilTag
```

띄워 두는 것 자체는 기체를 건드리지 않는다 — `link_node` 는 `/cmd_vel` 이 오기 전에
한 발도 쏘지 않고, `nav_node` 는 `/nav/goal` 이 오기 전에 `/cmd_vel` 을 내지 않는다.

띄운 뒤 토픽이 제대로 차는지는 다른 터미널에서:

```bash
python3 tools/topic_check.py            # 10초 듣고 토픽별 Hz·최대 간격·값 판정 (0=통과)
python3 tools/topic_check.py list       # 보이는 토픽
python3 tools/topic_check.py echo /rd/fix
```

⚠⚠ **UDP 14550 은 `link_node` 가 독점한다.** 두 번째 bind 는 커널에 따라 성공해
버리는데 그러면 패킷이 둘로 쪼개져 **양쪽 다 망가진다.** QGC·`decode_check.py`·
`gps_capture.py` 와 **동시에 못 쓴다.** `rd_cmd.py` 는 이제 브리지를 거치므로 같이 쓴다.

## 토픽

**다운링크** (`link_node`, 5Hz — 앱이 주는 만큼)

| 토픽 | 형 | 내용 |
|---|---|---|
| `/rd/fix` | `sensor_msgs/NavSatFix` | 위경도 · `altitude` 에 **상대고도**를 넣는다 |
| `/rd/rel_alt` | `std_msgs/Float32` | 상대고도 m (이륙 지점 기준) |
| `/rd/attitude` | `geometry_msgs/Vector3Stamped` | roll · pitch · yaw, **단위 deg** |
| `/rd/heading` | `std_msgs/Float64` | 기수 deg (0=북, 시계방향) |
| `/rd/velocity` | `geometry_msgs/TwistStamped` | 대지 속도벡터 m/s, **ENU**(x=동 y=북 z=위) |
| `/rd/gspeed` | `std_msgs/Float32` | 대지속도 크기 m/s |
| `/rd/battery` | `sensor_msgs/BatteryState` | 전압·전류 (`patches/03` 적용 후 유효) |
| `/rd/ack` | `std_msgs/Int32MultiArray` | `[명령번호, 결과]` — COMMAND_ACK |

**업링크** `/cmd_vel` (`geometry_msgs/Twist`, 앱 규약) → #84, 20Hz · 캡 수평 2.5 / 수직 1.5 m/s / 75°/s
`/rd/command` (`std_msgs/Float64MultiArray`: `[명령번호, p1..p7]`) → COMMAND_LONG. `rd_cmd.py` 가 짐벌·이륙에 쓴다
**항법** `/nav/goal` (`NavSatFix`) · `/nav/cancel` (`Empty`) · `/nav/status` · `/nav/distance`

⚠ 같은 값을 재발행하지 않는다 — 구독자가 "좌표가 살아있다"고 착각하면 안 된다.
링크가 끊긴 것과 기체가 멈춰 있는 것은 구별돼야 한다.

**영상** (`video_node`) `/image_raw` · `/camera_info`(**대략값**, 아래) — 둘은 같은 stamp·`frame_id=camera`

## AprilTag — 거리·기울기 청사진 (2026-09-15)

`apriltag_ros`(apt) 가 `/image_raw` + `/camera_info` 를 받아 **`/tf` 에 카메라→태그 자세**를 낸다.

**보정 전 대략값.** Mavic 2 Pro 대각 화각 77°, 스트림 1632x1088(대각 1961px) →
`f = 980.7 / tan 38.5° ≈ 1233` → video_node 기본값 `fx=fy=1250`, 중심=프레임 가운데, 왜곡 0.
거리는 f 에 비례해 틀리므로 **±10% 정도**. 보정하면 `-p fx:=… -p fy:=… -p cx:=… -p cy:=…` 만 바꾼다.

**태그.** `tags/tag36h11_id0.png`(36h11, ID 0) 를 인쇄한다. 그림은 12칸이고 가운데 검은 사각형이 8칸이다.
**검은 사각형 바깥 한 변을 자로 재서** `config/tags.yaml` 의 `size`(m)에 넣는다 — 흰 여백은 빼고.
(A4 에 꽉 차게 뽑으면 칸당 약 1.7cm → size ≈ 0.14). 고친 뒤 `colcon build` 다시.
검출 한계 가늠: 16cm 태그가 10m 에서 1250×0.16/10 = 20px.

```bash
ros2 launch rd_bridge bridge.launch.py tag:=true link:=false nav:=false
ros2 topic hz /detections                          # 태그가 보이면 영상 fps 근처
ros2 run tf2_ros tf2_echo camera tag36h11:0        # 거리·자세
```

**읽는 법** — `Translation [x y z]` m, 카메라 광학 좌표계:

| | 방향 | 짐벌 -90°(아래)일 때 |
|---|---|---|
| x | 화면 오른쪽 | 수평 오차 |
| y | 화면 아래 | 수평 오차 |
| **z** | **카메라 앞 = 거리** | **≈ 고도** |

`Rotation RPY` 는 태그가 카메라에 대해 틀어진 각이다. **정면에 세운 값을 먼저 적어 두고**
기울였을 때의 차이로 본다(기준 자세가 0 이 아닐 수 있다).

**확인할 것 (기체 카메라 앞, 줄자)**

| 실제 거리 | z | 차이 % | 30° 돌렸을 때 RPY 변화 |
|---|---|---|---|
| 1 m | | | |
| 2 m | | | |
| 3 m | | | |

차이가 **일정 비율**이면 f 문제(보정으로 해결), **들쭉날쭉**이면 검출·해상도·손실 프레임 문제.

## ★ 항법을 우리가 갖는다 — `flyTo()` 를 안 쓰는 이유

앱의 `flyTo()` 는 DJI 네이티브 웨이포인트 미션이 **아니다.** 매 틱 PID 로 속도를 다시
계산해 `setVelocities()` 를 부르는 것이고(`processFlyTo()`), 그 채널은 cmd_vel 과
**완전히 같다**(제약 #19). 앱이 하는 일을 우리가 그대로 할 수 있다.

가져와서 얻는 것:
- **이득·감속 프로파일·도착 판정을 우리가 정한다.** 앱 것은 캡이 박혀 있고 고칠 수 없다
- **태그 추종과 같은 루프에 들어간다** — GPS 목표와 태그 상대위치(E·F)를 한 유도기에서
  섞을 수 있다. 정밀 착륙(G)은 이게 없으면 안 된다
- `MISSION_ITEM_INT` 경로를 안 탄다 — `target_system` 함정(제약 #18)도 지나간다
- **무슨 일이 일어나는지 보인다.** 앱 안 PID 는 로그가 없다

물려받는 것은 그대로다 — `#84` 를 쓰므로 **P 모드**(#4) · **스틱 5초 차단**(#5) ·
**Virtual Stick 조종권**(#21).

### 쓰는 법

```bash
# 현재 위치 기준: x=앞 m, y=오른쪽 m, z=상대고도 m (z<=0 이면 고도 유지)
#   받는 순간의 기수로 북·동으로 바꿔 고정한다 (9/15 실기 후 북·동 → 기체 기준으로 변경)
ros2 topic pub --once /nav/goal_offset geometry_msgs/msg/Vector3 '{x: 5.0, y: 0.0, z: 0.0}'
# 좌표로 (altitude = 상대고도)
ros2 topic pub --once /nav/goal sensor_msgs/msg/NavSatFix '{latitude: 37.5665, longitude: 126.9780, altitude: 10.0}'
ros2 topic pub --once /nav/cancel std_msgs/msg/Empty '{}'      # 중단
```

### 하는 일 — 앱 flyTo() 와 같은 P 제어

```
rel   = 목표 방위 - 기수
speed = min(max_speed, kp × 거리)          kp 0.35 (앱과 같다), max_speed 2.5
vx = speed·cos(rel)   vy = speed·sin(rel)   ← 앱 규약: y>0 오른쪽
vz = clamp(kp_alt × 고도오차, ±max_vz)
```

- 거리·고도오차가 둘 다 `arrive_radius`(1.0m) 안이면 0 을 한 번 내고 멈춘다
- 좌표가 1초 넘게 안 오면 멈춘다
- 기록: `~/drone_ws/navlog/<시각>.csv` (t, lat, lon, alt, hdg, dist, vx, vy, vz)
- 속도 캡은 `link_node`(`max_vx`·`max_vy` 2.5) 에도 있다
- 이륙은 하지 않는다

## 축·부호 — /cmd_vel 은 앱 규약 그대로 (2026-09-15)

REP-103(FLU) 이 아니다. 앱이 받는 뜻 그대로 쓴다 (9/9 SIM 실측, 제약 #2):

| `/cmd_vel` | 앱 | >0 의 뜻 |
|---|---|---|
| `linear.x` | `vx` | 앞 |
| `linear.y` | `vy` | **오른쪽** |
| `linear.z` | `vz` | 위 |
| `angular.z` (rad/s) | `yawRate` (deg/s) | **시계** |

`link_node` 는 부호를 바꾸지 않는다(`scale_*` 전부 1). ⚠ `teleop_twist_keyboard` 같은 ROS
표준 도구는 좌우·회전이 반대로 간다.

⚠ `/rd/velocity`(다운링크)는 **지구 기준 ENU** 이고 `/cmd_vel` 은 **기체 기준**이다. 다른 좌표계다.

## ⚠ 조종권을 RC 로 되돌리는 순서

앱은 #84 를 하나 받으면 `startMotion()` 에서 `setVirtualSticksEnabled(true)` 를 부르고
(`DroneModel.java:2257`), 그동안 DJI 는 **물리 RC 스틱을 무시한다.** 명령을 끊어도
데드맨이 부르는 `cancelMotion()` 은 **타이머만 죽인다**(`:2279`) — 조종권은 안 돌아온다.

```
① 노드를 끈다 (안 끄면 50ms 뒤에 다시 빼앗아 간다)
② 스틱에서 손 떼고 5초 (timerIgnoreMavLink 만료)
③ 앱 화면의 Safety(자물쇠) 버튼   ← 실제로 푸는 건 이것뿐이다 (MainActivity:824)
안 되면 S 모드 (9/11 에 실제로 이걸로 복구)
```

## 개발 편의 (검증이 아니다)

- `video_node -p source:=<파일.h264>` — 저장분 재생. 기체 없이 `/image_raw` 가 나온다
- `tools/fake_app.py` — 앱 대역. #84·#76 을 디코드해 보여주고 ACK 를 돌려준다

둘 다 **크래시 점검과 개발 편의용**이다. 기체 없이 얻은 결과는 배관이 이어졌다는
것까지만 말해 준다 — 기체가 실제로 어떻게 움직이는지는 아무것도 말해 주지 않는다.

## 아직 없는 것

- **`CameraInfo`** — 태그의 상대 위치(solvePnP)에 필요. 보정 전 값을 그럴듯하게
  흘리면 E 에서 거리가 왜 안 맞는지 찾기 어려워지므로 일부러 안 넣었다
- `/tag_pose` 와 태그 추종 — E·F. **유도기가 준비됐으니 여기에 태그를 먹이면 된다**
- 이륙·착륙 명령 (A `rd_cmd.py` 의 몫)

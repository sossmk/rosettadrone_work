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
| `apriltag_node` 기동 · `/tf`(`tag0`) | ✅ | ⬜ |
| `tag_follow_node` 계산 (태그 PNG 로) | ✅ | ⬜ |

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
ros2 launch rd_bridge bridge.launch.py tag:=true follow:=true nav:=false  # 태그 추종
ros2 launch rd_bridge bridge.launch.py camera_info_file:=<경로>/camera.yaml rectify:=true
```

띄워 두는 것 자체는 기체를 건드리지 않는다 — `link_node` 는 `/cmd_vel` 이 오기 전에
한 발도 쏘지 않고, `nav_node` 는 `/nav/goal` 이 오기 전에, `tag_follow_node` 는
`/tag/follow true` 가 오기 전에 `/cmd_vel` 을 내지 않는다.

⚠ `nav_node`·`rd_cmd.py`·`tag_follow_node` 는 **셋 다 `/cmd_vel` 에 쓴다. 한 번에 하나만.**

띄운 뒤 토픽이 제대로 차는지는 다른 터미널에서:

```bash
ros2 topic list
ros2 topic hz /rd/attitude              # 10Hz 근처여야 — 토픽별 기대 주기는 토픽.md
ros2 topic echo /rd/fix
```

⚠⚠ **UDP 14550 은 `link_node` 가 독점한다.** 두 번째 bind 는 커널에 따라 성공해
버리는데 그러면 패킷이 둘로 쪼개져 **양쪽 다 망가진다.** QGC·
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

**영상** (`video_node`) `/image_raw` · `/camera_info` — 둘은 같은 stamp·`frame_id=camera`
보정 파일을 주기 전에는 `/camera_info` 가 **대략값**이다 (아래 「카메라 보정」)
**태그** (`tag:=true`) `/detections` · `/tf`(`camera` → `tag0`)
**추종** (`follow:=true`) `/tag/follow` (`Bool`) · `/tag/cancel` (`Empty`) → `/cmd_vel`

## 카메라 보정 (2026-09-16)

보정 전에는 `video_node` 가 **대략값** `/camera_info` 를 낸다 — Mavic 2 Pro 대각 화각 77°,
스트림 1632x1088(대각 1961px) → `f = 980.7 / tan 38.5° ≈ 1233` → `fx=fy=1250`, 중심=프레임
가운데, 왜곡 0. 거리는 f 에 비례해 틀리므로 **±10% 정도**.

체커보드로 보정하면 그 결과 파일을 그대로 먹인다 (`fx` 파라미터를 하나씩 고치지 않는다):

```bash
# 1) 영상만 띄우고
ros2 launch rd_bridge bridge.launch.py link:=false nav:=false
# 2) 보정 — --size 는 내부 코너 수, --square 는 한 칸 m. 자로 잰 값을 넣는다
ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.025 \
  --no-service-check --ros-args -r image:=/image_raw
# 3) SAVE 를 누르면 /tmp/calibrationdata.tar.gz 에 ost.yaml 이 들어 있다
tar xzf /tmp/calibrationdata.tar.gz -C /tmp ost.yaml
cp /tmp/ost.yaml ros2/src/rd_bridge/config/camera.yaml
# 4) 그 파일로 다시 띄운다
ros2 launch rd_bridge bridge.launch.py tag:=true \
  camera_info_file:=$PWD/ros2/src/rd_bridge/config/camera.yaml rectify:=true
```

- `--no-service-check` 가 필요하다 — `video_node` 에 `set_camera_info` 서비스가 없다
- `rectify:=true` 면 `video_node` 가 **왜곡을 펴서** 내보내고, `camera_info` 의 `K` 도 편 값으로
  바꿔 `D` 를 0 으로 낸다. `image_proc/rectify_node` 를 안 쓰는 이유는 그게 **영상만 펴고
  camera_info 는 그대로** 내보내서, 편 영상에 안 편 `K·D` 가 붙기 때문이다
- 보정 해상도가 지금 영상과 다르면 에러 로그를 낸다. **고쳐 쓰지 않는다** — 다시 보정할 것

## AprilTag — 거리·자세 (2026-09-15, 9/16 보완)

`apriltag_ros`(apt) 가 `/image_raw` + `/camera_info` 를 받아 **`/tf` 에 카메라→태그 자세**를 낸다.

⚠ **`/camera_info` 가 없으면 태그가 하나도 안 잡힌다.** apriltag_ros 는
`image_transport::CameraSubscriber` 로 영상과 camera_info 를 **쌍으로** 구독해서, 한쪽이
없으면 검출 콜백 자체가 안 돈다. "왜 검출이 0 이지" 의 첫 번째 용의자다.

⚠ **`/detections` 에는 자세가 없다** — family·id·중심·코너·homography 뿐이다.
거리와 자세는 **`/tf` 로만** 나온다.

**태그.** **지금 쓰는 인쇄물은 검은 사각형 한 변 192mm** (작은 한 칸 24mm · 36h11 은 검은
사각형이 8칸이라 24×8=192). `config/tags.yaml` 의 `size: 0.192` 가 이 값이다.

⚠ 저장소의 `tags/tag36h11-id0-120mm-A4.pdf` 는 **이것과 다른 것**이다 (A4 100% → 120mm).
그림 전체가 12칸이라 192mm 태그의 인쇄면은 24×12 = **288mm 로 A4 에 안 들어간다.**
새로 뽑을 때는 크기를 **재서** `size` 를 고칠 것 — 파일 이름을 믿지 말 것.

다른 크기로 뽑으려면 생성 도구와 원본 PNG 를 태그에서 되살린다:

```bash
git show v1-2026-09-16:tools/make_print.py    > tools/make_print.py
git show v1-2026-09-16:tags/tag36h11_id0.png  > tags/tag36h11_id0.png
```

인쇄는 **100%(페이지에 맞춤 끄기)**. 종이에 찍힌 100mm 눈금을 자로 재서 확인하고,
검은 사각형도 재서 다르면 그 값을 `size` 에 넣고 `colcon build` 를 다시 한다.

**검출 한계 가늠** (화면 픽셀 = fx × size / 거리, fx≈1250):

| 태그 | 3m | 5m | 8m |
|---|---|---|---|
| **192mm (지금 쓰는 것)** | **80px** | **48px** | **30px** |
| 120mm (A4 한 장) | 50px | 30px | 19px |

그래서 `detector.decimate` 를 **1.0** 으로 내렸다 — 2.0 은 검출용 해상도를 반으로 깎아
탑뷰 5m 에서 15px 밖에 안 남는다.

```bash
ros2 launch rd_bridge bridge.launch.py tag:=true link:=false nav:=false
ros2 topic hz /detections                # 태그가 보이면 영상 fps 근처
ros2 run tf2_ros tf2_echo camera tag0    # 거리·자세
```

**프레임 이름은 `tag0` 이다.** `tags.yaml` 의 `tag.ids`/`tag.frames` 로 못 박았다 —
안 박으면 판마다 `36h11:0` / `tag36h11:0` 로 달라져 `tf2_echo` 가 헛다리를 짚는다.

**읽는 법** — `Translation [x y z]` m, 카메라 광학 좌표계:

| | 방향 | 짐벌 -90°(아래)일 때 |
|---|---|---|
| x | 화면 오른쪽 | 기체 오른쪽 오차 |
| y | 화면 아래 | 기체 **뒤** 오차 (앞 = -y) |
| **z** | **카메라 앞 = 거리** | **≈ 태그 위 고도** |

`Rotation` 은 태그가 카메라에 대해 틀어진 각이다. 똑바로 선 태그를 똑바로 찍으면
태그의 **x축이 화면 오른쪽(+90°)** 을 가리킨다(태그 PNG 로 실측) — `tag_follow_node` 의
`yaw_offset` 90° 는 이 값을 0 으로 맞추려고 빼는 것이다.

**안 잡힐 때** — `tools/tag_check.py` 가 네 가지 family(태그 규격)를 다 돌려 본다.
굴러다니는 태그가 36h11 이 아닐 수 있다.
```bash
python3 tools/tag_check.py file 사진.jpg --sweep   # decimate 를 바꿔 가며
python3 tools/tag_check.py live                    # /image_raw 를 보면서
```

**확인할 것 (기체 카메라 앞, 줄자)**

| 실제 거리 | z | 차이 % |
|---|---|---|
| 1 m | | |
| 2 m | | |
| 3 m | | |

차이가 **일정 비율**이면 `size` 나 f 문제(보정으로 해결), **들쭉날쭉**이면 검출·해상도·손실 프레임 문제.

## 태그 추종 — `tag_follow_node` (2026-09-16)

**짐벌 -90°(탑뷰)** 로 태그 위에 가서 서고, 기수를 태그에 맞춘다. `nav_node` 와 같은 단순 P 제어.

```bash
ros2 launch rd_bridge bridge.launch.py nav:=false tag:=true follow:=true \
  camera_info_file:=$PWD/ros2/src/rd_bridge/config/camera.yaml rectify:=true
ros2 topic pub --once /tag/follow std_msgs/msg/Bool '{data: true}'    # 켜기
ros2 topic pub --once /tag/cancel std_msgs/msg/Empty '{}'             # 끄기
```

⚠ **`nav_node`·`rd_cmd.py` 와 동시에 쓰지 않는다** — 셋 다 `/cmd_vel` 에 쓴다.
⚠ 뜨자마자 움직이지 않는다. `/tag/follow true` 를 받아야 명령을 낸다.

| 파라미터 | 기본 | 뜻 |
|---|---|---|
| `tag_frame` | `tag0` | `/tf` 에서 고를 태그 프레임 |
| `hold_alt` | 3.0 | 태그 위 몇 m 에 설까 |
| `kp` · `max_speed` | 0.35 · 0.8 | 수평 P 이득 · m/s |
| `kp_alt` · `max_vz` | 0.35 · 0.5 | 수직 |
| `deadband` | 0.3 | 이 안이면 수평 정지 (m) |
| `sign_fwd` · `sign_right` · `sign_yaw` | -1 · +1 · +1 | **실기 지상 확인으로 확정한다** |
| `yaw_offset` | 90.0 | 똑바로 맞춰진 상태를 0° 로 (위 「읽는 법」 참고) |
| `yaw_enable` · `kp_yaw` · `max_yaw_rate` · `yaw_deadband` | true · 0.5 · 20°/s · 5° | 기수 맞추기 |
| `lost_secs` · `giveup_secs` | 0.5 · 3.0 | 안 보이면 제자리 → 완전 해제 |
| `publish` | true | `false` 면 계산·로그만 (지상 dry-run) |

`follow_cmd()` 한 함수에 제어를 몰아 뒀다 — **나중에 `nav_node` 의 한 모드로 그대로 옮기려고**
(GPS 이동 → 도착 → 태그 발견 → 추종 임무 시퀀스). 기록은 `~/drone_ws/taglog/*.csv`.

⚠ 짐벌 -90° 에서는 **기수를 돌리면 화면이 통째로 돈다** — 위치 루프와 yaw 루프가 서로를
흔들 수 있다. 첫 시험은 `yaw_enable:=false` 로 위치만 본 뒤 켠다.

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

**개발 편의용**이다. 기체 없이 얻은 결과는 배관이 이어졌다는
것까지만 말해 준다 — 기체가 실제로 어떻게 움직이는지는 아무것도 말해 주지 않는다.

## 아직 없는 것

- **실기로 확인한 카메라 보정값** — `config/camera.yaml` 은 9/16 실기에서 만든다.
  그 전까지 `/camera_info` 는 대략값이고 거리는 ±10%
- **태그 추종의 실기 확인** — `tag_follow_node` 는 노트북에서 계산만 맞춰 봤다.
  `sign_fwd`·`sign_right`·`sign_yaw` 는 지상 확인 전까지 **추정값**이다
- 임무 시퀀스 (GPS 이동 → 태그 발견 → 추종) — `nav_node` 와 합치는 건 추종이 된 뒤에
- 이륙·착륙 명령 (A `rd_cmd.py` 의 몫)

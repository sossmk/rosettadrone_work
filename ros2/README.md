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
| `nav_node` 유도·안전 게이트 | ✅ | ⬜ |

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ros2 && colcon build --symlink-install
source install/setup.bash
```

추가 apt 설치가 없다. `h264parse`(plugins-bad)는 필요 없다.

## 실행

```bash
ros2 launch rd_bridge bridge.launch.py              # 영상 + 링크 + 항법
ros2 launch rd_bridge bridge.launch.py nav:=false   # 항법 빼고 (수동 cmd_vel)
ros2 launch rd_bridge bridge.launch.py link:=false  # 영상만 (기체를 안 건드린다)
```

띄워 두는 것 자체는 기체를 건드리지 않는다 — `link_node` 는 `/cmd_vel` 이 오기 전에
한 발도 쏘지 않고, `nav_node` 는 `/nav/goal` 이 오기 전에 `/cmd_vel` 을 내지 않는다.

⚠⚠ **UDP 14550 은 `link_node` 가 독점한다.** 두 번째 bind 는 커널에 따라 성공해
버리는데 그러면 패킷이 둘로 쪼개져 **양쪽 다 망가진다.** QGC·`decode_check.py`·
`cmd_vel_drive.py` 와 **동시에 못 쓴다.**

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

**업링크** `/cmd_vel` (`geometry_msgs/Twist`) → #84, 20Hz
**항법** `/nav/goal` (`NavSatFix`) · `/nav/cancel` (`Empty`) · `/nav/status` · `/nav/distance`

⚠ 같은 값을 재발행하지 않는다 — 구독자가 "좌표가 살아있다"고 착각하면 안 된다.
링크가 끊긴 것과 기체가 멈춰 있는 것은 구별돼야 한다.

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

### 목표 주기

```bash
ros2 topic pub --once /nav/goal sensor_msgs/msg/NavSatFix \
  '{latitude: 37.5665, longitude: 126.9780, altitude: 10.0}'   # altitude = 상대고도
ros2 topic pub --once /nav/cancel std_msgs/msg/Empty '{}'      # 중단
```

목표 좌표는 `python3 tools/goto_gps.py capture` 로 잰다. (같은 파일의 `goto` 는
**폐기됐다** — 그게 `flyTo()` 를 쓰던 옛 경로다.)

### 유도 법칙

```
dist, brg = 목표까지 거리·방위각
rel       = brg - 기수                      (0=앞, +90=오른쪽)
speed     = min(max_speed, kp_pos × max(0, dist - coast))
vx = speed·cos(rel)    vy = -speed·sin(rel)   ← ROS FLU 는 y 가 왼쪽
vz = clamp(kp_alt × 고도오차)
```

⚠ `speed` 에서 **`coast`(기본 0.7m)를 빼는 이유**: 정지 명령 후에도 **0.6~0.8m 더 간다**
(제약 #6, 9/9 실측). 목표보다 `coast` 앞에서 속도를 0 으로 만들고 **타력으로 들어가게**
한다. 끝까지 밀면 반드시 지나친다.

### 안전 게이트 — 항법이 우리 것이니 이건 우리 책임이다

| | 무엇 | 기본값 |
|---|---|---|
| 1 | **좌표가 오래되면 즉시 정지** | `stale_secs` 0.6초 |
| 2 | fix 없으면 목표 거부 / 비행 중 상실하면 중단 | — |
| 3 | **지오펜스** — 먼 목표 거부, 벗어나면 중단 | `max_range` 50m |
| 4 | **진척 없으면 중단** ← P 모드가 아니어서 명령이 조용히 버려지는 경우가 여기서 잡힌다 | 5초에 0.5m |
| 5 | 최대 시간 | `max_secs` 120초 |
| 6 | 너무 낮은 목표 고도 거부 | `min_goal_alt` 2m |
| 7 | 멈출 때 0 을 0.5초 쏜 뒤 **발행을 끊는다** | — |

⚠ **속도 캡이 두 군데 있다** — `nav_node`(`max_speed` 0.5) 와 `link_node`(`max_vx`·
`max_vy` 0.5). link 쪽이 마지막 클램프다. 올릴 때는 **둘 다** 올려야 한다(일부러
이중으로 뒀다).

⚠ `nav_node` 는 **이륙을 하지 않는다.** 이미 떠 있는 상태를 전제한다.

## 축·부호 — ROS 와 앱의 규약이 다르다

ROS REP-103 기체계는 **FLU**(x=앞, y=**왼쪽**, z=위, angular.z>0=**반시계**).
앱은 vx>0=앞, vy>0=**오른쪽**, vz>0=**위**, yawRate>0=**시계** (9/9 SIM 실측, 제약 #2).

| ROS | → 앱 | |
|---|---|---|
| `linear.x` | `vx` | 그대로 |
| `linear.y` | `vy` | **반전** (왼쪽→오른쪽) |
| `linear.z` | `vz` | **그대로** — 둘 다 위가 양수 |
| `angular.z` | `yawRate` | **반전** + rad/s→deg/s |

⚠ 할일.md 에 오래 있던 "vz 부호를 뒤집을 것" 은 **MAVLink NED 기준으로 쓴 메모**였다.
NED 는 z 가 아래 양수라 ROS(z=위)에서 오면 뒤집어야 맞지만, 이 앱은 그 필드를 이미
"위가 양수"로 읽는다 — 두 번 뒤집으면 도로 제자리다. **뒤집히는 축은 `vy` 와
`angular.z` 이고 `vz` 는 아니다.** (2026-09-14 에 고쳤다.)

실기에서 다르면 코드를 고치지 말고 파라미터로: `-p scale_vy:=1.0`

⚠ 혼동 주의 — `/rd/velocity`(다운링크)의 `z>0=위` 는 **지구 기준 ENU** 이야기이고,
`/cmd_vel` 의 `linear.z` 는 **기체 기준 BODY** 다. 둘은 다른 좌표계다.

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
- `tools/fake_app.py` — 앱 대역. #84·#73·#76 을 디코드해 보여주고 ACK 를 돌려준다

둘 다 **크래시 점검과 개발 편의용**이다. 기체 없이 얻은 결과는 배관이 이어졌다는
것까지만 말해 준다 — 기체가 실제로 어떻게 움직이는지는 아무것도 말해 주지 않는다.

## 아직 없는 것

- **`CameraInfo`** — 태그의 상대 위치(solvePnP)에 필요. 보정 전 값을 그럴듯하게
  흘리면 E 에서 거리가 왜 안 맞는지 찾기 어려워지므로 일부러 안 넣었다
- `/tag_pose` 와 태그 추종 — E·F. **유도기가 준비됐으니 여기에 태그를 먹이면 된다**
- 이륙·착륙 명령 (A `rd_cmd.py` 의 몫)

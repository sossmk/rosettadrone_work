# RosettaDrone 작업 씨앗 (drone_ws seed)

> **2026-09-07 부터 GitHub 비공개 저장소가 정본입니다** —
> `https://github.com/sossmk/rosettadrone_work`.
> 어느 쪽이 최신인지 날짜로 판단하던 방식은 끝났다. `git pull` 하면 기계가 판정한다.

**이 폴더 하나면 새 PC에서 작업을 그대로 이어갈 수 있습니다.** 약 170KB 입니다.

## 새 PC에서 할 일

```bash
# gh 가 없으면 먼저: sudo apt install -y gh && gh auth login && gh auth setup-git
git clone https://github.com/sossmk/rosettadrone_work.git ~/drone_ws/backup
```

기록·도구·인증키가 전부 따라옵니다. **여기까지가 대부분의 경우 필요한 전부입니다.**

### APK 를 다시 빌드해야 할 때만 (선택)

```bash
bash ~/drone_ws/backup/setup_new_pc.sh
```

JDK 11 · Android SDK 5종 · 소스(커밋 고정) · 빌드 · QGC 까지 되살립니다.
20~40분 · 2.7GB 를 받고, 끝나면 188MB APK 가 나옵니다.

⚠ **앱 소스를 고칠 때만 필요합니다.** APK 는 RM500 안에서 도는 앱이라, 새로 빌드해도
**RM500 이 옆에 있어야 설치**할 수 있다. 기록을 읽거나 PC쪽 파이썬 도구(`tools/`)를
고치는 일은 clone 만으로 된다.

## 왜 2.7GB를 통째로 안 옮기는가

| 원래 있던 것 | 크기 | 옮기나 | 왜 |
|---|---|---|---|
| `rosettadrone/` | 1.3GB | ❌ | 공개 저장소의 **손 안 댄** 복사본. 다른 건 `keys.xml` 4줄뿐 |
| `qgc/` | 735MB | ❌ | 공식 릴리스에서 재다운로드 |
| `jdk/` + tarball | 499MB | ❌ | Adoptium 에서 재다운로드 |
| cmdline-tools.zip | 174MB | ❌ | 구글에서 재다운로드 |
| **기록·도구·키** | **152KB** | ✅ | **이것만 세상에 하나뿐** |

재다운로드가 되려면 **버전이 정확해야** 하는데, 그 버전들은 `setup_new_pc.sh` 안에 박아 놨습니다.
(Gradle 6.8.3 은 Java 16+ 에서 못 돌고 sdkmanager 는 Java 17+ 를 요구하는 등, 조합이 까다롭습니다.)

## 들어 있는 것

| 경로 | 내용 |
|---|---|
| `진행상황.md` | **누적 기록.** 날짜별 블록 + 현재 상태 표. 여기부터 읽으세요 |
| `할일.md` | **앞으로 할 것.** AR 태그 추종·정밀 착륙 패키지의 구조와 순서, 지고 갈 제약 |
| `patches/` | 앱 소스 패치 (함정 5 — 영상 켜면 죽는 문제) |
| `진행기록.md` | 명령·에러 원문 |
| `rosettadrone_progress.html` | 보고용 요약 |
| `tools/link_check.sh` | **T0 — 기체 켜기 전 링크 위생.** IP·포트점유·방화벽을 한 번에 보고, **앱에 넣을 GCS IP 를 찍어 준다** |
| `tools/udp_probe.py` | UDP 14550 에 뭐가 오는지 센다 (앱↔방화벽↔QGC 원인 분리) |
| `tools/decode_check.py` | **T1 — 다운링크.** MAVLink 를 풀어 실제 값을 보고 **0,0 의 원인을 판정**한다 |
| `tools/uplink_test.py` | **T2 — 업링크.** PC→앱 명령 경로가 열렸는지 본다 |
| `tools/gimbal_test.py` | **T3 — 짐벌 틸팅.** 프로펠러 없이 지상에서 업링크 전체를 증명하는 가장 값싼 실험 |
| `tools/cmd_vel_test.py` | **T5 — 속도 명령.** 20Hz 연속 발행 + **기수 기준 이동 방향**·거리 자동 측정 |
| `tools/mav.py` | MAVLink v2 최소 구현(공유). CRC 는 앱의 `CRC.java` 에서 직접 뽑았다 |
| `tools/file_bridge.py` | RM500 ↔ PC 파일 다리 (브라우저로 양방향) |
| `logs/` | 8/28 실측 증거 — 논문용 |
| `secrets/keys.xml` | **DJI App Key. 아래 경고 참조** |
| `setup_new_pc.sh` | 새 PC 복구 스크립트 |

## 두 PC 를 오갈 때

```bash
git pull                      # 시작할 때: 저쪽에서 올린 것을 받는다
# ... 작업 ...
git add -A
git commit -m "무엇을 했나"
git push                      # 끝낼 때: 올린다
```

⚠ 옛 README 는 `pack.sh` 로 날짜 붙은 압축을 만들라고 했는데 **그 스크립트는 존재한 적이
없다.** git 으로 옮겨오면서 그 절이 통째로 필요 없어졌으므로 지웠다.

## 🔒 이 저장소를 public 으로 바꾸지 마세요

**DJI App Key 가 평문으로 들어 있고, 커밋 히스토리에도 남아 있습니다.**
`secrets/keys.xml` 뿐 아니라 `진행상황.md` · `진행기록.md` 본문에도 같은 키가 적혀 있고,
첫 커밋 `f529ffd` 안에도 들어 있습니다.

**편의를 위해 일부러 그대로 두기로 했습니다**(2026-09-07 결정) — 비공개 저장소이므로
clone 한 번에 인증키까지 따라와서 새 PC 설정이 명령 한 줄로 끝납니다.
대신 **대가가 하나 있습니다: 이 저장소는 앞으로도 계속 비공개여야 합니다.**

- ❌ GitHub 에서 **Settings → Change visibility → Public** 을 누르지 말 것
- ❌ 이 저장소를 fork 하거나 내용을 공개 저장소에 복사하지 말 것
- 공개해야 할 일이 생기면 **히스토리를 다시 써서 키를 지운 뒤**에만 가능합니다
  (커밋이 적어 `git filter-branch` 로 할 만합니다)

## 새 PC에서 손으로 고쳐야 하는 것

**`tools/` 는 이제 고칠 것이 없습니다.** 예전에는 `GCS_IP = "172.20.10.9"` 가 박혀 있어
핫스팟 IP 가 바뀔 때마다 세 파일을 손대야 했지만, 전부 `0.0.0.0` 으로 bind 하도록 고쳤습니다
(2026-09-09). 이 PC 에 꽂힌 모든 랜선을 다 듣고, 답장 주소는 받은 봉투에서 읽습니다.

**손으로 넣어야 하는 곳은 RM500 앱 한 군데뿐입니다.** 그 값은 이걸로 확인하세요:

```bash
./tools/link_check.sh          # 앱에 넣을 GCS IP + 포트점유 + 방화벽을 한 번에
```

## 지금 어디까지 왔나 (2026-09-09 기준)

- ✅ 빌드 성공 · RM500 설치 · **DJI 라이트 green + `Mavic 2`**
- ✅ **T1 다운링크** — 실기 텔레메트리. 위경도 `0,0` 은 **실내 GPS 미수신이고 앱은 정상**(위성 0개·NO_FIX)
- ✅ **T2 업링크** — `COMMAND_ACK` · `AUTOPILOT_VERSION` 수신
- ✅ **T3 짐벌** — 명령으로 카메라가 실제로 움직임. 기체 하드웨어까지 전 경로 증명
- ✅ **T5 cmd_vel (SIM)** — 축 대응·부호 확정. `vx>0`=앞 · `vy>0`=오른쪽 · **`vz>0`=위(MAVLink 와 반대)** · `yawRate>0`=시계방향
- ⬜ **T4 영상** — 켜면 앱이 죽는다(함정 5). **원인 규명 끝, 패치도 있음**(`patches/01`). NDK 재빌드만 남았다
- ⬜ 야외 GPS fix 확인
- ⬜ `markWaypoint()` NPE 지뢰 미패치 (`MainActivity.java:511,528`) — 앱 화면에서 웨이포인트를 찍을 때만 터진다. MAVLink 경로는 무관

⚠ **`COMMAND_ACK: ACCEPTED` 를 성공으로 읽지 마세요.** 앱은 DJI 가 에러를 돌려줘도
무조건 ACCEPTED 를 보냅니다(`DroneModel.java:2029`). 상태 변화로 검증해야 합니다.

앞으로 할 일은 **`할일.md`** 에 있습니다.

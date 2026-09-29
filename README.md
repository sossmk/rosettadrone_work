# RosettaDrone 작업 씨앗 (drone_ws seed)

> **2026-09-07 부터 GitHub 비공개 저장소가 정본입니다** —
> `https://github.com/sossmk/rosettadrone_work`.
> 어느 쪽이 최신인지 날짜로 판단하던 방식은 끝났다. `git pull` 하면 기계가 판정한다.

**이 폴더 하나면 새 PC에서 작업을 그대로 이어갈 수 있습니다.** 약 170KB 입니다.

## 새 PC에서 처음부터 — 이 순서대로만 하면 됩니다

> 2026-09-10 에 실제로 이 경로를 밟아 **T4 영상까지 통과**했습니다.
> 순서를 바꾸거나 건너뛰면 그날 우리가 밟은 함정을 그대로 밟습니다.

### 1. 기록·도구 받기 (2분)

```bash
# gh 가 없으면 먼저: sudo apt install -y gh && gh auth login && gh auth setup-git
git clone https://github.com/sossmk/rosettadrone_work.git ~/rosettadrone_work
cd ~/rosettadrone_work
```

기록·도구·인증키가 전부 따라옵니다. **PC 쪽 도구(`tools/`)만 쓸 거면 여기서 끝입니다.**

### 2. 툴체인 + 빌드 (15~40분 · 2.7GB)

```bash
bash ~/rosettadrone_work/setup_new_pc.sh
```

JDK 11 · Android SDK · NDK · 소스(커밋 고정) · **패치 4개 적용** · 빌드 · QGC 까지 합니다.
끝나면 `~/drone_ws/rosettadrone/app/build/outputs/apk/debug/app-debug.apk` (188MB).

⚠ **패치 없이 빌드하면 영상이 동작하지 않습니다.** 원본 소스는
`AndroidManifest.xml` 에서 `VideoService` 가 주석 처리돼 있습니다. 스크립트가
자동으로 붙이고, **붙지 않으면 빌드 전에 멈춥니다.**

스크립트가 끝에 이 둘을 확인해 줍니다 — **둘 다 ✓ 여야 합니다**:
```
✓ Manifest 에 VideoService 선언 확인 (patches/02 반영됨)
✓ .so 에 dataCallbackMID 널 가드 확인 (patches/04 반영됨)
```

### 3. RM500 에 설치

```bash
mkdir -p ~/drone_ws/apk
cp ~/drone_ws/rosettadrone/app/build/outputs/apk/debug/app-debug.apk ~/drone_ws/apk/
python3 -m http.server 8000 -d ~/drone_ws/apk
# RM500 브라우저로 http://<이 PC IP>:8000/ → apk 를 누르면 설치 관리자가 뜬다
```

`adb install` 이 이 기기에서 불안정해서(9/10) HTTP 로 옮긴다. MTP 로 복사해도 파일 관리자가 apk 를 필터로 숨겨서 안 된다.
RM500 → PC 로 파일을 올려야 하면 `git show v1-2026-09-16:tools/file_bridge.py > /tmp/file_bridge.py`

⚠ **다른 PC 에서 빌드한 앱이 이미 깔려 있으면 덮어쓰기가 안 됩니다.**
디버그 서명이 PC마다 달라서 *"같은 이름을 가진 패키지가 있어서 설치가 안 된다"* 가 뜹니다.
→ **기존 앱을 제거하고 새로 설치하세요.**

### 4. ★ 앱 설정 — 여기서 제일 많이 막힙니다

**앱을 새로 깔면 설정이 전부 초기화됩니다.** 2026-09-10 에 여기서 20분을 썼습니다.

```bash
./tools/link_check.sh          # 앱에 넣을 GCS IP 를 찍어 준다
```

RM500 앱 설정에 **전부** 넣으세요:

| 항목 | 값 |
|---|---|
| **`Use GCS on an external device`** | **켜기** ← 제일 잘 빠뜨립니다 |
| `GCS IP` | `link_check.sh` 가 찍어 준 값 (매번 바뀝니다) |
| `Port` | `14550` |
| **`Enable video feed`** | **켜기** |
| `Video port` | `5600` |
| `Video bitrate` | `2` |
| RC 모드 스위치 | **P** (cmd_vel 은 P 에서만 먹습니다) |

그다음 **강제 중지 → 앱 열기 → `START`**(SIM·TEST 아님).

⚠ **`Use GCS on an external device` 가 꺼져 있거나 IP 가 틀리면 앱은 크래시도 안 하고
조용히 아무것도 안 보냅니다.** 크래시와 구별이 안 됩니다 — 구별법은 아래.

### 5. 확인

```bash
./tools/link_check.sh                  # T0 링크 위생
ros2 launch rd_bridge bridge.launch.py nav:=false    # 브리지 (다른 터미널)
ros2 topic echo /gcs/daiv/rd/battery            # T1 다운링크 + 배터리 전압
ros2 topic hz /gcs/daiv/camera_info    # T4 영상 (publish_fps, 기본 3Hz — 영상과 1:1). 안 나오면 video_node.py docstring 의 gst-launch 로 창부터
```

**정상이면 이렇게 나옵니다** (2026-09-10 실측):
```
배터리  16.98 V   전류 13.05 A   잔량 91%
✓ T4 통과 — 1632x1088 · IDR 31장 · 2514 kbps
```

### 막혔을 때 — 증상으로 원인 가르기

| 증상 | 먼저 의심할 것 |
|---|---|
| **텔레메트리가 처음부터 0패킷** | **설정.** `Use GCS…` 토글 · IP 오타. 크래시는 보통 몇 초 보내다 끊깁니다 |
| 몇 초 보내다 끊긴다 | **크래시.** 툼스톤을 읽으세요(아래) |
| 텔레메트리는 오는데 영상만 0 | `Enable video feed` · Video port · `patches/02` 반영 여부 |
| 좌표가 `-32.85, -71.21` | **TEST 모드.** 강제 중지 후 START |
| 좌표가 `0, 0` · 위성 0개 | **정상.** 실내 GPS 미수신입니다. 야외로 |

**크래시 원문 읽는 법** (USB 로 됩니다 — 옛 기록의 "USB 가 안 통한다"는 틀렸습니다):

RM500 의 USB 옵션을 **`Transfer files`(MTP)** 로 두고 케이블을 꽂으면
`/run/user/1000/gvfs/mtp:host=DJI_rm500_*/Internal shared storage/dji_logs/` 가 열립니다.

| 경로 | 내용 |
|---|---|
| `tombstones/` | 네이티브 크래시 — `signal` · `fault addr` · backtrace |
| `logs/<날짜>/android` | logcat 전문 (28~40MB) |

⚠ 툼스톤이 언제 것인지는 **파일 날짜가 아니라 `pid`/`tid` 를 logcat 과 맞춰** 확인하세요.
2026-09-10 에 파일 날짜를 믿고 오판했습니다.

⚠ `Transfer photos`(PTP)로 바꾸면 "Folder is Empty" 가 뜹니다 — 정상입니다. MTP 를 쓰세요.

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
| `진행기록-2026-08-28_09-14.md` | 9/14 이전 날짜 블록 (진행상황.md 에서 옮김) |
| `가이드북.md` | **혼자 운용하기** — 현장 루틴 · 런치 조합 · 대형/파라미터 바꾸기 · 코드 지도 · 로그 읽기 · 문제 해결 |
| `할일.md` | **앞으로 할 것.** AR 태그 추종·정밀 착륙 패키지의 구조와 순서, 지고 갈 제약 |
| `patches/` | **앱 소스 패치 4개.** `02` 가 영상을 살리는 핵심 (Manifest 의 `VideoService` 주석 복구) |
| `HTML_for_study/` | **브라우저로 여는 설명 문서 모음.** `구조.html` 이 전체 구조 한 장 요약(층·데이터 흐름·토픽·언제 어떤 도구를 쓰나) · `공부-*.html` 노드별 해설 · `architecture-ros2.html`·`workflow-usv-experiment.html` 다이어그램 |
| `tools/link_check.sh` | **기체 켜기 전 링크 위생.** IP·포트점유·방화벽을 한 번에 보고, **앱에 넣을 GCS IP 를 찍어 준다** |
| `tools/rd_cmd.py` | **업링크 명령 도구.** `ping`(업링크 열렸나) · `gimbal`(짐벌 틸트) · `takeoff`(이륙) · `drive`(방향키 조종) · `follow`(태그 추종 시작·튜닝) · `mission`(경유점 비행). ⚠ 조종권은 저절로 RC 로 안 돌아온다 — S 모드 전환(9/11 실기) |
| `tools/tag_check.py` | **태그가 왜 안 잡히나** — 사진·영상에서 태그를 찾아 이유를 말해 준다 |
| `토픽.md` | **토픽 한 장** — 형식·주기·단위·좌표계. 주기 확인은 `ros2 topic hz` |
| `missions/` | 경유점 파일 (`rd_cmd.py mission`) |
| `tools/gps_capture.py` | **경로 기록** — RC 로 날며 Enter 마다 좌표를 `missions/*.txt` 에 쌓는다 (`rd_cmd.py mission` 으로 날림) |
| `ros2/` | **ROS 2 브리지 + 항법 (`rd_bridge`).** 영상 → `/gcs/daiv/cam/compressed` · 다운링크 → `/gcs/daiv/rd/*` · `/gcs/daiv/cmd_vel` → #84 · **`nav_node` = 유도 루프(우리 것, `flyTo()` 대체)** · USV 추종 · 임무 시퀀서 — `ros2/README.md` |
| `tools/mav.py` | MAVLink v2 최소 구현(공유). CRC 는 앱의 `CRC.java` 에서 직접 뽑았다 |
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
`secrets/keys.xml` 뿐 아니라 `진행상황.md` 본문과 (지운) `진행기록.md` 의 히스토리에도 같은 키가 있고,
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

## 지금 어디까지 왔나

**`진행상황.md` 맨 위 「현재 상태」** 를 보세요. 앞으로 할 일도 거기 「다음에 할 일」에 있습니다.
(9/14 기준으로 여기 적어 두었던 요약은 `진행기록-2026-08-28_09-14.md` 로 옮겼습니다.)

⚠ **`COMMAND_ACK: ACCEPTED` 를 성공으로 읽지 마세요.** 앱은 DJI 가 에러를 돌려줘도
무조건 ACCEPTED 를 보냅니다(`DroneModel.java:2029`). 상태 변화로 검증해야 합니다.

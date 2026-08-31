# RosettaDrone 작업 씨앗 (drone_ws seed)

**이 폴더 하나면 새 PC에서 작업을 그대로 이어갈 수 있습니다.** 152KB 입니다.

## 새 PC에서 할 일 (한 줄)

```bash
bash setup_new_pc.sh
```

JDK 11 · Android SDK 5종 · 소스(커밋 고정) · 인증키 · 빌드 · QGC 까지 자동으로 되살립니다.
20~40분 걸리고, 끝나면 188MB APK가 나옵니다.

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
| `진행기록.md` | 명령·에러 원문 |
| `rosettadrone_progress.html` | 보고용 요약 |
| `tools/udp_probe.py` | UDP 14550 에 뭐가 오는지 센다 (앱↔방화벽↔QGC 원인 분리) |
| `tools/decode_check.py` | MAVLink 를 풀어 실제 값을 본다 (**가짜/진짜 좌표 판별**) |
| `tools/uplink_test.py` | PC→앱 명령 경로가 열렸는지 본다 (아직 미실행) |
| `logs/` | 8/28 실측 증거 — 논문용 |
| `secrets/keys.xml` | **DJI App Key. 아래 경고 참조** |
| `setup_new_pc.sh` | 새 PC 복구 스크립트 |

## ⚠ secrets/keys.xml — 공개 저장소에 올리지 마세요

DJI App Key 가 평문으로 들어 있습니다. GitHub 에 올린다면 **반드시 Private** 로 하세요.
(참고: `진행상황.md` 본문에도 같은 키가 적혀 있습니다.)

## 새 PC에서 손으로 고쳐야 하는 것

`tools/*.py` 의 `GCS_IP = "172.20.10.9"` 는 옛 핫스팟 IP 입니다.
새 PC 의 IP 로 바꾸세요: `ip -4 addr | grep inet`

## 지금 어디까지 왔나 (2026-08-28 기준)

- ✅ 빌드 성공 · RM500 설치 · **DJI 라이트 green + `Mavic 2`** (kill-switch 2 통과)
- ✅ 다운링크 텔레메트리 수신 (초당 ~56패킷, 12종)
- ⬜ **업링크(PC→앱) 미확인** ← `uplink_test.py` 를 아직 안 돌렸습니다. **다음 할 일**
- ⬜ `markWaypoint()` NPE 지뢰 미패치 (`MainActivity.java:511,528`)

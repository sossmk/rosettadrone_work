# 시선 유지 성능 분석 (DT0503)

논문지 2.3.4절 "시선 유지를 위한 기수각 제어"를 실험으로 평가하기 위한 작업 폴더.
목표: USV가 관제 영상의 **화면 중앙**에 얼마나 잘 유지됐는지 수치로 보이기 (논문 3.2.x 새 절, 약 1/2쪽).

## 분석 방법 (두 갈래)

| | (A) bag 기반 기수각 오차 | (B) SAM2 영상 판독 |
|---|---|---|
| 입력 | 무인기 위치·기수각 + USV 위치 | bag의 JPEG 프레임 |
| 계산 | 식(3) 목표 기수각 ψ_d − 실제 기수각 ψ = e_ψ | USV 마스크 중심 − 화면 중앙(408, 272) = (dx, dy) |
| 의미 | 제어기가 판단한 오차 (GNSS가 정확하다는 가정) | 실제 영상에서 보인 오차 |

- (A) ≈ (B) → 오차 원인은 제어 성능
- (A) 작고 (B) 큼 → GNSS 위치 오차나 시간 지연
- **실제 결과는 셋째 경우였다**: (A) 작고 (B) 큰데 원인은 GNSS도 지연도 아니라
  **짐벌**이다 (yaw 지연 0.9 s + 틸트 42°에서의 roll 누출). `분석기록.md` 5절
- 픽셀 ↔ 각도 환산: `camera_info` 내부 파라미터 사용 (없으면 수평 화각으로 근사, 816 px에서 5° ≈ 45 px)

## 필요한 데이터 (Git에 넣지 말 것)

다른 PC로 옮길 때 USB나 클라우드로 복사해서 `data/` 아래에 둔다.

| 무엇 | 원래 위치 (이 PC) | 크기 | 넣을 곳 |
|---|---|---|---|
| 무인기 bag DT0503 | `한국해양로봇기술연구회/논문_실험자료/log관련/fig/2026-10-02/DT05/03/bag/` (폴더 통째로) | 약 68 MB | `data/drone_DT0503/` |
| USV bag DT0503 | `한국해양로봇기술연구회/논문_실험자료/DT05/03.zip` (압축 해제) | 27 MB (풀면 133 MB) | `data/usv_DT0503/` |

> 이 PC(리눅스, RTX 5080)에는 이미 있다 — `data/drone_DT0503`는 `~/usv_log/DT05/03/bag`로 심볼릭 링크,
> `data/usv_DT0503`는 `~/usv_log/03.zip`을 푼 것이다.

- 영상은 쓰지 않는다. 분석은 bag JPEG만으로 한다. 직접 녹화한 `org_fe045…mp4`는 촬영 시각이 14:46~15:01이라 DT0503이 아니라 ZZ0201 비행이다.

- 무인기 bag 토픽: `/gcs/daiv/cam/compressed`(JPEG 816×544, 956장), `/gcs/daiv/camera_info`, `/gcs/daiv/rd/fix`, `/gcs/daiv/rd/heading`, `/gcs/daiv/rd/attitude`, `/gcs/daiv/usv/follow`(추종점 변경 시점)
- USV bag 토픽: `/maiv/hemisphere_gps/navsatfix`, `/maiv/fog/euler_orientation`, `/maiv/fog/ned_vel`
- 기록 시간: 무인기 11:00:24~11:08:56, USV 11:00:21~11:08:53 (2026-10-02 KST)
- 시간 동기화: 9/30 이후 bag은 시각 동기화됨. USV→관제소 시각 차는 약 0.205 s(인수인계 문서)

## 다른 PC에서 환경 설치

조건: Python 3.12, NVIDIA GPU. 다운로드 약 5 GB (와이파이에서 할 것).

```powershell
cd 분석_시선유지
powershell -ExecutionPolicy Bypass -File setup.ps1            # RTX 50 시리즈 (CUDA 12.8)
powershell -ExecutionPolicy Bypass -File setup.ps1 -Cuda cu124  # 그 이전 GPU
```

```bash
bash setup.sh          # 리눅스 — 같은 내용. 그 이전 GPU면 bash setup.sh cu124
```

하는 일: `.venv` 생성 → PyTorch(GPU) → `requirements.txt`(SAM2 공식, rosbags, OpenCV 등) → SAM2.1 small 가중치(184 MB)를 `checkpoints/`에 받기 → GPU 인식 확인.
지울 때는 `.venv/`와 `checkpoints/` 폴더만 삭제하면 된다.

그림의 한글 라벨은 `scripts/hangul_font.py`가 **설치된 폰트 중에서 알아서 고른다**
(리눅스 Noto/나눔 → 윈도우 맑은 고딕 → macOS 애플고딕). 하나도 없으면 깔라는 말을 찍고
그림은 그대로 나온다 (한글만 네모). 리눅스는 `sudo apt install fonts-noto-cjk`.

## 작업 순서

**2026-11-17 전부 완료.** 무엇을 어떤 근거로 정했는지는 [`분석기록.md`](분석기록.md)에 있다.
다른 PC로 결과만 옮겨 보려면 [`옮기기.md`](옮기기.md).

1. [x] 환경 설치 (리눅스는 `setup.sh`)
2. [x] bag 추출 → `outputs/`에 CSV (`scripts/extract_bags.py`)
   - 프레임별: 시각, 파일명 / 무인기: 시각, 위경도, 기수각, 짐벌 / USV: 시각, 위경도, 기수각
   - 추종점 구간: **선미 하나뿐이고 구간 내 전환이 없다** (기하로 판정 — `분석기록.md` 2절)
3. [x] (A) e_ψ 시계열, 구간별 RMS·최대값 (`scripts/analyze_a.py`)
   - 대형 전환이 없어 앞먹임 효과의 근거로는 못 쓴다
4. [x] (B) SAM2 추적 (`scripts/track_b.py`) — 박스 한 번으로 511장 전부, 재지정 불필요
5. [x] 검수: `outputs/b_overlay.mp4`를 눈으로 확인. 틀린 프레임 없음
6. [x] (A)·(B) 비교 그래프 `outputs/fig_gaze.png` + 수치 `outputs/compare_stats.txt`

## 영상만 있을 때 (LOG 없음) — 화면 중심에서 얼마나 벗어났나만 잰다

bag 이 없으면 (A) 는 못 낸다. 영상에서 **USV 중심 − 화면 중심**만 재는 길이다.
구간은 사람이 정한다.

```bash
# 0) 영상 길이·해상도만 먼저 본다
env -u PYTHONPATH .venv/bin/python scripts/video_frames.py --video 내영상.mp4 --probe

# 1) 구간을 잘라 프레임으로 푼다 (--from · --to 는 "분:초" 또는 초)
env -u PYTHONPATH .venv/bin/python scripts/video_frames.py \
    --video 내영상.mp4 --out outputs_vid --from 2:10 --to 5:30 --fps 5

# 2) 첫 프레임에서 USV 자리를 눈으로 읽는다 → outputs_vid/pick_box.png
env -u PYTHONPATH .venv/bin/python scripts/pick_box.py --out outputs_vid

# 3) 추적 — 여기서 나오는 b_track.csv 가 재려던 값이다
env -u PYTHONPATH .venv/bin/python scripts/track_b.py --out outputs_vid \
    --box x1,y1,x2,y2 --overlay

# 4) (선택) 요약 표와 그림
env -u PYTHONPATH .venv/bin/python scripts/report_b.py --out outputs_vid
```

`outputs_vid/b_track.csv`

| 열 | 뜻 |
|---|---|
| `idx`, `t` | 프레임 번호, 영상 시작부터의 초 |
| `cx`, `cy` | USV 마스크 중심 (px) |
| `dx`, `dy` | **USV 중심 − 화면 중심** (+ 는 오른쪽·아래) |
| `dist` | **그 둘 사이 거리 (px)** |
| `area`, `ok` | 마스크 넓이, 못 찾았으면 `ok=0` |

- 구간은 1번에서 잘라도 되고, 이미 푼 프레임에서 `track_b.py --from 1:30 --to 4:00`으로 다시 잘라도 된다.
- 픽셀을 각도로 바꾸려면 화각이 필요하다. `--hfov`(기본 77°, Mavic 2 Pro 사양) 또는 `--fx`로 준다.
  **화각이 틀려도 픽셀 값은 영향 없다.**
- 추적이 틀어지면 `b_overlay.mp4`에서 그 프레임을 찾아 `--refine 번호:x1,y1,x2,y2`.
- USV가 화면 밖으로 나가면 그 프레임은 `ok=0`으로 남는다 (실패가 아니라 없다는 표시).

## 다른 비행으로 같은 분석 하기 (LOG 있음)

스크립트는 전부 인자를 받는다. DT0503 전용 값이 박혀 있지 않다.

```bash
# 1) bag 두 개를 data/ 아래에 둔다 (폴더 이름은 아무거나)
# 2) 추출 — 결과 폴더를 따로 쓴다
env -u PYTHONPATH .venv/bin/python scripts/extract_bags.py \
    --drone data/drone_XX --usv data/usv_XX --out outputs_XX --frames

# 3) (A) — 분석 구간은 /gcs/daiv/usv/follow 가 true 인 동안을 스스로 찾는다
env -u PYTHONPATH .venv/bin/python scripts/analyze_a.py --out outputs_XX

# 4) 첫 프레임에서 USV 자리를 눈으로 읽는다 → outputs_XX/pick_box.png
env -u PYTHONPATH .venv/bin/python scripts/pick_box.py --out outputs_XX

# 5) (B) — 읽은 박스를 준다. 구간은 (A)가 정한 것을 그대로 따른다
env -u PYTHONPATH .venv/bin/python scripts/track_b.py --out outputs_XX \
    --box x1,y1,x2,y2 --overlay

# 6) 비교
env -u PYTHONPATH .venv/bin/python scripts/compare.py --out outputs_XX
```

**사람이 해야 하는 건 4번 하나다.** 나머지는 자동이다.

손볼 데가 생기는 경우

| 증상 | 어떻게 |
|---|---|
| 추종 토픽이 다르다 (예: 태그 추종) | `analyze_a.py --topic /gcs/daiv/tag/follow` |
| 구간을 직접 정하고 싶다 | `analyze_a.py --window t0,t1` (UNIX 초) |
| 중간에 추적이 엉뚱한 데로 간다 | `b_overlay.mp4`에서 틀어진 프레임 번호를 찾아 → `pick_box.py 번호`로 다시 읽고 → `track_b.py --refine 번호:x1,y1,x2,y2` (여러 번 가능) |
| USV 말고 다른 배가 잡힌다 | 박스를 더 좁게. 두 척이 겹쳐 보이는 프레임은 피해서 시작 프레임을 고른다 |
| 대형 추정이 이상하다 | `a_stats.txt`의 "대형 추정" 줄은 기하로 판정한 것이다. USV 기수 보정값이 그 비행에 맞는지 확인 |

주의: `compare.py`의 짐벌 모형(yaw 지연·roll 누출)은 비행마다 **다시 맞춘다**. DT0503에서 나온
0.9 s와 0.74는 그 비행의 값이지 상수가 아니다.

## Git 사용 시

- **코드와 이 README만** 올린다. `.gitignore`에 데이터·가상환경·가중치·결과물이 제외되어 있음.

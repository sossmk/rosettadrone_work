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
- 픽셀 ↔ 각도 환산: `camera_info` 내부 파라미터 사용 (없으면 수평 화각으로 근사, 816 px에서 5° ≈ 45 px)

## 필요한 데이터 (Git에 넣지 말 것)

다른 PC로 옮길 때 USB나 클라우드로 복사해서 `data/` 아래에 둔다.

| 무엇 | 원래 위치 (이 PC) | 크기 | 넣을 곳 |
|---|---|---|---|
| 무인기 bag DT0503 | `논문지_자료/maiv_ws/maiv_ws/src/maiv_rosbag_logger/log/2026-10-02/DT05/03/bag/` | 약 71 MB | `data/drone_DT0503/` |
| USV bag DT0503 | `논문지_자료/03/03/bag/` | 약 140 MB | `data/usv_DT0503/` |
| (선택) 직접 녹화 영상 | 사용자 보관 | — | `data/video/` |

- 무인기 bag 토픽: `/gcs/daiv/cam/compressed`(JPEG 816×544, 956장), `/gcs/daiv/camera_info`, `/gcs/daiv/rd/fix`, `/gcs/daiv/rd/heading`, `/gcs/daiv/rd/attitude`, `/gcs/daiv/usv/follow`(추종점 변경 시점)
- USV bag 토픽: `/maiv/hemisphere_gps/navsatfix`, `/maiv/fog/euler_orientation`, `/maiv/fog/ned_vel`
- 기록 시간: 무인기 11:00:24~11:08:56, USV 11:00:21~11:08:53 (2026-10-02 KST)
- 시간 동기화: 9/30 이후 bag은 시각 동기화됨. USV→관제소 시각 차는 약 0.205 s(인수인계 문서)

## 다른 PC에서 환경 설치

조건: Windows, Python 3.12, NVIDIA GPU. 다운로드 약 3~4 GB (와이파이에서 할 것).

```powershell
cd 분석_시선유지
powershell -ExecutionPolicy Bypass -File setup.ps1            # RTX 50 시리즈 (CUDA 12.8)
powershell -ExecutionPolicy Bypass -File setup.ps1 -Cuda cu124  # 그 이전 GPU
```

`setup.ps1`이 하는 일: `.venv` 생성 → PyTorch(GPU) → `requirements.txt`(SAM2 공식, rosbags, OpenCV 등) → SAM2.1 small 가중치(184 MB)를 `checkpoints/`에 받기 → GPU 인식 확인.
지울 때는 `.venv/`와 `checkpoints/` 폴더만 삭제하면 된다.

## 작업 순서

1. [ ] 환경 설치 (`setup.ps1`)
2. [ ] bag 추출 → `outputs/`에 CSV
   - 프레임별: 시각, 파일명 / 무인기: 시각, 위경도, 기수각, 짐벌 / USV: 시각, 위경도, 기수각
   - 추종점(좌현·우현·선미) 구간 표시
3. [ ] (A) e_ψ 시계열, 구간별 RMS·최대값 (특히 대형 전환 구간 → 앞먹임 효과 근거)
4. [ ] (B) SAM2 추적: 첫 프레임에 USV 박스 지정 → 전체 프레임 전파 → 놓치면 그 지점에서 박스 재지정
5. [ ] 검수: 추적 결과를 겹친 확인용 영상을 눈으로 보고 틀린 프레임 기록
6. [ ] (A)·(B) 비교 그래프 1장 + 수치 → 논문 3.2.x

## Git 사용 시

- **코드와 이 README만** 올린다. `.gitignore`에 데이터·가상환경·가중치·결과물이 제외되어 있음.

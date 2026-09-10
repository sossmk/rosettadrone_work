#!/usr/bin/env bash
# 영상을 눈으로 본다. 두 가지 모드.
#
#   ./tools/video_view.sh              # 실시간 — 기체에서 오는 것을 창으로 띄운다
#   ./tools/video_view.sh 파일.h264    # 저장된 파일을 재생한다 (드론 없이)
#
# 창을 닫거나 Ctrl+C 로 끝낸다.
#
# 실시간 경로에 h264parse(gstreamer1.0-plugins-bad)는 필요 없다 — 없이도 된다.
# 저장 파일 재생은 OpenCV 로 한다(자체 디코더를 갖고 있어 추가 설치가 필요 없다).
set -u
PORT="${PORT:-5600}"

# ---------------------------------------------------------------- 저장 파일
if [ $# -ge 1 ]; then
  exec python3 - "$1" <<'PLAYFILE'
import sys, cv2
p = sys.argv[1]
cap = cv2.VideoCapture(p)
if not cap.isOpened():
    sys.exit(f"열 수 없다: {p}")
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
delay = max(1, int(1000 / (fps if fps > 1 else 30)))
print(f"재생: {p}   (아무 키나 누르면 종료)")
n = 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    n += 1
    cv2.imshow("RosettaDrone - 저장된 영상", frame)
    if cv2.waitKey(delay) != -1:
        break
cap.release()
cv2.destroyAllWindows()
print(f"{n} 프레임 재생함")
PLAYFILE
fi

# ---------------------------------------------------------------- 실시간
command -v gst-launch-1.0 >/dev/null || { echo "gst-launch-1.0 가 없다"; exit 1; }

# 창을 띄우기 전에 3초만 들어 본다.
# 아무것도 안 오면 gst 는 조용히 기다리기만 해서 "왜 안 나오지" 로 시간을 버린다.
# 2026-09-10 에 IP 가 어긋나서 세 번 겪었다. 그래서 먼저 판정하고, 앱이 엉뚱한
# 주소로 쏘고 있으면 그 주소에 누가 있는지까지 보여 준다.
echo "UDP $PORT 를 3초간 확인하는 중…"
if python3 - "$PORT" <<'LISTEN'
import socket, sys, time
port = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.bind(("0.0.0.0", port))
except OSError as e:
    print(f"  포트 {port} 를 못 잡는다: {e}")
    sys.exit(1)
s.settimeout(0.4)
n, t0, src = 0, time.time(), None
while time.time() - t0 < 3:
    try:
        _, a = s.recvfrom(65535)
        n += 1
        src = a[0]
    except socket.timeout:
        pass
print(f"  {n}패킷" + (f"  (보낸 곳 {src})" if src else ""))
sys.exit(0 if n else 1)
LISTEN
then
  echo "영상이 온다. 창을 띄운다 — 창을 닫거나 Ctrl+C 로 끝낸다."
  exec gst-launch-1.0 -q \
    udpsrc port="$PORT" \
      caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96" \
    ! rtpjitterbuffer latency=120 drop-on-latency=true \
    ! rtph264depay \
    ! avdec_h264 output-corrupt=false \
    ! videoconvert \
    ! autovideosink sync=false
fi

# ---------------------------------------------------------------- 안 올 때
echo
echo "  ✗ 영상이 안 온다. 아래를 확인할 것."
echo
echo "  ① 이 PC 의 IP — 앱의 GCS IP 가 이 중 하나여야 한다:"
ip -4 -brief addr | awk '$2=="UP" {split($3,a,"/"); printf "       %-16s (%s)\n", a[1], $1}'
echo "     핫스팟은 접속할 때마다 다른 주소를 준다. 어제 값을 그대로 쓰면 안 된다."
echo
echo "  ② 같은 망의 다른 기기 — 앱이 이쪽으로 쏘고 있을 수 있다:"
ip neigh 2>/dev/null \
  | awk '$1 ~ /^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$/ && $NF != "FAILED" && $5 != "" {
           printf "       %-16s %s%s\n", $1, $5, (index($5,"34:d2:62")==1 ? "   ← RM500" : "") }' \
  | sort -u
echo
echo "  ③ 앱 설정:  Use GCS on an external device 켜기 · Enable video feed 켜기 · Video port $PORT"
echo "  ④ 앱이 START 상태인가 (SIM·TEST 아님)"
echo
echo "  더 자세히:  python3 tools/video_probe.py --secs 10"
exit 1

#!/usr/bin/env bash
# T0 — 링크 위생 점검. 기체를 켜기 전에 돌린다.
#
# 왜: 이 구간의 실패는 넷 다 화면상 "아무것도 안 옴" 으로 똑같이 보인다.
#     (앱이 안 쏨 / 방화벽 / 포트 점유 / IP 불일치)
#     미리 지워 두지 않으면 T1·T2 의 결론을 믿을 수 없다.
set -u
PORT="${1:-14550}"
VPORT="${2:-5600}"

echo "=== T0 링크 위생 ==============================================="

# ① 이 PC 의 주소 — 이 중 하나를 RM500 앱의 GCS IP 에 넣는다
echo
echo "① 이 PC 의 IPv4 주소"
ip -4 -o addr show scope global | awk '{printf "     %-14s %s\n", $2, $4}'

HOTSPOT=$(ip -4 -o addr show scope global \
          | awk '$2 ~ /^(wl|wlan|enx|usb)/ {split($4,a,"/"); print a[1]; exit}')
WIRED=$(ip -4 -o addr show scope global \
        | awk '$2 ~ /^(en|eth)/ && $2 !~ /^enx/ {split($4,a,"/"); print a[1]; exit}')

echo
if [ -n "$HOTSPOT" ]; then
  echo "  ➜ 앱(RM500)의 GCS IP 에 넣을 값:  $HOTSPOT"
  echo "    (무선/테더링 인터페이스. RM500 과 같은 망에 있어야 한다)"
elif [ -n "$WIRED" ]; then
  echo "  ⚠ 무선/테더링 주소가 없다. 유선만 있음: $WIRED"
  echo "    RM500 이 이 유선망에 붙어 있지 않다면 통신이 안 된다."
  echo "    → 휴대폰 핫스팟을 켜고 이 PC 를 붙인 뒤 다시 실행할 것"
else
  echo "  ✗ 쓸 수 있는 주소가 없다. 네트워크부터 연결할 것"
fi

# ② 포트를 누가 잡고 있나 — udp_probe 와 QGC 를 동시에 켜면 여기서 걸린다
echo
echo "② UDP 포트 점유 (텔레메트리 $PORT · 영상 $VPORT)"
BUSY=$(ss -lunp 2>/dev/null | grep -E ":($PORT|$VPORT)\b")
if [ -z "$BUSY" ]; then
  echo "     비어 있음 ✓"
else
  echo "$BUSY" | sed 's/^/     /'
  echo "  ⚠ 이미 누가 잡고 있다. QGC 와 udp_probe.py 는 동시에 못 쓴다."
  echo "    나중에 켠 쪽이 조용히 실패하고, 그게 '드론이 안 보낸다' 처럼 보인다."
fi

# ③ 방화벽 — is-active 만 보면 틀린다(2026-08-28 에 실제로 잘못 짚었음)
echo
echo "③ 방화벽(ufw)"
if [ -r /etc/ufw/ufw.conf ]; then
  EN=$(grep -E '^ENABLED=' /etc/ufw/ufw.conf | cut -d= -f2)
  if [ "$EN" = "no" ]; then
    echo "     ENABLED=no → 규칙이 적용되지 않음. 막지 않는다 ✓"
  else
    echo "     ENABLED=$EN → 규칙이 산다. 아래를 확인/추가할 것:"
    echo "       sudo ufw status verbose"
    echo "       sudo ufw allow $PORT/udp  comment 'MAVLink RosettaDrone'"
    echo "       sudo ufw allow $VPORT/udp comment 'RosettaDrone video'"
  fi
else
  echo "     /etc/ufw/ufw.conf 없음 — ufw 미설치로 보인다 ✓"
fi

# ④ 다음 단계 안내
echo
echo "=== 다음 ======================================================="
echo "  ④ RM500 앱에서 눈으로 확인:"
echo "       Use GCS on an external device : 켜기"
echo "       GCS IP   : ${HOTSPOT:-<위 ① 의 무선 주소>}"
echo "       Port     : $PORT"
echo "       RC 모드 스위치 : P   (cmd_vel 은 P 모드에서만 먹는다)"
echo
echo "  T1 다운링크 :  python3 tools/decode_check.py $PORT"
echo "  T2 업링크   :  python3 tools/uplink_test.py  $PORT"
echo "  (둘 다 $PORT 를 잡는다. 동시에 돌리지 말 것)"

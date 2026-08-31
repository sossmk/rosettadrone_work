#!/usr/bin/env python3
"""PC → 앱 방향(명령)이 열려 있는지 본다. 기체를 움직이는 명령은 안 보낸다.

보내는 것: HEARTBEAT(GCS 존재 알림) + MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES(520)
받기를 기대하는 것: COMMAND_ACK(77) · AUTOPILOT_VERSION(148)

핵심: 앱 소켓은 socket.connect(GCS_IP, 14550) 상태다.
     → 보내는 쪽도 반드시 14550 을 써야 앱이 받는다(아니면 조용히 버려진다).
"""
import socket, struct, time, sys

GCS_IP, GCS_PORT = "172.20.10.9", 14550
SYSID, COMPID = 255, 190          # GCS 로서의 정체

def crc16_mcrf4xx(data, crc=0xFFFF):
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

_seq = 0
def frame(msgid, payload, crc_extra):
    """MAVLink v2 한 장을 만든다."""
    global _seq
    payload = payload.rstrip(b"\x00") or b"\x00"        # v2 truncation
    hdr = struct.pack("<BBBBBB", len(payload), 0, 0, _seq, SYSID, COMPID)
    hdr += struct.pack("<I", msgid)[:3]
    _seq = (_seq + 1) % 256
    c = crc16_mcrf4xx(hdr + payload)
    c = crc16_mcrf4xx(bytes([crc_extra]), c)
    return b"\xFD" + hdr + payload + struct.pack("<H", c)

def heartbeat():
    # custom_mode, type=6(GCS), autopilot=8(INVALID), base_mode, status=4(ACTIVE), ver=3
    return frame(0, struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3), 50)

def command_long(cmd, p1=0.0):
    p = struct.pack("<7f", p1, 0, 0, 0, 0, 0, 0)
    p += struct.pack("<HBBB", cmd, 0, 0, 0)   # target_system=0 → "아무나 실행" (sysid 불일치 회피)
    return frame(76, p, 152)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((GCS_IP, GCS_PORT))
s.settimeout(1.0)

# 1) 앱이 어느 포트에서 쏘는지 알아낸다 (앱을 껐다 켜면 매번 바뀐다)
print(f"[1] 앱의 송신 포트를 찾는 중… (14550 에서 수신)")
app_addr = None
t0 = time.time()
while time.time() - t0 < 6:
    try:
        d, addr = s.recvfrom(4096)
    except socket.timeout:
        continue
    app_addr = addr
    break
if not app_addr:
    print("    ✗ 아무것도 안 온다. 앱이 START 상태인지, 핫스팟이 붙었는지 확인할 것.")
    sys.exit(1)
print(f"    ✓ 앱 = {app_addr[0]}:{app_addr[1]}  ← 여기로 명령을 보낸다")

# 2) 안전한 명령만 보내고 응답을 기다린다
print(f"[2] HEARTBEAT + REQUEST_AUTOPILOT_CAPABILITIES(520) 송신 · 10초 대기")
seen, sent = {}, 0
t0 = time.time()
while time.time() - t0 < 10:
    if time.time() - t0 > sent:          # 1초에 한 번씩
        s.sendto(heartbeat(), app_addr)
        s.sendto(command_long(520, 1.0), app_addr)
        sent += 1
    try:
        d, addr = s.recvfrom(4096)
    except socket.timeout:
        continue
    if not d or d[0] != 0xFD or len(d) < 12:
        continue
    ln = d[1]; mid = d[7] | (d[8] << 8) | (d[9] << 16)
    p = d[10:10+ln].ljust(64, b"\x00")
    if mid == 77:                                   # COMMAND_ACK
        cmd, res = struct.unpack_from("<HB", p, 0)
        seen.setdefault("ACK", set()).add((cmd, res))
    elif mid == 148:                                # AUTOPILOT_VERSION
        cap, = struct.unpack_from("<Q", p, 0)
        seen["VERSION"] = f"capabilities=0x{cap:016x}"

print()
res_name = {0: "ACCEPTED ✓", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
if "ACK" in seen:
    print("  ✓ COMMAND_ACK 도착 — 상향 경로가 열려 있다")
    for cmd, res in sorted(seen["ACK"]):
        print(f"      명령 {cmd} → {res_name.get(res, res)}")
else:
    print("  ✗ COMMAND_ACK 없음 — 앱이 우리 패킷을 안 받고 있다")
if "VERSION" in seen:
    print(f"  ✓ AUTOPILOT_VERSION 도착 — {seen['VERSION']}")

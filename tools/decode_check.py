#!/usr/bin/env python3
"""MAVLink v1/v2 를 모두 풀어 실제 값을 본다."""
import socket, struct, time, math, collections

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("172.20.10.9", 14550)); s.settimeout(1.0)

got, ver, n, t0 = {}, collections.Counter(), 0, time.time()
while time.time() - t0 < 12:
    try: d, addr = s.recvfrom(4096)
    except socket.timeout: continue
    n += 1
    if not d: continue
    if d[0] == 0xFE and len(d) >= 8:                       # v1
        ln, mid, off = d[1], d[5], 6; ver["v1"] += 1
    elif d[0] == 0xFD and len(d) >= 12:                    # v2
        ln = d[1]; mid = d[7] | (d[8] << 8) | (d[9] << 16); off = 10; ver["v2"] += 1
    else:
        ver["기타"] += 1; continue
    p = d[off:off+ln].ljust(64, b"\x00")   # v2 truncation 복원
    got.setdefault("_from", addr[0])
    try:
        if mid == 33 and True:
            _, lat, lon, alt, ralt = struct.unpack_from("<Iiiii", p, 0)
            got["위치"] = f"위도 {lat/1e7:.6f}  경도 {lon/1e7:.6f}  고도 {alt/1000:.1f} m (상대 {ralt/1000:.1f} m)"
        elif mid == 24 and True:
            got["GPS"] = f"fix={p[28]}  위성 {p[29]}개"
        elif mid == 1 and True:
            got["배터리"] = f"{struct.unpack_from('<H',p,14)[0]/1000:.2f} V   잔량 {struct.unpack_from('<b',p,30)[0]}%"
        elif mid == 30 and True:
            _, r, pi, y = struct.unpack_from("<Ifff", p, 0)
            got["자세"] = f"roll {math.degrees(r):+.1f}°  pitch {math.degrees(pi):+.1f}°  yaw {math.degrees(y):+.1f}°"
        elif mid == 74 and True:
            asp, gsp, altf, climb = struct.unpack_from("<ffff", p, 0)
            got["비행"] = f"대지속도 {gsp:.2f} m/s  고도 {altf:.1f} m  기수 {struct.unpack_from('<h',p,16)[0]}°"
        elif mid == 0 and True:
            got["HEARTBEAT"] = f"type={p[4]} autopilot={p[5]} base_mode={p[6]} system_status={p[8]}"
    except Exception:
        pass

print(f"=== 12초 동안 패킷 {n}개 수신 · 형식 {dict(ver)} · 보낸 곳 {got.get('_from','(없음)')} ===")
for k in ("HEARTBEAT","위치","GPS","배터리","자세","비행"):
    print(f"  {k:<10} {got.get(k,'(안 옴)')}")

#!/usr/bin/env python3
"""UDP 14550 에 무엇이 도착하는지만 본다 — QGC 없이.

왜 필요한가:
  Step 7 에서 "QGC 에 아무것도 안 뜬다"는 증상은 원인이 셋인데 화면으로는 구별이 안 된다.
    (가) 앱이 안 쏜다        → 여기서 패킷 0
    (나) 네트워크/방화벽이 막는다 → 여기서 패킷 0  (단, 앱 화면의 GCS 등은 꺼져 있음)
    (다) QGC 설정 문제        → 여기서는 패킷이 보인다  ← 이걸로 (다)를 분리한다
  즉 이 스크립트에 패킷이 보이면 배관은 살아 있고 문제는 QGC 쪽이다.

의존성 없음(표준 라이브러리만). QGC 와 동시에 못 쓴다 — 포트를 하나가 잡는다.

사용:  .../python3 udp_probe.py [포트]     (기본 14550)  Ctrl+C 로 종료
"""
import socket, sys, time, collections

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14550
BIND = sys.argv[2] if len(sys.argv) > 2 else "0.0.0.0"   # 특정 인터페이스만 듣고 싶을 때

# MAVLink 메시지 ID 중 눈으로 확인할 값들
NAMES = {0: "HEARTBEAT", 1: "SYS_STATUS", 24: "GPS_RAW_INT", 30: "ATTITUDE",
         33: "GLOBAL_POSITION_INT", 35: "RC_CHANNELS_RAW", 65: "RC_CHANNELS",
         74: "VFR_HUD", 147: "BATTERY_STATUS", 253: "STATUSTEXT"}

def parse(b):
    """MAVLink v1(0xFE) / v2(0xFD) 헤더에서 msgid 만 뽑는다."""
    if not b:
        return None
    if b[0] == 0xFE and len(b) >= 6:          # v1: STX LEN SEQ SYS COMP MSGID
        return b[5]
    if b[0] == 0xFD and len(b) >= 10:         # v2: msgid 가 24bit 리틀엔디언
        return b[7] | (b[8] << 8) | (b[9] << 16)
    return None

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((BIND, PORT))
s.settimeout(1.0)

print(f"UDP {BIND}:{PORT} 에서 듣는 중 — Ctrl+C 로 종료")
print("(10초 동안 아무것도 안 뜨면 앱이 안 쏘거나 방화벽/네트워크가 막는 것이다)\n")

seen, senders, n, nbytes = collections.Counter(), set(), 0, 0
t0 = last = time.time()
try:
    while True:
        try:
            data, addr = s.recvfrom(4096)
        except socket.timeout:
            if time.time() - last >= 5:
                el = time.time() - t0
                print(f"[{el:5.0f}s] 패킷 {n}개 / {nbytes}바이트" +
                      (f" / 보낸 곳 {', '.join(sorted(senders))}" if senders else "  ← 아직 아무것도 안 옴"))
                last = time.time()
            continue
        n += 1; nbytes += len(data); senders.add(f"{addr[0]}:{addr[1]}")
        mid = parse(data)
        if mid is not None:
            if mid not in seen:
                print(f"  ★ 처음 보는 메시지  id={mid:<4} {NAMES.get(mid, '(이름 미등록)')}   from {addr[0]}")
            seen[mid] += 1
        if time.time() - last >= 5:
            el = time.time() - t0
            print(f"[{el:5.0f}s] 패킷 {n}개 / {nbytes}바이트 / 보낸 곳 {', '.join(sorted(senders))}")
            last = time.time()
except KeyboardInterrupt:
    pass
finally:
    print("\n=== 요약 ===")
    print(f"총 {n}개 패킷 / {nbytes}바이트 / {time.time()-t0:.0f}초")
    print(f"보낸 곳: {', '.join(sorted(senders)) or '(없음)'}")
    if seen:
        print("메시지 종류별 개수:")
        for mid, c in seen.most_common():
            print(f"  {mid:<5} {NAMES.get(mid,'(이름 미등록)'):<22} {c}")
        if 0 in seen:
            print("\n→ HEARTBEAT 이 온다. **MAVLink 파이프라인은 열려 있다.**")
            print("  QGC 에 안 뜬다면 원인은 QGC 쪽(포트/링크 설정)이다.")
    else:
        print("\n→ 아무것도 안 왔다. 앱의 'Use GCS on an external device'/IP, 방화벽, 같은 망인지를 본다.")

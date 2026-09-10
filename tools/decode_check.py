#!/usr/bin/env python3
"""MAVLink v1/v2 를 모두 풀어 실제 값을 본다.

사용법:  python3 tools/decode_check.py [포트=14550] [초=12]

⚠ bind 는 0.0.0.0 이다 — 이 PC 에 꽂힌 모든 랜선(유선·핫스팟)을 다 듣는다.
  특정 IP 로 bind 하면 핫스팟 IP 가 바뀔 때마다 파일을 고쳐야 하고,
  그 IP 가 없으면 실행 즉시 죽는다. 어차피 앱 주소는 봉투(recvfrom)에서 읽는다.
"""
import socket, struct, time, math, collections, sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14550
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

# DummyProduct.java 의 가짜 좌표 — 테스트 모드를 식별하는 지문
FAKE_LAT, FAKE_LON = -32.8540743, -71.2153629

FIX = {0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX",
       4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", PORT)); s.settimeout(1.0)

print(f"UDP 0.0.0.0:{PORT} 에서 {SECS:.0f}초 듣는 중…\n")

got, ver, n, t0 = {}, collections.Counter(), 0, time.time()
raw = {}          # 판정에 쓸 원시값
while time.time() - t0 < SECS:
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
        if mid == 33:                                      # GLOBAL_POSITION_INT
            _, lat, lon, alt, ralt = struct.unpack_from("<Iiiii", p, 0)
            raw["lat"], raw["lon"] = lat / 1e7, lon / 1e7
            got["위치"] = (f"위도 {lat/1e7:.6f}  경도 {lon/1e7:.6f}  "
                           f"고도 {alt/1000:.1f} m (상대 {ralt/1000:.1f} m)")
        elif mid == 24:                                    # GPS_RAW_INT
            glat, glon = struct.unpack_from("<ii", p, 8)
            fix, sats = p[28], p[29]
            raw["fix"], raw["sats"] = fix, sats
            raw["glat"], raw["glon"] = glat / 1e7, glon / 1e7
            got["GPS"] = (f"fix={fix}({FIX.get(fix, '?')})  위성 {sats}개  "
                          f"[GPS_RAW 좌표 {glat/1e7:.6f}, {glon/1e7:.6f}]")
        elif mid == 1:                                     # SYS_STATUS
            # 전류(offset 16, 10mA 단위)도 찍는다 — 전압 0V 가설의 판정에 쓴다.
            # 앱은 전압·전류를 둘 다 '합산 콜백'(배터리 여러 개 기종용) 변수에서
            # 꺼내 보낸다(DroneModel.java:1248,1249). Mavic 2 는 배터리가 하나라
            # 그 콜백이 안 도는 것으로 보인다 → 둘 다 0이면 가설이 맞다.
            raw["volt"] = struct.unpack_from('<H', p, 14)[0] / 1000.0
            raw["amp"] = struct.unpack_from('<h', p, 16)[0] / 100.0
            got["배터리"] = (f"{raw['volt']:.2f} V   전류 {raw['amp']:.2f} A   "
                             f"잔량 {struct.unpack_from('<b',p,30)[0]}%")
        elif mid == 30:                                    # ATTITUDE
            _, r, pi, y = struct.unpack_from("<Ifff", p, 0)
            got["자세"] = (f"roll {math.degrees(r):+.1f}°  pitch {math.degrees(pi):+.1f}°  "
                           f"yaw {math.degrees(y):+.1f}°")
        elif mid == 74:                                    # VFR_HUD
            asp, gsp, altf, climb = struct.unpack_from("<ffff", p, 0)
            got["비행"] = (f"대지속도 {gsp:.2f} m/s  고도 {altf:.1f} m  "
                           f"기수 {struct.unpack_from('<h',p,16)[0]}°")
        elif mid == 0:                                     # HEARTBEAT
            got["HEARTBEAT"] = (f"type={p[4]} autopilot={p[5]} "
                                f"base_mode={p[6]} system_status={p[8]}")
    except Exception:
        pass

print(f"=== 패킷 {n}개 수신 · 형식 {dict(ver)} · 보낸 곳 {got.get('_from','(없음)')} ===")
for k in ("HEARTBEAT", "위치", "GPS", "배터리", "자세", "비행"):
    print(f"  {k:<10} {got.get(k,'(안 옴)')}")

# ── T1 판정 ────────────────────────────────────────────────────────────
# 좌표가 0,0 일 때 "실내라 GPS 가 없는 것" 과 "앱 버그" 를 가르는 근거는
# GPS_RAW_INT 의 위성 수·fix_type 뿐이다. 화면만 봐서는 구별되지 않는다.
print("\n=== 판정 ===")
if n == 0:
    print("  ✗ 아무것도 안 옴 — T0(링크 위생)부터. 앱 START 상태 / 앱의 GCS IP / 포트 점유 확인")
elif "lat" not in raw:
    print("  ⚠ 패킷은 오는데 GLOBAL_POSITION_INT(33) 가 없음 — 기체가 안 붙었을 가능성")
else:
    lat, lon = raw["lat"], raw["lon"]
    fix, sats = raw.get("fix"), raw.get("sats")
    if abs(lat - FAKE_LAT) < 1e-3 and abs(lon - FAKE_LON) < 1e-3:
        print("  ✗ 테스트 모드(DummyProduct)의 가짜 좌표다.")
        print("    → 설정 → 앱 → RosettaDrone → 강제 중지 → 다시 열고 START")
    elif lat == 0.0 and lon == 0.0:
        if fix is None:
            print("  ⚠ 좌표 0,0 인데 GPS_RAW_INT(24) 가 안 와서 원인을 못 가린다")
        elif sats == 0 or fix < 2:
            print(f"  ✓ 좌표 0,0 · 위성 {sats}개 · fix={FIX.get(fix,fix)}")
            print("    → GPS 미수신이 원인. 앱은 정상이다. 야외로 나가야 한다")
            print("    → 실내에서 좌표가 필요하면 SIM 모드 + pref_sim_pos_lat/lon 사용")
        else:
            print(f"  ✗ 위성 {sats}개 · fix={FIX.get(fix,fix)} 인데 좌표가 0,0")
            print("    → GPS 는 잡혔다. 이건 앱/SDK 쪽 문제다. 소스를 볼 차례")
    else:
        print(f"  ✓ 실좌표 수신: {lat:.6f}, {lon:.6f}  (위성 {sats}개 · fix={FIX.get(fix,fix)})")
        if fix is not None and fix < 3:
            print("    ⚠ 다만 3D_FIX 가 아니다 — 위치 정확도를 믿지 말 것")

# ── 배터리 전압 0V 판정 (9/10 가설) ────────────────────────────────
# 앱은 전압·전류를 '합산 콜백'(배터리 여러 개 다는 기종용) 변수에서 꺼내 보내고,
# 잔량만 일반 배터리 콜백 값으로 계산한다 (DroneModel.java:1242 vs 1248,1249).
# Mavic 2 는 배터리가 하나라 합산 콜백이 안 도는 것으로 보인다.
if "volt" in raw:
    v, a = raw["volt"], raw["amp"]
    print()
    if v == 0 and a == 0:
        print("  ✓ 전압·전류가 둘 다 0 — 9/10 가설과 맞는다")
        print("    앱이 '합산 콜백'(배터리 여러 개 기종용) 변수를 내보내고 있다.")
        print("    진짜 값은 앱이 이미 읽고 있다(mCVoltage_mV). 내보내기만 하면 된다")
        print("    → patches/02 후보. send_sys_status() 두 줄")
    elif v == 0:
        print(f"  ⚠ 전압만 0인데 전류는 {a:.2f} A 다 — 9/10 가설이 깨진다")
        print("    둘 다 같은 합산 콜백에서 오므로 전류가 살아 있으면 설명이 안 된다.")
        print("    DroneModel.java 의 send_sys_status() 를 다시 읽을 것")
    else:
        print(f"  ✓ 전압 {v:.2f} V · 전류 {a:.2f} A — 0V 문제가 해결됐다")

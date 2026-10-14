#!/usr/bin/env python3
"""MAVLink v2 최소 구현 (표준 라이브러리만). link_node 와 도구들이 같이 쓴다.

rd_bridge/mav.py 는 이 파일의 심링크다 — 사본을 만들지 않는다.
CRC_EXTRA 는 앱 소스 CRC.java 에서 뽑은 값이다.
"""
import socket, struct, time, math

# msgid -> crc_extra
CRC_EXTRA = {
    0:  50,    # HEARTBEAT
    1:  124,   # SYS_STATUS
    24: 24,    # GPS_RAW_INT
    30: 28,    # ATTITUDE
    33: 104,   # GLOBAL_POSITION_INT
    74: 20,    # VFR_HUD
    76: 152,   # COMMAND_LONG
    77: 143,   # COMMAND_ACK
    84: 143,   # SET_POSITION_TARGET_LOCAL_NED
}

MAV_RESULT = {0: "ACCEPTED ✓", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
              3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}

CMD_NAV_TAKEOFF = 22
CMD_NAV_LAND = 21
CMD_DO_SET_SERVO = 183

# #84 마스크 — 위치 3비트와 YAW 를 반드시 무시해야 앱이 속도 명령으로 받는다.
# YAW 를 안 끄면 앱의 yaw PID 가 yawRate 를 덮어써 기수가 0° 로 돈다.
MASK_VEL_YAWRATE = (1 | 2 | 4) | (64 | 128 | 256) | 1024   # = 1479
MAV_FRAME_BODY_NED = 8


def crc16_mcrf4xx(data, crc=0xFFFF):
    for b in data:
        t = (b ^ (crc & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def parse(d):
    """받은 바이트 → (msgid, 페이로드). 아니면 (None, None).

    v2 는 페이로드 뒤쪽 0 을 잘라 보낸다 — 0 으로 채워 되돌려야 unpack 이 된다.
    """
    if not d:
        return None, None
    if d[0] == 0xFE and len(d) >= 8:                      # v1
        return d[5], d[6:6 + d[1]].ljust(80, b"\x00")
    if d[0] == 0xFD and len(d) >= 12:                     # v2
        mid = d[7] | (d[8] << 8) | (d[9] << 16)
        return mid, d[10:10 + d[1]].ljust(80, b"\x00")
    return None, None


def meters_between(lat1, lon1, lat2, lon2):
    """두 좌표 사이 거리 (m)."""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def bearing_between(lat1, lon1, lat2, lon2):
    """1 에서 2 를 본 방위각 (0~360°, 북=0, 동=90)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def offset_to_gps(lat, lon, heading, fwd, right):
    """기체 기준(앞·오른쪽 m) → 좌표. heading 은 deg (0=북, 시계)."""
    h = math.radians(heading)
    north = fwd * math.cos(h) - right * math.sin(h)
    east = fwd * math.sin(h) + right * math.cos(h)
    R = 6371000.0
    return (lat + math.degrees(north / R),
            lon + math.degrees(east / (R * math.cos(math.radians(lat)))))


def relative_bearing(move_bearing, heading):
    """방위각 → 기수 기준 상대각(-180~180). 0=앞, +90=오른쪽."""
    return (move_bearing - heading + 540.0) % 360.0 - 180.0


def name_direction(rel):
    """상대각 → 8방위 말."""
    names = ["앞", "앞-오른쪽", "오른쪽", "뒤-오른쪽", "뒤", "뒤-왼쪽", "왼쪽", "앞-왼쪽"]
    return names[int(((rel + 360.0) % 360.0 + 22.5) % 360.0 // 45.0)]


FAKE_LAT, FAKE_LON = -32.8540743, -71.2153629  # 앱 TEST 모드의 가짜 좌표


def bad_fix(lat, lon):
    """믿으면 안 되는 좌표면 그 이유, 멀쩡하면 None. 가짜 좌표를 목표로 쓰면 수천 km 를 날아간다."""
    if lat is None or lon is None:
        return "아직 좌표를 못 받았다"
    if math.isnan(lat) or math.isnan(lon) or math.isinf(lat) or math.isinf(lon):
        return "숫자가 아니다 (NaN·inf)"
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return f"범위를 벗어났다 ({lat}, {lon})"
    if abs(lat - FAKE_LAT) < 1e-3 and abs(lon - FAKE_LON) < 1e-3:
        return "앱 TEST 모드의 가짜 좌표다"
    if lat == 0.0 and lon == 0.0:
        return "0,0 이다 (위성 미수신)"
    return None


class Link:
    """UDP 14550 을 잡고, 앱이 마지막으로 보낸 주소로 답장한다.

    앱은 송신 포트를 켤 때마다 바꾸므로(임시 포트) 고정 주소로 쏘면 버려진다.
    앱이 도중에 재시작해도 따라가도록 poll() 이 매번 주소를 갱신한다.
    """

    def __init__(self, port=14550, sysid=255, compid=190):
        self.sysid, self.compid = sysid, compid
        self.seq = 0
        self.addr = None
        self._last_beat = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.02)
        # 최근 텔레메트리. t_* 는 그 값이 도착한 시각 — 새 값과 옛 값을 가른다
        self.tel = {}

    def frame(self, msgid, payload):
        p = payload.rstrip(b"\x00") or b"\x00"            # v2 truncation
        h = struct.pack("<BBBBBB", len(p), 0, 0, self.seq, self.sysid, self.compid)
        h += struct.pack("<I", msgid)[:3]
        self.seq = (self.seq + 1) % 256
        c = crc16_mcrf4xx(h + p)
        c = crc16_mcrf4xx(bytes([CRC_EXTRA[msgid]]), c)
        return b"\xFD" + h + p + struct.pack("<H", c)

    def send(self, msgid, payload):
        if self.addr:
            self.sock.sendto(self.frame(msgid, payload), self.addr)

    def discover(self, secs=6.0):
        """앱이 어느 주소:포트에서 쏘는지 알아낸다."""
        t0 = time.time()
        while time.time() - t0 < secs:
            try:
                _, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            self.addr = addr
            return addr
        return None

    def heartbeat(self):
        """GCS HEARTBEAT — 1초에 한 번만 나간다 (앱의 GCS_TIMEOUT 2초)."""
        now = time.time()
        if now - self._last_beat < 1.0:
            return
        self._last_beat = now
        # custom_mode, type=6(GCS), autopilot=8(INVALID), base_mode, status=4(ACTIVE), ver=3
        self.send(0, struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3))

    def command_long(self, cmd, *params):
        p = list(params) + [0.0] * (7 - len(params))
        payload = struct.pack("<7f", *p[:7])
        payload += struct.pack("<HBBB", cmd, 0, 0, 0)     # target_system=0 → 아무나 실행
        self.send(76, payload)

    def set_velocity(self, vx, vy, vz, yaw_rate_deg=0.0):
        """#84 기체 기준 속도 (m/s, deg/s). 한 발의 효력은 200ms — 20Hz 로 계속 보낸다."""
        payload = struct.pack(
            "<Ifffffffffff",
            0,                    # time_boot_ms
            0.0, 0.0, 0.0,        # x, y, z   (마스크로 무시)
            vx, vy, vz,
            0.0, 0.0, 0.0,        # afx, afy, afz
            0.0,                  # yaw
            math.radians(yaw_rate_deg),
        )
        payload += struct.pack("<HBBB", MASK_VEL_YAWRATE, 0, 0, MAV_FRAME_BODY_NED)
        self.send(84, payload)

    def poll(self):
        """도착해 있는 것을 전부 읽어 self.tel 을 갱신하고 [(msgid, 페이로드)] 를 돌려준다."""
        out = []
        while True:
            try:
                d, self.addr = self.sock.recvfrom(4096)
            except (socket.timeout, BlockingIOError):
                break
            mid, p = parse(d)
            if mid is None:
                continue
            out.append((mid, p))
            try:
                if mid == 33:                              # GLOBAL_POSITION_INT
                    _, lat, lon, _, ralt = struct.unpack_from("<Iiiii", p, 0)
                    self.tel["lat"], self.tel["lon"] = lat / 1e7, lon / 1e7
                    self.tel["alt"] = ralt / 1000.0        # 상대고도
                    # 속도는 cm/s · NED — vd>0 이 하강이다 (#84 의 vz 와 반대)
                    vn, ve, vd = struct.unpack_from("<hhh", p, 20)
                    self.tel["vn"], self.tel["ve"], self.tel["vd"] = vn / 100.0, ve / 100.0, vd / 100.0
                    self.tel["t_pos"] = time.time()
                elif mid == 30:                            # ATTITUDE
                    _, r, pi, y = struct.unpack_from("<Ifff", p, 0)
                    self.tel["roll"] = math.degrees(r)
                    self.tel["pitch"] = math.degrees(pi)
                    self.tel["yaw"] = math.degrees(y) % 360.0
                    self.tel["t_att"] = time.time()
                elif mid == 74:                            # VFR_HUD
                    self.tel["gspeed"] = struct.unpack_from("<f", p, 4)[0]
                    self.tel["t_hud"] = time.time()
                elif mid == 24:                            # GPS_RAW_INT
                    self.tel["fix"], self.tel["sats"] = p[28], p[29]
                elif mid == 1:                             # SYS_STATUS (patches/03 필요)
                    self.tel["volt"] = struct.unpack_from("<H", p, 14)[0] / 1000.0
                    self.tel["amp"] = struct.unpack_from("<h", p, 16)[0] / 100.0
                    self.tel["batt_pct"] = struct.unpack_from("<b", p, 30)[0]
                    self.tel["t_sys"] = time.time()
            except Exception:
                pass
        return out

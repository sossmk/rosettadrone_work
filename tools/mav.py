#!/usr/bin/env python3
"""MAVLink v2 최소 구현 — 이 저장소의 시험 도구들이 공유한다.

의존성 없음(표준 라이브러리만). pymavlink 를 안 쓰는 이유는 8/28 기록과 같다:
새 PC 에서 pip 없이도 바로 돌아야 한다.

⚠ CRC_EXTRA 는 추측이 아니라 앱 소스에서 뽑은 값이다:
   rosettadrone/app/src/main/java/com/MAVLink/common/CRC.java
"""
import socket, struct, time, math

# msgid -> crc_extra  (CRC.java 의 MAVLINK_MESSAGE_CRCS 에서 발췌)
CRC_EXTRA = {
    0:  50,    # HEARTBEAT
    1:  124,   # SYS_STATUS
    24: 24,    # GPS_RAW_INT
    30: 28,    # ATTITUDE
    33: 104,   # GLOBAL_POSITION_INT
    47: 153,   # MISSION_ACK
    73: 38,    # MISSION_ITEM_INT
    74: 20,    # VFR_HUD
    76: 152,   # COMMAND_LONG
    77: 143,   # COMMAND_ACK
    84: 143,   # SET_POSITION_TARGET_LOCAL_NED
}

MAV_RESULT = {0: "ACCEPTED ✓", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
              3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}

MAV_MISSION_RESULT = {0: "MISSION_ACCEPTED ✓", 1: "MISSION_ERROR",
                       2: "MISSION_UNSUPPORTED_FRAME", 3: "MISSION_UNSUPPORTED",
                       4: "MISSION_NO_SPACE", 5: "MISSION_INVALID",
                       13: "MISSION_DENIED"}

# MAV_CMD
CMD_NAV_TAKEOFF = 22
CMD_DO_SET_SERVO = 183

# POSITION_TARGET_TYPEMASK (mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK)
#   속도만 쓰려면 위치 3비트를 반드시 켜야 한다 — DroneModel.startMotion() 이
#   ignorePosX && ignorePosY && ignorePosZ 일 때만 isVelocityCommand 로 친다.
#   YAW_IGNORE 도 반드시 켠다. 안 켜면 processVelocities() 가 yaw PID 로
#   yawRate 를 덮어써서 기수 0° 로 제멋대로 돈다 (DroneModel.java:2212).
MASK_VEL_YAWRATE = (1 | 2 | 4) | (64 | 128 | 256) | 1024   # = 1479

MAV_FRAME_BODY_NED = 8
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16


def crc16_mcrf4xx(data, crc=0xFFFF):
    for b in data:
        t = (b ^ (crc & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def parse(d):
    """받은 바이트에서 (msgid, 페이로드) 를 뽑는다. 아니면 (None, None).

    ⚠ MAVLink v2 는 페이로드 뒤쪽 0 을 잘라 보낸다(truncation).
      0 으로 채워 되돌리지 않으면 멀쩡한 메시지를 통째로 놓친다 (8/28 에 실제로 겪음).
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
    """두 좌표 사이 거리 (m). 짧은 거리라 구면 근사로 충분하다."""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def bearing_between(lat1, lon1, lat2, lon2):
    """lat1,lon1 에서 lat2,lon2 를 본 방위각 (0~360°, 북=0, 동=90)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def relative_bearing(move_bearing, heading):
    """이동 방위각을 기수 기준 상대각(-180~180)으로. 0=앞, +90=오른쪽, 180=뒤, -90=왼쪽."""
    return (move_bearing - heading + 540.0) % 360.0 - 180.0


def name_direction(rel):
    """상대각을 사람 말로. 45° 폭으로 8방위."""
    if rel is None:
        return "?"
    r = (rel + 360.0) % 360.0
    names = ["앞", "앞-오른쪽", "오른쪽", "뒤-오른쪽", "뒤", "뒤-왼쪽", "왼쪽", "앞-왼쪽"]
    return names[int((r + 22.5) % 360.0 // 45.0)]


class Link:
    """UDP 14550 을 잡고, 앱이 보낸 봉투에서 발신 주소를 읽어 그리로 답장한다.

    앱은 new DatagramSocket() 으로 임시 포트를 쓰므로(MAVLinkConnection.java)
    송신 포트가 켤 때마다 바뀐다. 고정 포트로 쏘면 조용히 버려진다.
    또 앱은 socket.connect(GCS_IP, 14550) 상태라 우리 쪽 포트는 14550 이어야 한다.
    """

    def __init__(self, port=14550, sysid=255, compid=190):
        self.sysid, self.compid = sysid, compid
        self.seq = 0
        self.addr = None
        self._last_beat = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))     # 모든 랜선을 듣는다 — 핫스팟 IP 무관
        self.sock.settimeout(0.02)

        # 최근 텔레메트리 (poll() 이 갱신한다)
        self.tel = {}
        self.addr_changes = 0     # 앱의 송신 포트가 바뀐 횟수 (아래 poll() 참고)

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
        """GCS HEARTBEAT. 1초에 한 번만 실제로 나간다 (앱의 GCS_TIMEOUT 은 2초)."""
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

    def set_velocity(self, vx, vy, vz, yaw_rate_deg=0.0,
                     mask=MASK_VEL_YAWRATE, frame=MAV_FRAME_BODY_NED):
        """SET_POSITION_TARGET_LOCAL_NED (#84) 로 속도를 보낸다.

        vx/vy/vz 는 m/s. yaw_rate 는 보기 편하라고 deg/s 로 받아 rad/s 로 바꿔 보낸다
        (앱이 MAVLinkReceiver 에서 Math.toDegrees() 로 되돌린다).

        ⚠ 이 한 발의 효력은 200ms 뿐이다 — velocityTicks=4 × MOTION_PERIOD_MS=50ms.
          연속 발행하지 않으면 곧바로 멈춘다 (DroneModel.java:2250, 2303).
        """
        payload = struct.pack(
            "<Ifffffffffff",
            0,                    # time_boot_ms
            0.0, 0.0, 0.0,        # x, y, z   (마스크로 무시됨)
            vx, vy, vz,
            0.0, 0.0, 0.0,        # afx, afy, afz
            0.0,                  # yaw
            math.radians(yaw_rate_deg),
        )
        payload += struct.pack("<HBBB", mask, 0, 0, frame)
        self.send(84, payload)

    def mission_item_int(self, lat_deg, lon_deg, alt_m,
                          frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          command=MAV_CMD_NAV_WAYPOINT,
                          target_system=1):
        """MISSION_ITEM_INT (#73) 로 단발 FlyTo 를 보낸다.

        MAVLinkReceiver.processMsgItemMessage() 가 MAV_CMD_NAV_WAYPOINT 를
        미션 밖(!isReceivingMission)에서 받으면 mModel.flyTo(x/1e7, y/1e7, z) 를
        직접 부른다 — 미션을 업로드하는 게 아니라 그 자리에서 한 번 날아간다.
        (`markWaypoint()` NPE 는 앱 화면 UI 전용이라 이 경로와 무관.)

        ⚠⚠ target_system=0("아무나")이 여기서는 안 통한다 — command_long() 의 관례를
          그대로 가져왔다가 9/11 실기에서 goto 가 조용히 무시되는 버그로 이어졌다.
          processMsgItemMessage() 맨 앞의 체크는:
              if (mModel.getSystemId() != msg_item.target_system) return;
          COMMAND_LONG 핸들러에만 있는 "target_system==0 이면 아무나" 예외가
          여기엔 없다. `mModel.getSystemId()` 기본값은 1(DroneModel.java:145) —
          그래서 기본값을 1로 둔다.

        ⚠ MAVLink 는 페이로드를 필드 크기 내림차순으로 재배열해 보낸다(XML 선언
          순서가 아니다) — 4바이트(param1-4·x·y·z) → 2바이트(seq·command) →
          1바이트(target_system·target_component·frame·current·autocontinue) →
          확장필드(mission_type). 순서를 XML 선언 그대로 짜면 CRC 는 맞아도
          필드가 죄다 엉뚱한 오프셋에서 읽혀 위험하다 — 아래는 재배열된 순서다.

        ⚠ z(고도)는 스케일 없이 그대로 넘어간다. flyTo() 의 고도 기준(상대/절대)은
          소스로 재확인 못 했다 — MAV_FRAME_GLOBAL_RELATIVE_ALT 로 보내고,
          `capture` 로 읽은 상대고도(tel["alt"])를 그대로 재사용하는 것을 전제로
          한다. 첫 실기 시도는 짧은 거리·저고도로 확인할 것.
        """
        payload = struct.pack(
            "<ffffiifHHBBBBBB",
            0.0, 0.0, 0.0, 0.0,                    # param1-4 (waypoint 는 미사용)
            int(round(lat_deg * 1e7)),             # x = lat*1e7
            int(round(lon_deg * 1e7)),             # y = lon*1e7
            alt_m,                                 # z
            0,                                      # seq
            command,                                # command (16=NAV_WAYPOINT)
            target_system, 0,                       # target_system(기본 1) · target_component
            frame,
            0,                                       # current
            0,                                       # autocontinue
            0,                                       # mission_type (0=MAV_MISSION_TYPE_MISSION)
        )
        self.send(73, payload)

    def poll(self):
        """지금 도착해 있는 것을 전부 읽어 self.tel 을 갱신하고 목록을 돌려준다.

        ⚠ 봉투의 발신 주소를 매번 따라간다. 앱은 new DatagramSocket() 으로 임시
          포트를 쓰므로 **앱을 재시작하면 포트가 바뀐다**(제약 #7). discover() 로
          한 번 잡은 주소를 계속 쓰면, 앱이 도중에 재시작한 뒤로는 죽은 포트에
          조용히 쏘게 된다 — 화면상 "명령이 안 먹는다"로만 보인다. 그래서 여기서
          갱신한다 (discover() 와 같은 규칙: 마지막으로 보낸 쪽이 앱이다).
        """
        out = []
        while True:
            try:
                d, addr = self.sock.recvfrom(4096)
            except (socket.timeout, BlockingIOError):
                break
            if addr != self.addr:
                if self.addr is not None:
                    self.addr_changes += 1
                self.addr = addr
            mid, p = parse(d)
            if mid is None:
                continue
            out.append((mid, p))
            try:
                if mid == 33:                              # GLOBAL_POSITION_INT
                    _, lat, lon, alt, ralt = struct.unpack_from("<Iiiii", p, 0)
                    self.tel["lat"], self.tel["lon"] = lat / 1e7, lon / 1e7
                    self.tel["alt"] = ralt / 1000.0
                elif mid == 30:                            # ATTITUDE
                    _, r, pi, y = struct.unpack_from("<Ifff", p, 0)
                    self.tel["yaw"] = math.degrees(y) % 360.0
                elif mid == 74:                            # VFR_HUD
                    _, gsp, altf, climb = struct.unpack_from("<ffff", p, 0)
                    self.tel["gspeed"] = gsp
                    self.tel["climb"] = climb
                elif mid == 24:                            # GPS_RAW_INT
                    self.tel["fix"], self.tel["sats"] = p[28], p[29]
                elif mid == 47:                            # MISSION_ACK
                    self.tel["mission_ack"] = p[2]          # MAV_MISSION_RESULT 코드
            except Exception:
                pass
        return out

    def wait_ack(self, cmd, secs=6.0):
        """특정 명령의 COMMAND_ACK 를 기다린다. (결과코드, 걸린시간) 또는 (None, secs)."""
        t0 = time.time()
        while time.time() - t0 < secs:
            self.heartbeat()
            for mid, p in self.poll():
                if mid == 77:
                    c, res = struct.unpack_from("<HB", p, 0)
                    if c == cmd:
                        return res, time.time() - t0
            time.sleep(0.01)
        return None, secs

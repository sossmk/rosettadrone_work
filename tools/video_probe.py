#!/usr/bin/env python3
"""T4 — 영상. RTP/H.264 를 풀어 「왜 안 되는지」 판정한다.

    python3 tools/video_probe.py                      # 5600 에서 15초
    python3 tools/video_probe.py --secs 30
    python3 tools/video_probe.py --save out.h264      # Annex-B 로 받아 적는다

`decode_check.py` 가 텔레메트리의 `0,0` 에 대해 한 일을 영상에 대해 한다.
「왔다/안 왔다」가 아니라 **어디서 끊겼는지**를 찍는 것이 목적이다.

⚠ bind 는 0.0.0.0 이다 — 이 PC 에 꽂힌 모든 랜선(유선·핫스팟)을 다 듣는다.
  특정 IP 로 bind 하면 핫스팟 IP 가 바뀔 때마다 파일을 고쳐야 한다.

⚠ **RTCP 가 같은 포트로 온다.** VideoService 가
  `setDestination(ip, videoPort, videoPort)` 로 RTP·RTCP 포트를 같은 값으로 준다
  (RtpSocket.java:191). 걸러내지 않으면 Sender Report(PT 200)를 영상으로 오독한다.
  영상은 **PT 96** 이다 (RtpSocket.java:121, 마커비트가 켜지면 224).

⚠ `--save` 파일을 반드시 남길 것. 한 번 받아 두면 **드론 없이도 진짜 데이터로**
  ROS 브리지와 AR 태그 검출을 개발할 수 있다.

앱 쪽 경로 (소스로 확인):
    DJI 콜백 → NativeHelper.parse(buf,size,1) → noparse()   ← 9/9 에 여기서 죽었다
      → VideoService.onDataRecv()   (isRunning 이 true 여야 통과)
      → splitNALs() → H264Packetizer → RtpSocket → UDP
"""
import argparse, socket, struct, sys, time, collections

NAL_NAMES = {
    1: "슬라이스(non-IDR)", 2: "슬라이스 A", 3: "슬라이스 B", 4: "슬라이스 C",
    5: "IDR 키프레임", 6: "SEI", 7: "SPS", 8: "PPS", 9: "AUD",
    10: "시퀀스 끝", 11: "스트림 끝", 12: "filler",
}
START = b"\x00\x00\x00\x01"


# ---------------------------------------------------------------- SPS 해석
class Bits:
    """SPS 를 읽기 위한 최소 비트 리더 (지수 골룸)."""

    def __init__(self, data):
        self.d, self.pos = data, 0

    def u1(self):
        b = (self.d[self.pos >> 3] >> (7 - (self.pos & 7))) & 1
        self.pos += 1
        return b

    def u(self, n):
        v = 0
        for _ in range(n):
            v = (v << 1) | self.u1()
        return v

    def ue(self):
        z = 0
        while self.u1() == 0:
            z += 1
            if z > 32:
                raise ValueError("ue 폭주")
        return (1 << z) - 1 + self.u(z) if z else 0

    def se(self):
        k = self.ue()
        return (k + 1) // 2 if k % 2 else -(k // 2)


def _unescape(d):
    """emulation prevention byte(00 00 03) 를 걷어낸다."""
    out, i = bytearray(), 0
    while i < len(d):
        if i + 2 < len(d) and d[i] == 0 and d[i + 1] == 0 and d[i + 2] == 3:
            out += b"\x00\x00"
            i += 3
        else:
            out.append(d[i])
            i += 1
    return bytes(out)


def parse_sps(nal):
    """SPS NAL 에서 (가로, 세로, profile, level) 을 뽑는다. 실패하면 None."""
    try:
        b = Bits(_unescape(nal[1:]))          # NAL 헤더 1바이트를 뗀다
        profile = b.u(8)
        b.u(8)                                # constraint flags + reserved
        level = b.u(8)
        b.ue()                                # seq_parameter_set_id
        chroma = 1
        if profile in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
            chroma = b.ue()
            if chroma == 3:
                b.u1()                        # separate_colour_plane_flag
            b.ue(); b.ue()                    # bit_depth luma/chroma minus8
            b.u1()                            # qpprime_y_zero_transform_bypass
            if b.u1():                        # seq_scaling_matrix_present
                for i in range(8 if chroma != 3 else 12):
                    if b.u1():
                        last = nxt = 8
                        for _ in range(16 if i < 6 else 64):
                            if nxt:
                                nxt = (last + b.se() + 256) % 256
                            last = nxt or last
        b.ue()                                # log2_max_frame_num_minus4
        poc = b.ue()
        if poc == 0:
            b.ue()
        elif poc == 1:
            b.u1(); b.se(); b.se()
            for _ in range(b.ue()):
                b.se()
        b.ue()                                # max_num_ref_frames
        b.u1()                                # gaps_in_frame_num_allowed
        w_mbs = b.ue() + 1
        h_units = b.ue() + 1
        frame_mbs_only = b.u1()
        if not frame_mbs_only:
            b.u1()                            # mb_adaptive_frame_field
        b.u1()                                # direct_8x8_inference
        cl = cr = ct = cb = 0
        if b.u1():                            # frame_cropping_flag
            cl, cr, ct, cb = b.ue(), b.ue(), b.ue(), b.ue()
        sub_w = 2 if chroma in (1, 2) else 1
        sub_h = 2 if chroma == 1 else 1
        unit_x = sub_w if chroma else 1
        unit_y = (sub_h if chroma else 1) * (2 - frame_mbs_only)
        w = w_mbs * 16 - (cl + cr) * unit_x
        h = h_units * 16 * (2 - frame_mbs_only) - (ct + cb) * unit_y
        if not (0 < w <= 8192 and 0 < h <= 8192):
            return None
        return w, h, profile, level
    except Exception:
        return None


# ---------------------------------------------------------------- 본체
def main():
    ap = argparse.ArgumentParser(
        description="RTP/H.264 를 풀어 영상이 흐르는지 판정한다 (T4)")
    ap.add_argument("--port", type=int, default=5600, help="듣는 포트 (기본 5600)")
    ap.add_argument("--secs", type=float, default=15.0, help="듣는 시간 (기본 15초)")
    ap.add_argument("--save", metavar="파일", help="Annex-B H.264 로 받아 적는다")
    a = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", a.port))
    except OSError as e:
        print(f"✗ 포트 {a.port} 을 못 잡았다: {e}")
        print("  이미 쓰는 프로그램이 있다 →  ss -lunp | grep {}".format(a.port))
        return 2
    s.settimeout(0.5)

    out = open(a.save, "wb") if a.save else None
    print(f"UDP 0.0.0.0:{a.port} 에서 {a.secs:.0f}초 듣는 중…"
          + (f"  → {a.save} 에 기록" if out else ""))
    print("(앱 설정: Enable video feed 켜기 · GCS IP = 이 PC · Video port = "
          f"{a.port})\n")

    senders = collections.Counter()
    kinds = collections.Counter()       # rtp / rtcp / 기타
    nals = collections.Counter()        # NAL 종류별
    frag = collections.Counter()        # 단일 / STAP-A / FU-A
    sps_info, ssrcs = None, set()
    pkts = payload_bytes = total_bytes = 0
    lost = reorder = 0
    prev_seq = None
    markers = 0
    t0 = last_tick = time.time()
    tick_pkts = 0

    while time.time() - t0 < a.secs:
        try:
            d, addr = s.recvfrom(65535)
        except socket.timeout:
            d = None
        if d:
            total_bytes += len(d)
            senders[addr[0]] += 1
            pkts += 1
            tick_pkts += 1

        now = time.time()
        if now - last_tick >= 1.0:
            kbps = total_bytes * 8 / 1000.0 / (now - t0)
            print(f"  {now-t0:4.0f}초   패킷 {pkts:6d}   {kbps:8.1f} kbps", end="\r")
            last_tick, tick_pkts = now, 0

        if not d or len(d) < 12:
            continue

        # ---- RTP / RTCP 가르기 --------------------------------------
        if (d[0] >> 6) != 2:                       # version != 2
            kinds["기타"] += 1
            continue
        # RTCP 의 PT 는 바이트 전체(200~204)다 — 마커비트를 떼면 안 된다.
        # 0x7F 를 먼저 걸면 200 이 72 로 뭉개져 영영 안 걸린다 (9/10 에 실측으로 잡음).
        if 200 <= d[1] <= 204:                     # RTCP (SR/RR/SDES/BYE/APP)
            kinds["rtcp"] += 1
            continue
        pt = d[1] & 0x7F                           # RTP 는 마커비트를 뗀다
        if pt != 96:
            kinds[f"pt{pt}"] += 1
            continue
        kinds["rtp"] += 1

        if d[1] & 0x80:
            markers += 1
        seq = struct.unpack_from(">H", d, 2)[0]
        ssrcs.add(struct.unpack_from(">I", d, 8)[0])

        if prev_seq is not None:
            gap = (seq - prev_seq) & 0xFFFF
            if gap == 0:
                pass
            elif gap < 0x8000:
                lost += gap - 1
            else:
                reorder += 1
        prev_seq = seq

        off = 12 + (d[0] & 0x0F) * 4               # CSRC
        if d[0] & 0x10:                            # 확장 헤더
            if len(d) < off + 4:
                continue
            off += 4 + struct.unpack_from(">H", d, off + 2)[0] * 4
        p = d[off:]
        if not p:
            continue
        payload_bytes += len(p)

        t = p[0] & 0x1F
        if 1 <= t <= 23:                           # 단일 NAL
            frag["단일"] += 1
            nals[t] += 1
            if t == 7 and sps_info is None:
                sps_info = parse_sps(p)
            if out:
                out.write(START + p)

        elif t == 24:                              # STAP-A (여러 NAL 한 봉투)
            frag["STAP-A"] += 1
            i = 1
            while i + 2 <= len(p):
                n = struct.unpack_from(">H", p, i)[0]
                i += 2
                unit = p[i:i + n]
                i += n
                if not unit:
                    break
                nals[unit[0] & 0x1F] += 1
                if (unit[0] & 0x1F) == 7 and sps_info is None:
                    sps_info = parse_sps(unit)
                if out:
                    out.write(START + unit)

        elif t == 28:                              # FU-A (조각)
            frag["FU-A"] += 1
            if len(p) < 2:
                continue
            fu = p[1]
            inner = fu & 0x1F
            if fu & 0x80:                          # 시작 조각
                nals[inner] += 1
                if out:
                    out.write(START + bytes([(p[0] & 0xE0) | inner]) + p[2:])
            elif out:
                out.write(p[2:])

        else:
            frag[f"타입{t}"] += 1

    s.close()
    if out:
        out.close()

    # ---------------------------------------------------------------- 판정
    el = max(time.time() - t0, 0.001)
    print(" " * 60, end="\r")
    print(f"\n{'='*58}\n결과 ({el:.0f}초)\n{'='*58}")
    print(f"  받은 패킷      {pkts}   ({total_bytes/1024:.0f} KiB, "
          f"{total_bytes*8/1000/el:.1f} kbps)")
    if senders:
        print("  보낸 곳        " + ", ".join(f"{ip} ({n})" for ip, n in senders.most_common()))
    if kinds:
        print("  갈래           " + "  ".join(f"{k}={v}" for k, v in kinds.most_common()))
    if kinds["rtp"]:
        print(f"  SSRC           {', '.join(hex(x) for x in ssrcs)}")
        print(f"  seq            손실 {lost} · 순서뒤바뀜 {reorder} · 마커(프레임끝) {markers}")
        print("  포장           " + "  ".join(f"{k}={v}" for k, v in frag.most_common()))
    if nals:
        print("  NAL 종류")
        for t, n in sorted(nals.items()):
            print(f"      {t:2d} {NAL_NAMES.get(t,'?'):<16} {n}")
    if sps_info:
        w, h, prof, lvl = sps_info
        print(f"  해상도         {w}x{h}  (profile {prof}, level {lvl/10:.1f})")
    if out:
        print(f"  기록           {a.save}  ({__import__('os').path.getsize(a.save)/1024:.0f} KiB)")

    print(f"\n{'-'*58}\n판정\n{'-'*58}")
    if pkts == 0:
        print("  ✗ 아무것도 안 온다.")
        print("    ① 앱 설정 — Enable video feed 가 켜져 있나? Video port 가 "
              f"{a.port} 인가?")
        print("    ② 목적지 IP — pref_separate_gcs 가 꺼져 있으면 GCS IP 로 간다.")
        print("       ./tools/link_check.sh 가 찍어 주는 IP 와 같아야 한다")
        print("    ③ VideoService.isRunning — setParameters() 가 안 불리면")
        print("       크래시 없이 조용히 아무것도 안 나간다 (바인딩 실패)")
        print("    ④ 방화벽 / 같은 핫스팟에 붙어 있나")
        return 1
    if kinds["rtp"] == 0:
        print(f"  ✗ 뭔가 오는데 영상 RTP(PT 96)가 아니다. 위 「갈래」를 볼 것.")
        print("    rtcp 만 온다면 링크는 섰는데 프레임이 안 실린 것이다 —")
        print("    noparse() 크래시가 아직이거나 onDataRecv 가 안 불리고 있다")
        return 1
    have_sps, have_pps = nals.get(7, 0), nals.get(8, 0)
    have_idr, have_slice = nals.get(5, 0), nals.get(1, 0)
    if not (have_sps and have_pps):
        print("  ⚠ 영상 RTP 는 오는데 SPS/PPS 가 없다 → 디코더가 못 연다.")
        print(f"    (SPS {have_sps} · PPS {have_pps})  더 길게 들어 볼 것: --secs 30")
        return 1
    if not have_idr:
        print("  ⚠ SPS/PPS 는 오는데 IDR 키프레임이 없다 → 첫 화면을 못 만든다.")
        print("    인코더가 키프레임을 안 내고 있다. --secs 30 으로 다시 볼 것")
        return 1
    print("  ✓ T4 통과 — SPS·PPS·IDR·슬라이스가 전부 왔다.")
    print(f"    키프레임 {have_idr}장 · 슬라이스 {have_slice}장"
          + (f" · {sps_info[0]}x{sps_info[1]}" if sps_info else ""))
    if lost:
        print(f"    ⚠ 패킷 손실 {lost} — 무선이 좁으면 화면이 깨진다")
    if a.save:
        print(f"    ▸ {a.save} 로 재생/개발:  ffplay {a.save}")
        print("      이 파일이 있으면 드론 없이 ROS 브리지·태그 검출을 만들 수 있다")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n중단")

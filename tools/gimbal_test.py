#!/usr/bin/env python3
"""T3 — 짐벌 틸팅. PC 명령이 기체 하드웨어를 실제로 움직이는지 본다.

    python3 tools/gimbal_test.py [--port 14550] [--yaw] [--angle -45]

왜 이걸 먼저 하나: **프로펠러 없이, 이륙 없이, 지상에서** 업링크 전체를
증명하는 가장 값싼 실험이다. 카메라가 실제로 아래를 보면
PC→앱→DJI SDK→기체 경로가 전부 살아 있다는 뜻이고, 그 다음부터
cmd_vel 은 "무슨 명령을 어떻게 만드나" 문제로 줄어든다.

⚠ 표준 명령이 안 먹는다. 앱에 DO_MOUNT_CONTROL(205) 도
  DO_GIMBAL_MANAGER_PITCHYAW(1000) 도 없다 (MAVLinkReceiver.java:290 에
  "TODO: Implement" 로 남아 있다). 유일한 경로가 DO_SET_SERVO(183) 다:

      DroneModel.doSetGimbal(channel, value):
          param = (value - 1500.0) / 5.5
          channel 9 → pitch (ABSOLUTE_ANGLE)
          channel 8 → yaw   (RELATIVE_ANGLE — 누적된다!)

  즉  PWM = 1500 + 각도 × 5.5   ( -90° → 1005,  0° → 1500 )
"""
import argparse, sys, os, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

CH_PITCH, CH_YAW = 9, 8


def pwm(angle_deg):
    """각도(°) → PWM. doSetGimbal() 의 역함수."""
    return 1500.0 + angle_deg * 5.5


def step(link, label, channel, angle):
    v = pwm(angle)
    print(f"  {label:<12} 채널 {channel} · {angle:+.0f}° · PWM {v:.1f} … ", end="", flush=True)
    link.command_long(mav.CMD_DO_SET_SERVO, float(channel), float(v))
    # rotate() 는 Rotation.Builder().time(2) — 2초에 걸쳐 돈다.
    # ACK 는 회전 콜백 안에서 나가므로(DroneModel.java:2029) 넉넉히 기다린다.
    res, dt = link.wait_ack(mav.CMD_DO_SET_SERVO, secs=8.0)
    if res is None:
        print("ACK 없음 ✗")
        return False
    print(f"{mav.MAV_RESULT.get(res, res)}  ({dt:.1f}초)")
    if res == 1:
        print("      → TEMPORARILY_REJECTED 는 mGimbal == null 이라는 뜻이다.")
        print("        기체가 안 붙었거나 짐벌을 못 잡았다.")
    elif res == 3:
        print("      → UNSUPPORTED. 채널이 8/9 가 아니면 이렇게 나온다.")
    return res == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--angle", type=float, default=None,
                    help="이 각도 하나만 보낸다 (기본: 0 → -45 → -90 → 0 순서)")
    ap.add_argument("--yaw", action="store_true",
                    help="pitch 대신 yaw(채널 8)를 돌린다. 상대각이라 누적된다")
    args = ap.parse_args()

    link = mav.Link(port=args.port)
    print(f"[1] 앱의 송신 포트를 찾는 중… (0.0.0.0:{args.port} 에서 수신)")
    addr = link.discover(6.0)
    if not addr:
        print("    ✗ 아무것도 안 온다. T0 부터: ./tools/link_check.sh")
        return 1
    print(f"    ✓ 앱 = {addr[0]}:{addr[1]}\n")

    ch = CH_YAW if args.yaw else CH_PITCH
    if args.angle is not None:
        seq = [(f"{'yaw' if args.yaw else 'pitch'}", args.angle)]
    elif args.yaw:
        seq = [("yaw +30°", 30.0), ("yaw -30°", -30.0)]
    else:
        seq = [("수평 0°", 0.0), ("아래 -45°", -45.0),
               ("수직 -90°", -90.0), ("수평 복귀", 0.0)]

    print("[2] 짐벌을 움직인다 — **카메라를 눈으로 보세요**")
    if not args.yaw:
        print("    -90° 에서 렌즈가 똑바로 땅을 봐야 한다\n")
    else:
        print("    ⚠ yaw 는 RELATIVE_ANGLE 이라 누적된다. 원위치는 손으로 맞춰야 한다\n")

    ok = 0
    for label, angle in seq:
        if step(link, label, ch, angle):
            ok += 1
        time.sleep(1.0)

    print(f"\n=== 판정 ===")
    if ok == len(seq):
        print(f"  ✓ {ok}/{len(seq)} 전부 ACCEPTED — **업링크가 열려 있다**")
        print("    PC → 앱 → DJI SDK → 기체 하드웨어 경로 전부 검증됨.")
        print("    다음: T4 영상(gst) · T5 cmd_vel(SIM 먼저)")
    elif ok:
        print(f"  ⚠ {ok}/{len(seq)} 만 통과 — 위의 결과코드를 보라")
    else:
        print("  ✗ 하나도 안 됐다.")
        print("    ACK 가 아예 없으면 업링크 자체가 막힌 것 → tools/uplink_test.py 로 분리")
        print("    ACK 는 오는데 거부면 짐벌/기체 문제 → 앱 화면의 DJI 로그 패널 확인")
    return 0 if ok == len(seq) else 1


if __name__ == "__main__":
    sys.exit(main())

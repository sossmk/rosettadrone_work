#!/usr/bin/env python3
"""AprilTag 가 왜 안 잡히나 — 사진·영상에서 태그를 찾아 그 이유를 말해 준다.

    python3 tools/tag_check.py file 사진.jpg           # 사진 한 장
    python3 tools/tag_check.py file 사진.jpg --sweep   # decimate 를 바꿔 가며
    python3 tools/tag_check.py live                    # /image_raw 구독 (브리지가 켜져 있어야 한다)

`family`(태그 규격) 는 AprilTag 의 인코딩 이름이다 — 36h11 은 6x6 비트에 최소 해밍거리 11.
**규격이 다르면 같은 그림이라도 검출기가 아예 못 읽는다.** 굴러다니는 태그가 36h11 이 아니면
`config/tags.yaml` 의 `family` 를 여기서 나온 이름으로 바꿔야 한다.

내는 값
    family · id     tags.yaml 의 family 를 정한다
    중심 (px)       화면 어디에 있나
    회전각 (deg)    똑바로 선 태그가 0°, 화면 기준 시계 +.
                    tag_follow_node 의 yaw 오차와 같은 양이라 부호 확인에 그대로 쓴다
    크기 (px)       진단용 — 안 잡힐 때 "너무 작아서"인지 가리는 값. 임무에는 안 쓴다

apriltag_ros 의 `detector.decimate` 는 검출 전에 영상을 1/N 로 줄인다. --sweep 은 그걸
흉내 내서 "어느 decimate 까지 잡히나" 를 보여 준다 — tags.yaml 에 넣을 값을 그대로 고른다.
"""
import argparse
import math
import os
import sys

import cv2

FAMILIES = [
    ("36h11", "DICT_APRILTAG_36h11"),
    ("36h10", "DICT_APRILTAG_36h10"),
    ("25h9", "DICT_APRILTAG_25h9"),
    ("16h5", "DICT_APRILTAG_16h5"),
]
MIN_PX = 20          # 이보다 작으면 "너무 작다" 로 본다 (경험칙, 실기로 고칠 것)


def _params():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


def detect_all(gray):
    """네 규격을 모두 돌려 [(family, id, 중심, 회전각deg, 크기px), ...]."""
    out = []
    p = _params()
    for name, dict_name in FAMILIES:
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
        if ids is None:
            continue
        for c, i in zip(corners, ids.ravel()):
            q = c.reshape(4, 2)
            cx, cy = q.mean(axis=0)
            dx, dy = q[1] - q[0]                   # 코너 0 → 1
            # 화면 y 가 아래라 시계가 +. OpenCV 의 AprilTag 사전은 코너 순서가 ArUco 와
            # 180° 돌아가 있어서, 똑바로 선 태그가 180° 로 나온다(우리 PNG 로 실측).
            # 180 을 더해 되돌려 **똑바로 선 태그 = 0°** 로 맞춘다
            ang = (math.degrees(math.atan2(dy, dx)) + 180.0 + 180.0) % 360.0 - 180.0
            side = sum(float(cv2.norm(q[k] - q[(k + 1) % 4])) for k in range(4)) / 4.0
            out.append((name, int(i), (float(cx), float(cy)), ang, side))
    return out


def show(found, w, h, indent=""):
    if not found:
        print(f"{indent}✗ 네 규격(36h11·36h10·25h9·16h5) 어디에도 안 잡힌다")
        return
    for fam, i, (cx, cy), ang, side in found:
        note = ""
        if side < MIN_PX:
            note = f"  ⚠ {side:.0f}px — 너무 작다 (decimate 를 내리거나 더 가까이)"
        print(f"{indent}✓ family {fam} · id {i} · 중심 ({cx:.0f},{cy:.0f})/{w}x{h} · "
              f"회전 {ang:+.1f}° · 크기 {side:.0f}px{note}")


def blur_score(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def cmd_file(args):
    img = cv2.imread(os.path.expanduser(args.path))
    if img is None:
        sys.exit(f"✗ 못 읽었다: {args.path}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    h, w = gray.shape
    print(f"{args.path} — {w}x{h} · 선명도 {blur_score(gray):.0f} (낮을수록 흐리다)")

    if not args.sweep:
        show(detect_all(gray), w, h)
        return 0

    for dec in (1.0, 1.5, 2.0, 3.0, 4.0):
        small = gray if dec == 1.0 else cv2.resize(
            gray, (max(1, int(w / dec)), max(1, int(h / dec))), interpolation=cv2.INTER_AREA)
        found = detect_all(small)
        print(f"decimate {dec:>3} → {small.shape[1]}x{small.shape[0]}")
        show(found, small.shape[1], small.shape[0], indent="    ")
    return 0


def cmd_live(args):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class Watch(Node):
        def __init__(self):
            super().__init__("tag_check")
            self.bridge = CvBridge()
            self.n = 0
            self.hits = 0
            self.create_subscription(Image, args.topic, self.on_img, 10)
            self.get_logger().info(f"{args.topic} 구독 — {args.secs}초")

        def on_img(self, m):
            self.n += 1
            if self.n % max(1, args.every):
                return
            gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(m, "bgr8"), cv2.COLOR_BGR2GRAY)
            found = detect_all(gray)
            self.hits += bool(found)
            h, w = gray.shape
            print(f"[{self.n:4}] 선명도 {blur_score(gray):5.0f}")
            show(found, w, h, indent="       ")

    rclpy.init()
    node = Watch()
    try:
        end = node.get_clock().now().nanoseconds + int(args.secs * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n프레임 {node.n} · 태그가 보인 프레임 {node.hits}")
        if node.n == 0:
            print(f"✗ {args.topic} 이 안 온다 — 먼저: ros2 launch rd_bridge bridge.launch.py "
                  f"link:=false nav:=false")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description="AprilTag 검출 진단 (규격·위치·회전)")
    sub = ap.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("file", help="사진 한 장에서 찾는다")
    p.add_argument("path", help="이미지 파일")
    p.add_argument("--sweep", action="store_true", help="decimate 를 바꿔 가며 어디까지 잡히나")

    p = sub.add_parser("live", help="/image_raw 를 구독해서 찾는다")
    p.add_argument("--topic", default="/image_raw", help="영상 토픽 (기본 /image_raw)")
    p.add_argument("--secs", type=float, default=15.0, help="몇 초 볼까 (기본 15)")
    p.add_argument("--every", type=int, default=10, help="몇 프레임마다 검사할까 (기본 10)")

    args = ap.parse_args()
    return {"file": cmd_file, "live": cmd_live}[args.mode](args)


if __name__ == "__main__":
    sys.exit(main())

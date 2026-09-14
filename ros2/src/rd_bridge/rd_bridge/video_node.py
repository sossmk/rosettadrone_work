#!/usr/bin/env python3
"""영상 발행 노드 — 기체의 H.264 스트림(또는 저장한 파일)을 sensor_msgs/Image 로 낸다.

    ros2 run rd_bridge video_node                                   # 실시간 (UDP 5600)
    ros2 run rd_bridge video_node --ros-args -p source:=~/drone_ws/capture/mavic2-real-20260910.h264

**드론 없이 개발할 수 있다.** 2026-09-10 에 받아 둔 진짜 스트림 파일을 그대로 먹이면
실시간과 같은 토픽이 나온다 — 태그 검출(할일.md E)·추종(F)은 이 모드로 만든다.

파라미터
    source      "udp" 또는 파일 경로            (기본 udp)
    port        UDP 포트                        (기본 5600 — 앱 설정의 Video port)
    topic       발행 토픽                       (기본 image_raw)
    frame_id    헤더 frame_id                   (기본 camera)
    loop        파일 끝에서 처음으로 되감기      (기본 True)
    fps         파일 재생 속도, 0=파일값 그대로  (기본 0.0)
    compressed  CompressedImage(jpeg)도 발행     (기본 False)
    jpeg_quality                                (기본 80)

────────────────────────────────────────────────────────────────
왜 GStreamer 파이프라인을 직접 짜지 않고 OpenCV 로 받는가
────────────────────────────────────────────────────────────────
실시간 경로는 `udpsrc ! rtph264depay ! avdec_h264` 로, video_view.sh 가 9/10 에
실기로 확인한 것과 같은 파이프라인이다. h264parse 가 필요 없다 —
gstreamer1.0-plugins-bad 를 안 깔아도 된다(할일.md D 의 "없는 것" 목록은 이제
실시간·파일 둘 다 해당 없음). 파일 경로는 OpenCV 자체 디코더(FFMPEG)가 raw
Annex-B .h264 를 바로 읽는다 — 여기도 추가 설치가 없다.

⚠ 손실 3% (9/10 실측). 프레임이 깨져도 노드가 죽지 않아야 한다 — 디코더가 못 낸
프레임은 그냥 건너뛰고 카운터만 올린다. 태그 검출도 이걸 견뎌야 한다.

⚠ CameraInfo 는 발행하지 않는다. solvePnP 로 태그의 상대 위치를 뽑으려면 내부
파라미터 보정이 먼저 필요한데(할일.md E), 보정 안 된 값을 그럴듯하게 흘려보내면
나중에 거리가 왜 안 맞는지 찾기 어려워진다. 보정을 하고 나서 여기에 붙인다.
"""
import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

# video_view.sh 가 9/10 에 실기로 확인한 것과 같은 구성. caps 를 명시하는 이유는
# udpsrc 가 빈 포트에서도 조용히 기다리기만 해서, 형식을 못 맞추면 "왜 안 나오지" 로
# 시간을 버리기 때문이다.
LIVE_PIPELINE = (
    "udpsrc port={port} "
    "caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,"
    "encoding-name=(string)H264,payload=(int)96 "
    "! rtph264depay ! avdec_h264 "
    "! videoconvert ! video/x-raw,format=BGR "
    "! appsink drop=true max-buffers=2 sync=false"
)


class VideoNode(Node):

    def __init__(self):
        super().__init__("rd_video")

        self.declare_parameter("source", "udp")
        self.declare_parameter("port", 5600)
        self.declare_parameter("topic", "image_raw")
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("loop", True)
        self.declare_parameter("fps", 0.0)
        self.declare_parameter("compressed", False)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("warn_after", 5.0)

        g = self.get_parameter
        self.source = os.path.expanduser(g("source").value)
        self.port = g("port").value
        self.frame_id = g("frame_id").value
        self.loop = g("loop").value
        self.fps = g("fps").value
        self.warn_after = g("warn_after").value
        self.jpeg_quality = g("jpeg_quality").value

        topic = g("topic").value
        self.pub = self.create_publisher(Image, topic, 10)
        self.pub_c = None
        if g("compressed").value:
            self.pub_c = self.create_publisher(CompressedImage, topic + "/compressed", 10)

        self.bridge = CvBridge()
        self.is_live = self.source == "udp"
        self.frames = 0        # 발행한 프레임
        self.misses = 0        # 디코더가 못 낸 read (손실·깨짐)
        self.stop = threading.Event()

        if self.is_live:
            self.get_logger().info(
                f"실시간: UDP {self.port} ← 앱 설정의 Video port. "
                f"안 나오면 ./tools/video_view.sh 로 먼저 배관을 판정할 것")
        elif not os.path.exists(self.source):
            raise SystemExit(f"파일이 없다: {self.source}")
        else:
            mb = os.path.getsize(self.source) / 1e6
            self.get_logger().info(f"파일 재생: {self.source} ({mb:.1f}MB, loop={self.loop})")

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ 열기
    def _open(self):
        if self.is_live:
            return cv2.VideoCapture(LIVE_PIPELINE.format(port=self.port), cv2.CAP_GSTREAMER)
        return cv2.VideoCapture(self.source)

    # -------------------------------------------------------------- 읽기 루프
    def _reader(self):
        cap = self._open()
        if not cap.isOpened():
            self.get_logger().error(
                "영상 소스를 열지 못했다. "
                + ("GStreamer 파이프라인 실패 — gst-inspect-1.0 avdec_h264 로 확인"
                   if self.is_live else f"파일을 열 수 없다: {self.source}"))
            self.stop.set()
            return

        # 파일 재생 속도.
        # ⚠ raw Annex-B .h264 에는 fps 정보가 없다 — 컨테이너가 없으니 타임스탬프도
        #   없다. OpenCV 는 이때 25.0 을 내놓는데 그건 그냥 기본값이고, 9/10 실측
        #   스트림은 31.9fps 였다. 그래서 raw 확장자는 실측값을 기본으로 쓴다.
        delay = 0.0
        if not self.is_live:
            raw = os.path.splitext(self.source)[1].lower() in (".h264", ".264", ".bin")
            if self.fps:
                src_fps, why = self.fps, "파라미터"
            elif raw:
                src_fps, why = 31.9, "9/10 실측(raw 파일엔 fps 정보가 없다)"
            else:
                src_fps, why = cap.get(cv2.CAP_PROP_FPS) or 30.0, "파일값"
            if src_fps <= 1:
                src_fps, why = 30.0, "기본값"
            delay = 1.0 / src_fps
            self.get_logger().info(f"재생 속도 {src_fps:.1f}fps ({why})")

        t_last_frame = time.time()
        t_last_log = time.time()
        t_last_warn = time.time()
        n_at_log = 0
        logged = 0
        nxt_deadline = time.time()

        while not self.stop.is_set():
            ok, frame = cap.read()

            if not ok:
                if self.is_live:
                    # 실시간은 아직 안 왔을 뿐일 수 있다 — 죽이지 않고 계속 기다린다
                    self.misses += 1
                    now = time.time()
                    if now - t_last_frame > self.warn_after and now - t_last_warn > self.warn_after:
                        t_last_warn = now
                        self.get_logger().warning(
                            f"{now - t_last_frame:.0f}초간 프레임이 없다. 앱 설정을 볼 것 — "
                            "Use GCS on an external device · GCS IP(핫스팟이 접속마다 IP를 바꾼다) · "
                            f"Enable video feed · Video port={self.port}. "
                            "판정은 ./tools/video_view.sh")
                    time.sleep(0.02)
                    continue

                # 파일 끝
                if self.loop:
                    cap.release()
                    cap = self._open()
                    continue
                self.get_logger().info(f"파일 끝. {self.frames} 프레임 발행함")
                self.stop.set()
                break

            t_last_frame = time.time()
            stamp = self.get_clock().now().to_msg()

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

            if self.pub_c is not None:
                ok_enc, buf = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                if ok_enc:
                    cmsg = CompressedImage()
                    cmsg.header.stamp = stamp
                    cmsg.header.frame_id = self.frame_id
                    cmsg.format = "jpeg"
                    cmsg.data = buf.tobytes()
                    self.pub_c.publish(cmsg)

            self.frames += 1
            if logged == 0:
                h, w = frame.shape[:2]
                self.get_logger().info(f"첫 프레임 {w}x{h} — 발행 시작")
                logged = 1

            now = time.time()
            if now - t_last_log >= 5.0:
                fps_now = (self.frames - n_at_log) / (now - t_last_log)
                self.get_logger().info(
                    f"{self.frames} 프레임 발행 · {fps_now:.1f}fps · 놓친 read {self.misses}")
                t_last_log, n_at_log = now, self.frames

            if delay:
                # 디코딩·발행에 쓴 시간을 뺀다. 그냥 sleep(delay) 하면 실제 발행이
                # 21fps 로 처졌다 (첫 측정에서 확인).
                nxt_deadline += delay
                slack = nxt_deadline - time.time()
                if slack > 0:
                    time.sleep(slack)
                else:
                    nxt_deadline = time.time()     # 못 따라가면 밀린 빚은 버린다

        cap.release()

    def destroy_node(self):
        self.stop.set()
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = VideoNode()
    try:
        while rclpy.ok() and not node.stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(f"\n총 {node.frames} 프레임 발행 · 놓친 read {node.misses}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

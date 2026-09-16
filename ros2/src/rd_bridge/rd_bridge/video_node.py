#!/usr/bin/env python3
"""영상 발행 노드 — 기체의 H.264 스트림(또는 저장한 파일)을 sensor_msgs/Image 로 낸다.

    ros2 run rd_bridge video_node                                   # 실시간 (UDP 5600)
    ros2 run rd_bridge video_node --ros-args -p source:=~/drone_ws/capture/mavic2-real-20260910.h264

영상이 안 나오면 이 노드를 끄고 창으로 먼저 본다 (5600 은 한 프로세스만 잡는다):

    gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" \\
      ! rtpjitterbuffer latency=120 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false

창이 안 뜨면 앱 쪽 문제(GCS IP · Enable video feed · Video port), 뜨면 이 노드 쪽 문제다.

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
실시간 경로는 `udpsrc ! rtph264depay ! avdec_h264` 로, 9/10 에 실기로 확인한
위의 창 띄우기와 같은 파이프라인이다. h264parse 가 필요 없다 —
gstreamer1.0-plugins-bad 를 안 깔아도 된다(할일.md D 의 "없는 것" 목록은 이제
실시간·파일 둘 다 해당 없음). 파일 경로는 OpenCV 자체 디코더(FFMPEG)가 raw
Annex-B .h264 를 바로 읽는다 — 여기도 추가 설치가 없다.

⚠ 손실 3% (9/10 실측). 프레임이 깨져도 노드가 죽지 않아야 한다 — 디코더가 못 낸
프레임은 그냥 건너뛰고 카운터만 올린다. 태그 검출도 이걸 견뎌야 한다.

⚠ /camera_info 는 보정 파일이 없으면 **대략값**이다 (2026-09-15, AprilTag 청사진용).
Mavic 2 Pro 대각 화각 77° 에서 계산한 fx=fy=1250, 중심=프레임 가운데, 왜곡 0 을 낸다.
태그 거리는 ±10% 정도 틀릴 수 있다.

    # 체커보드로 보정한 뒤 (cameracalibrator 가 낸 ost.yaml 을 그대로 쓴다)
    ros2 run rd_bridge video_node --ros-args -p camera_info_file:=<경로>/camera.yaml

⚠ apriltag_ros 는 영상과 /camera_info 를 **쌍으로** 구독한다(image_transport::CameraSubscriber).
camera_info 가 안 나오면 검출 콜백 자체가 안 돌아 태그가 하나도 안 잡힌다.
"""
import os
import threading
import time

import cv2
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

# 9/10 에 실기로 확인한 구성 (docstring 의 창 띄우기와 같다). caps 를 명시하는 이유는
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
        self.declare_parameter("camera_info_file", "")   # 보정 결과 YAML. 있으면 이게 이긴다
        self.declare_parameter("rectify", False)         # 왜곡 보정 후 발행 (보정 파일 필요)
        self.declare_parameter("fx", 1250.0)     # 대략값 — 보정 전 (docstring 참고)
        self.declare_parameter("fy", 1250.0)
        self.declare_parameter("cx", 0.0)        # 0 = 프레임 가운데
        self.declare_parameter("cy", 0.0)

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
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", 10)
        self.info = None       # 첫 프레임 크기로 만든다
        self.remap = None      # rectify 일 때 (map1, map2)

        self.bridge = CvBridge()
        self.is_live = self.source == "udp"
        self.frames = 0        # 발행한 프레임
        self.misses = 0        # 디코더가 못 낸 read (손실·깨짐)
        self.stop = threading.Event()

        if self.is_live:
            self.get_logger().info(
                f"실시간: UDP {self.port} ← 앱 설정의 Video port. "
                f"안 나오면 이 노드를 끄고 docstring 의 gst-launch 로 창부터 띄워 볼 것")
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
                            "판정은 docstring 의 gst-launch 창 띄우기")
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

            if self.info is None:
                self.info = self._camera_info(frame.shape[1], frame.shape[0])
            if self.remap is not None:
                frame = cv2.remap(frame, self.remap[0], self.remap[1], cv2.INTER_LINEAR)

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

            self.info.header = msg.header          # 이미지와 같은 stamp 여야 apriltag 가 짝을 맞춘다
            self.pub_info.publish(self.info)

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

    def _info_from_file(self, path, w, h):
        """cameracalibrator 가 내는 ROS camera_info YAML(ost.yaml) 을 그대로 읽는다."""
        try:
            with open(path) as f:
                d = yaml.safe_load(f)
            m = CameraInfo()
            m.width = int(d.get("image_width", w))
            m.height = int(d.get("image_height", h))
            m.distortion_model = d.get("distortion_model", "plumb_bob")
            m.d = [float(x) for x in d["distortion_coefficients"]["data"]]
            m.k = [float(x) for x in d["camera_matrix"]["data"]]
            m.r = [float(x) for x in d["rectification_matrix"]["data"]]
            m.p = [float(x) for x in d["projection_matrix"]["data"]]
        except Exception as e:
            self.get_logger().error(
                f"camera_info_file 을 못 읽었다: {path} ({e}) — 대략값으로 간다")
            return None

        if (m.width, m.height) != (w, h):
            # 고쳐서 쓰지 않는다 — 어림으로 늘린 값은 틀린 줄도 모르고 쓰게 된다
            self.get_logger().error(
                f"보정 해상도 {m.width}x{m.height} 가 지금 영상 {w}x{h} 와 다르다 — "
                f"거리가 그만큼 틀린다. 같은 해상도로 다시 보정할 것")
        self.get_logger().info(
            f"보정값 {path} — fx={m.k[0]:.1f} fy={m.k[4]:.1f} "
            f"cx={m.k[2]:.1f} cy={m.k[5]:.1f} 왜곡 {len(m.d)}개")

        if self.get_parameter("rectify").value:
            self._make_remap(m, w, h)
        return m

    def _make_remap(self, m, w, h):
        """왜곡을 펴서 내보낸다 — apriltag_ros 는 입력이 이미 펴진 영상이라고 보고 푼다.

        image_proc/rectify_node 를 쓰지 않는 이유: 그건 영상만 펴고 camera_info 는 그대로
        내보내서, 편 영상에 안 편 K·D 가 붙는다. 여기서 같이 바꿔야 앞뒤가 맞는다.
        """
        import numpy as np
        k = np.array(m.k, dtype=np.float64).reshape(3, 3)
        d = np.array(m.d, dtype=np.float64)
        if not d.any():
            self.get_logger().info("rectify — 왜곡계수가 0 이라 할 일이 없다")
            return
        new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 0.0, (w, h))
        self.remap = cv2.initUndistortRectifyMap(k, d, None, new_k, (w, h), cv2.CV_16SC2)
        m.k = [float(v) for v in new_k.ravel()]
        m.d = [0.0] * len(m.d)
        m.p = [new_k[0, 0], 0.0, new_k[0, 2], 0.0,
               0.0, new_k[1, 1], new_k[1, 2], 0.0,
               0.0, 0.0, 1.0, 0.0]
        self.get_logger().info(
            f"rectify 켬 — 편 뒤 fx={m.k[0]:.1f} fy={m.k[4]:.1f} "
            f"cx={m.k[2]:.1f} cy={m.k[5]:.1f}, 왜곡 0 으로 발행")

    def _camera_info(self, w, h):
        path = os.path.expanduser(self.get_parameter("camera_info_file").value)
        if path:
            m = self._info_from_file(path, w, h)
            if m is not None:
                return m

        g = self.get_parameter
        fx, fy = float(g("fx").value), float(g("fy").value)
        cx, cy = float(g("cx").value) or w / 2.0, float(g("cy").value) or h / 2.0
        self.get_logger().warning(
            f"camera_info 대략값 fx={fx:.0f} fy={fy:.0f} cx={cx:.0f} cy={cy:.0f} — 보정 전, 거리 ±10%")
        if self.get_parameter("rectify").value:
            self.get_logger().warning(
                "rectify:=true 인데 보정 파일이 없다 — 펼 왜곡값을 모르니 그냥 원본을 낸다")
        m = CameraInfo()
        m.width, m.height = w, h
        m.distortion_model = "plumb_bob"
        m.d = [0.0] * 5
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

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

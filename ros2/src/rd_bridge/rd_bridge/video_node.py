#!/usr/bin/env python3
"""영상 노드 — 기체의 H.264(UDP 5600) 또는 저장한 파일을 JPEG · /camera_info 로 낸다.

    ros2 run rd_bridge video_node                                   # 실시간
    ros2 run rd_bridge video_node --ros-args -p source:=~/drone_ws/capture/mavic2-real-20260910.h264
    ros2 run rd_bridge video_node --ros-args -p camera_info_file:=<경로>/camera.yaml -p rectify:=true

영상이 안 나오면 이 노드를 끄고 창으로 먼저 본다 (5600 은 한 프로세스만 잡는다):

    gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" \\
      ! rtpjitterbuffer latency=120 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false

창이 안 뜨면 앱 쪽 문제(GCS IP · Enable video feed · Video port), 뜨면 이 노드 쪽 문제다.

파라미터
    source            "udp" 또는 .h264 파일 경로     (기본 udp)
    port              UDP 포트                       (기본 5600)
    loop · fps        파일 되감기 · 재생 속도(0=자동)
    camera_info_file  보정 YAML (cameracalibrator 의 ost.yaml 형식)
    rectify           왜곡을 펴서 낸다 (보정 파일이 있어야 뜻이 있다)
    jpeg_quality      JPEG 품질 1~100 (기본 80). 태그 검출이 나쁘면 올린다
    publish_fps       발행 상한 Hz (기본 3). 0 이면 오는 대로 다 낸다(~30Hz)
    scale             축소 배율 0<s<=1 (기본 0.5). camera_info 도 같이 줄인다
    fx fy cx cy       보정 파일이 없을 때의 대략값 — 거리가 ±10% 틀린다

⚠ raw(Image) 는 발행하지 않는다 — 1.2Gbps 라 누가 구독하면 회선이 죽는다. JPEG 만 낸다.
  apriltag 도 image_transport:=compressed 로 같은 토픽을 구독해 로컬에서 푼다.
⚠ apriltag_ros 는 영상과 /camera_info 를 **쌍으로** 구독한다. camera_info 가 없으면 검출이 0 이다.
⚠ 프레임 손실(~3%)이 있다. 디코더가 못 낸 프레임은 건너뛰고 세기만 한다.
"""
import os
import threading
import time

import cv2
import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

FRAME_ID = "camera"      # /tf 의 부모 프레임 (camera → tag0)
# caps 를 적어 두는 이유 — 없으면 udpsrc 가 형식을 몰라 조용히 기다리기만 한다
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
        for name, val in [("source", "udp"), ("port", 5600), ("loop", True), ("fps", 0.0),
                          ("warn_after", 5.0), ("camera_info_file", ""), ("rectify", False),
                          ("jpeg_quality", 80), ("publish_fps", 3.0), ("scale", 0.5),
                          ("fx", 1250.0), ("fy", 1250.0), ("cx", 0.0), ("cy", 0.0)]:
            self.declare_parameter(name, val)
        g = self.get_parameter
        self.source = os.path.expanduser(g("source").value)
        self.port = g("port").value
        self.loop = g("loop").value
        self.fps = g("fps").value
        self.warn_after = g("warn_after").value
        self.jpeg_quality = int(g("jpeg_quality").value)
        self.publish_fps = float(g("publish_fps").value)
        self.next_pub = 0.0    # publish_fps 게이트의 다음 발행 시각
        self.scale = float(g("scale").value)
        if not 0.0 < self.scale <= 1.0:
            self.get_logger().warning(
                f"scale={self.scale} 은 (0, 1] 밖이다 — 1.0(원본)으로 둔다. 확대는 대역폭에 의미가 없다")
            self.scale = 1.0

        # ⚠ raw(Image) 는 **발행하지 않는다.** 1632x1088 bgr8 30Hz = 1.2Gbps 라
        # 관제 PC 에서 누가 목록에서 집는 순간 기가비트 회선이 죽는다 (9/22 실제 발생).
        # 토픽이 아예 없으면 그 사고가 불가능하다. 여기서 바로 JPEG 으로 내고,
        # apriltag 는 image_transport:=compressed 로 같은 토픽을 구독해 로컬에서 푼다.
        # ⚠ RELIABLE 로 낸다 — apriltag_node 구독이 RELIABLE 고정이고 qos_overrides 를
        # 선언하지 않아 맞출 방법이 없다. 관제 PC 는 BEST_EFFORT 로 붙으면 되고,
        # 그 리더에게는 재전송이 안 나간다 (리더별로 따로 처리된다).
        self.pub = self.create_publisher(CompressedImage, "cam/compressed", 10)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", 10)
        self.info = None       # 첫 프레임 크기로 만든다
        self.remap = None      # rectify 일 때 (map1, map2)

        self.is_live = self.source == "udp"
        self.frames = 0        # 발행한 프레임
        self.misses = 0        # 디코더가 못 낸 read
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

        # 파일 재생 속도. raw .h264 엔 fps 정보가 없어 OpenCV 가 25 를 지어내므로 실측 31.9 를 쓴다
        delay = 0.0
        if not self.is_live:
            raw = os.path.splitext(self.source)[1].lower() in (".h264", ".264", ".bin")
            src_fps = self.fps or (31.9 if raw else cap.get(cv2.CAP_PROP_FPS))
            if src_fps <= 1:
                src_fps = 30.0
            delay = 1.0 / src_fps
            self.get_logger().info(f"재생 속도 {src_fps:.1f}fps")

        t_last_frame = t_last_log = t_last_warn = nxt_deadline = time.time()
        n_at_log = 0

        while not self.stop.is_set():
            ok, frame = cap.read()

            if not ok:
                if self.is_live:
                    # 아직 안 왔을 뿐일 수 있다 — 죽지 않고 기다린다
                    self.misses += 1
                    now = time.time()
                    if now - t_last_frame > self.warn_after and now - t_last_warn > self.warn_after:
                        t_last_warn = now
                        self.get_logger().warning(
                            f"{now - t_last_frame:.0f}초간 프레임이 없다. 앱 설정을 볼 것 — "
                            "Use GCS on an external device · GCS IP(핫스팟이 접속마다 IP를 바꾼다) · "
                            f"Enable video feed · Video port={self.port}")
                    time.sleep(0.02)
                    continue
                if self.loop:                              # 파일 끝
                    cap.release()
                    cap = self._open()
                    continue
                self.get_logger().info(f"파일 끝. {self.frames} 프레임 발행함")
                self.stop.set()
                break

            t_last_frame = time.time()
            if self.info is None:
                self.info = self._camera_info(frame.shape[1], frame.shape[0])
                # ⚠ 축소하면 camera_info 도 같이 줄여야 앞뒤가 맞는다. rectify 가 K 를
                #   고쳐 쓴 **뒤**에 적용해야 하므로 여기다 (그 전에 하면 덮어써진다).
                self._scale_info(self.info, self.scale)
                self.get_logger().info(
                    f"첫 프레임 {frame.shape[1]}x{frame.shape[0]} → "
                    f"{self.info.width}x{self.info.height} 발행 시작 (scale={self.scale})")

            # 발행 상한. H.264 는 프레임을 건너뛰고 못 푸니 디코딩은 다 하되 여기서 버린다.
            # 이 자리인 이유 — 아래 remap · imencode · publish 가 통째로 생략돼
            # 인코딩 CPU 와 회선 대역폭이 같이 준다. camera_info 도 같은 게이트 뒤라
            # 영상과 짝·stamp 가 저절로 유지된다.
            # 밀린 빚은 쌓지 않는다 (실시간은 따라잡을 대상이 없다).
            if self.publish_fps > 0:
                if t_last_frame < self.next_pub:
                    continue
                self.next_pub = t_last_frame + 1.0 / self.publish_fps

            if self.remap is not None:
                frame = cv2.remap(frame, self.remap[0], self.remap[1], cv2.INTER_LINEAR)

            # 축소는 remap **뒤**다 — 보정 맵이 원본 해상도 기준으로 만들어져 있다.
            # INTER_AREA 는 축소 전용으로, 줄일 때 모아레가 안 생긴다.
            if self.scale != 1.0:
                frame = cv2.resize(frame, None, fx=self.scale, fy=self.scale,
                                   interpolation=cv2.INTER_AREA)

            ok_enc, buf = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok_enc:
                self.misses += 1
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = FRAME_ID
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            self.pub.publish(msg)
            self.info.header = msg.header          # 같은 stamp 여야 apriltag 가 짝을 맞춘다
            self.pub_info.publish(self.info)
            self.frames += 1

            now = time.time()
            if now - t_last_log >= 5.0:
                fps_now = (self.frames - n_at_log) / (now - t_last_log)
                self.get_logger().info(
                    f"{self.frames} 프레임 발행 · {fps_now:.1f}fps · 놓친 read {self.misses}")
                t_last_log, n_at_log = now, self.frames

            if delay:
                # 디코딩에 쓴 시간을 빼고 잔다. 못 따라가면 밀린 빚은 버린다
                nxt_deadline += delay
                slack = nxt_deadline - time.time()
                if slack > 0:
                    time.sleep(slack)
                else:
                    nxt_deadline = time.time()

        cap.release()

    # ------------------------------------------------------------ camera_info
    def _scale_info(self, m, s):
        """camera_info 를 영상과 같은 배율로 줄인다.

        초점거리(fx·fy)와 주점(cx·cy)은 픽셀 단위라 해상도에 비례한다 — 안 줄이면
        거리·좌표가 배율만큼 틀린다. 반대로 D(왜곡계수)와 R(회전)은 정규화 값이라
        해상도와 무관하므로 건드리지 않는다.
        """
        if s == 1.0:
            return
        m.width = int(round(m.width * s))
        m.height = int(round(m.height * s))
        k = list(m.k)
        for i in (0, 2, 4, 5):          # fx, cx, fy, cy
            k[i] *= s
        m.k = k
        p = list(m.p)
        for i in (0, 2, 5, 6):          # fx, cx, fy, cy
            p[i] *= s
        m.p = p

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
        if g("rectify").value:
            self.get_logger().warning("rectify:=true 인데 보정 파일이 없다 — 원본을 낸다")
        m = CameraInfo()
        m.width, m.height = w, h
        m.distortion_model = "plumb_bob"
        m.d = [0.0] * 5
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def _info_from_file(self, path, w, h):
        """cameracalibrator 의 ost.yaml 형식을 읽는다."""
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
            self.get_logger().error(f"camera_info_file 을 못 읽었다: {path} ({e}) — 대략값으로 간다")
            return None

        if (m.width, m.height) != (w, h):
            # 늘려서 쓰지 않는다 — 틀린 줄 모르고 쓰게 된다
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
        """왜곡을 펴고 camera_info 도 편 값(K 새로, D=0)으로 바꾼다.

        image_proc 을 안 쓰는 이유 — 영상만 펴고 camera_info 는 그대로 내서 앞뒤가 안 맞는다.
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

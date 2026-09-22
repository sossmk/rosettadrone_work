# 관제 PC 에서 드론 영상 보기

RC 노트북이 내보내는 토픽:

    /gcs/daiv/cam/compressed    sensor_msgs/msg/CompressedImage    JPEG, ~30Hz, ~46Mbps

## ⚠ rqt_image_view 에서 이 토픽을 직접 고르면 죽는다

    create_subscription() called for existing topic name rt/gcs/daiv/cam/compressed
    with incompatible type sensor_msgs::msg::dds_::CompressedImage_
    terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'

image_transport 규칙상 `/gcs/daiv/cam/compressed` 는 "베이스 `/gcs/daiv/cam` + transport `compressed`"
라는 뜻이다. rqt 는 이걸 베이스 토픽으로 착각해 raw(Image) 로 구독하려다 타입이 안 맞아 죽는다.
베이스 `/gcs/daiv/cam` 은 광고되지 않으므로 목록에서 고를 수도 없다.

## 해결 — 로컬에서 한 번 풀고 본다

관제 PC 에서 이것만 실행한다 (설치할 것 없음, 표준 패키지):

    ros2 run image_transport republish --ros-args -r __node:=gcs_view \
      -p in_transport:=compressed -p out_transport:=raw \
      -r in:=/gcs/daiv/cam -r in/compressed:=/gcs/daiv/cam/compressed \
      -r out:=/gcs/view

그 다음 rqt_image_view 에서 `/gcs/view` 를 고르면 정상으로 보인다.
`/gcs/view` 는 관제 PC 안에서만 도는 raw 라 랜선을 타지 않는다.

⚠ `-r in:=/gcs/daiv/cam` 만 주면 안 된다. `-r in/compressed:=...` 를 반드시 같이 준다
  (베이스만 remap 하면 구독 자체가 안 만들어진다 — 실측 확인).

## 절대 하지 말 것

`/rc/image_raw` 를 구독하지 말 것. 1632x1088 bgr8 30Hz = **1.2Gbps** 로, 기가비트 회선
용량을 넘는다. 붙는 순간 RC 노트북의 영상이 1Hz 로 주저앉고 드론 링크까지 불안정해진다.
(2026-09-22 실제 발생 — 원격 rqt 3개가 붙어 TX 3.7GB, RX drop 31만)

## MAIV 커스텀 메시지가 안 읽히면

    TopicInfo.__init__(): can not get message class for type "asv_micom/msg/Bms"

메시지 패키지를 source 안 한 것이다. RC 노트북 기준:

    source ~/maiv_ws/install/setup.bash

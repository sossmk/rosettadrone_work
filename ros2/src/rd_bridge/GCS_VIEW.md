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

## 타입이 둘로 보이는 건 발행 탓이 아니다

    /gcs/daiv/cam/compressed [sensor_msgs/msg/CompressedImage, sensor_msgs/msg/Image]

`ros2 topic list -t` 는 그 이름에 걸린 **퍼블리셔와 구독자의 타입을 전부 합쳐서** 보여준다.
우리 퍼블리셔는 언제나 `CompressedImage` 하나다. 뒤에 붙은 `Image` 는 위의 rqt_image_view 가
만든 **잘못된 구독**이 올린 것이고, 그 rqt 를 닫으면 사라진다. 확인은 엔드포인트별로 본다:

    ros2 topic info -v /gcs/daiv/cam/compressed     # Publisher 쪽 Topic type 만 보면 된다

⚠ `ros2 topic list` 는 daemon 캐시를 본다. 구독자를 떼고도 그대로면 `ros2 daemon stop` 후 다시 볼 것
  (실측 — 캐시 때문에 한참 둘로 남아 있었다).

## 해결 — 토픽을 명령줄에 직접 준다

관제 PC 에서 이 한 줄이면 된다:

    ros2 run rqt_image_view rqt_image_view /gcs/daiv/cam/compressed

토픽을 **인자로 박는 게** 핵심이다. 드롭다운에서 고르면 rqt 가 이름을 해석하다 사고를 치지만,
인자로 주면 그 과정이 없다. 실측 (jazzy, 2026-09-22) — 충돌 없이 정상 표시.

republish 로 raw 를 만들어 볼 필요는 **없다.** 그건 예전에 raw 토픽(`/gcs/daiv/cam`)을
같이 내던 시절의 우회로다. 지금은 video_node 가 JPEG 만 내므로 충돌할 상대가 없다.

### 그래도 관제 PC 에서 죽는다면 (다른 배포판·구버전 rqt)

그때만 중간에서 한 번 풀어 본다:

    ros2 run image_transport republish --ros-args -r __node:=gcs_view \
      -p in_transport:=compressed -p out_transport:=raw \
      -r in:=/gcs/daiv/cam -r in/compressed:=/gcs/daiv/cam/compressed \
      -r out:=/gcs/view

그 다음 rqt_image_view 에서 `/gcs/view` 를 고른다. 이 터미널은 켜둔 채로 쓴다 —
닫거나 Ctrl+C 하면 노드가 죽고 `/gcs/view` 도 사라진다.

⚠ `-r in:=...` 만 주면 안 된다. remap 규칙은 `in` 이라는 이름에만 걸리고 image_transport 가
  따로 만드는 `in/compressed` 에는 안 번져서, remap 안 된 생짜 `/in/compressed` 를 구독한다
  (실측 — 구독이 안 생기는 게 아니라 엉뚱한 이름에 생긴다).

⚠ republish 는 게으르다. 출력을 누가 구독할 때까지 입력을 안 받으므로, 띄운 직후
  `ros2 topic info` 에 구독자 0 으로 보여도 정상이다.

## 절대 하지 말 것

`/rc/image_raw` 를 구독하지 말 것. 1632x1088 bgr8 30Hz = **1.2Gbps** 로, 기가비트 회선
용량을 넘는다. 붙는 순간 RC 노트북의 영상이 1Hz 로 주저앉고 드론 링크까지 불안정해진다.
(2026-09-22 실제 발생 — 원격 rqt 3개가 붙어 TX 3.7GB, RX drop 31만)

⚠ 단, **그 "RX drop 31만" 은 혼잡의 증거가 아니다.** 같은 인터페이스의 `rx_errors` ·
  `rx_over_errors` · `rx_fifo_errors` · `rx_missed_errors` 가 전부 0 이고, 송신 qdisc 도
  4일간 드롭 2개 · overlimits 0 이다. `rx_dropped` 만 큰 것은 호스트가 받은 뒤 버린 것
  (들어줄 소켓이 없는 DDS 멀티캐스트 등)에 가깝다. 1.2Gbps 가 기가비트를 넘는다는 **산수는
  그대로 유효하니 구독하지 말 것**이되, 낮의 사고 원인이 정말 대역폭이었는지는 **미확정**이다
  (진행상황.md 9/22 저녁 §0). 혼잡의 진짜 증거는 **"영상을 켤 때 시스템 전체 ping 이
  튀는 것"** 이었고, 그 메커니즘은 단순 포화가 아니라 **조각 손실 + RELIABLE 재전송**이다:
  프레임 190KB 는 1500 MTU 에서 조각 137개가 되고, 조각 하나만 잃어도 프레임이 통째로
  버려진다. RELIABLE 이라 그때마다 재전송이 나가 회선을 더 막는다 — 악순환.

## MAIV 커스텀 메시지가 안 읽히면

    TopicInfo.__init__(): can not get message class for type "asv_micom/msg/Bms"

메시지 패키지를 source 안 한 것이다. RC 노트북 기준:

    source ~/maiv_ws/install/setup.bash

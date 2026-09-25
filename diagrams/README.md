# 그림 — archify 로 다시 확인하기

임무 시퀀서(`rd_mission`)를 그림 세 장으로 둔다. **스펙(JSON)이 원본**이고 HTML 은 결과물이다.
코드를 고치면 JSON 을 고치고 다시 렌더한다 — 그림이 코드와 어긋나면 그림이 틀린 것이다.

| 파일 | 무엇 |
|---|---|
| `mission-modes.lifecycle.json` → `mission-modes.html` | 7개 모드 시퀀스와 전이 조건 |
| `mission-arming.architecture.json` → `mission-arming.html` | 누가 `/gcs/daiv/cmd_vel` 의 주인이 되는가 |
| `mission-node-tick.workflow.json` → `mission-node-tick.html` | `mission_node.py` 코드 — 20Hz `tick()` 한 바퀴 안의 함수 순서 |

## 다시 렌더

```bash
A=~/.claude/skills/archify
D=~/rosettadrone_work/diagrams

node $A/bin/archify.mjs validate lifecycle     $D/mission-modes.lifecycle.json    --quality showcase --json
node $A/bin/archify.mjs deliver  lifecycle     $D/mission-modes.lifecycle.json    $D/mission-modes.html   --quality showcase
node $A/bin/archify.mjs deliver  architecture  $D/mission-arming.architecture.json $D/mission-arming.html --quality showcase
node $A/bin/archify.mjs deliver  workflow      $D/mission-node-tick.workflow.json  $D/mission-node-tick.html --quality showcase

# 브라우저 실측(오버플로·가독성) — 스크린샷이 같은 폴더에 남는다
node $A/bin/archify.mjs visual-check $D/mission-modes.html  --json
node $A/bin/archify.mjs visual-check $D/mission-arming.html --json
```

`deliver` 가 0 이 아니면 실패다 — 이전 HTML 이 그대로 남으니 **성공으로 읽지 말 것**.
`--quality showcase` 는 9개 검사와 합성 오류 0 을 요구한다.

## 고칠 때 같이 봐야 하는 것

- 모드를 늘리거나 전이를 바꿨다 → `mission-modes.lifecycle.json` 의 `states`·`transitions`,
  그리고 `mission_node.selftest()` 의 assert 3(출구 보장)
- `tick()`·`switchover()`·`go()` 의 순서를 바꿨다 → `mission-node-tick.workflow.json`
- `CTRL` 표를 바꿨다 → `mission-arming.architecture.json` 의 `arm-*` 연결
- **그림보다 먼저** `python3 ros2/src/rd_bridge/rd_bridge/mission_node.py --selftest`

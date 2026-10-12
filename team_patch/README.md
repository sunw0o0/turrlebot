# 팀 저장소용 주차 패치

## parking.patch (v1) - 팀 main 에 이미 머지됨 (PR #6)
표지판을 보면 정해진 6단계(접근 -> 90도 -> 들어가기 -> 대기 -> 나오기 -> 90도)로 주차.

## parking_v2.patch - 팀 main d749880 (PR #6 머지 후) 기준
주차 구역 방식으로 다시 만든 버전.

1. 주차 표지판 -> 주차 구역 모드. 운전은 lane_follower (왼쪽 노란선 따라가기)
2. 오른쪽 흰선이 사라지면 "구역 안"
3. 왼쪽 PSD <= 0.2m -> 오른쪽 칸에 주차 / 오른쪽 PSD <= 0.2m -> 왼쪽 칸에 주차
   90도 -> 2초 전진 -> 2초 대기 -> 180도 -> 앞 PSD 0.3m 까지 전진 -> 90도 (전부 같은 방향으로 돎)
4. 다시 왼쪽 노란선 따라가기
5. 오른쪽 흰선이 다시 보이면 주차 구간 끝 -> parking_done

| 파일 | 변경 |
|---|---|
| `interfaces/msg/LaneInfo.msg` | `left_detected`, `right_detected` 추가 |
| `vision/lane_detection/...` | 위 두 값 채우기, 노란선을 왼쪽에서만 찾는 `yellow_search_max_ratio` (기본 1.0 = 기존과 같음) |
| `lane_follower/src/parking_node.cpp`, `config/parking.yaml` | 주차 구역 방식으로 새로 작성 |
| `lane_follower/launch/*.launch.py` | `task_planner` 인자로 주차 노드 동작 방식 결정 |
| `task_planner/src/task_planner_node.cpp` | PARKING 상태에서 `parking_active` 일 때만 PARKING 모드, 아니면 LANE |

```bash
cd ~/team && git checkout main && git pull
git checkout -b parking-zone
git apply ~/turrlebot/team_patch/parking_v2.patch
cd colcon_ws && colcon build --packages-select interfaces lane_detection lane_follower task_planner turtle_gui path_planner map_planner
```

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

## construction.patch - parking_v2.patch 위에 적용
공사 표지판(sign_construction)을 보면 장애물 피하기 모드.

1. 공사 표지판이 가까이 3번 연속 -> control_mode CONSTRUCTION (lane_follower 정지, construction_node 가 운전)
2. image_bev 에서 노란선 / 흰선 / 갈색 박스를 색으로 찾음 (lane_detection, path_planner 실측값)
   - 장애물 + 차선: 장애물 양옆 빈 공간 중 넓은 쪽 가운데로 (= 장애물에 붙은 차선에서 멀어짐)
   - 로봇 중심은 항상 두 차선 안쪽 (line_keep_px)
   - 차선이 안 보이면 직진, 장애물이 없으면 차선 가운데
3. 장애물이 3초 동안 안 보이면 끝 -> LANE

| 파일 | 변경 |
|---|---|
| `interfaces/msg/ControlMode.msg` | `CONSTRUCTION = 5` 추가 |
| `lane_follower/src/construction_node.cpp`, `config/construction.yaml` | 새 파일 |
| `lane_follower/CMakeLists.txt`, `package.xml` | OpenCV, cv_bridge 의존성 |
| `lane_follower/launch/lane_follower.launch.py` | construction_node 실행 (`construction:=false` 로 끄기) |
| `task_planner` | `construction_active` 동안 CONSTRUCTION 모드 |
| `turtle_gui` | 모드 이름 CONSTRUCTION 표시 |

```bash
cd ~/team                     # 젯슨: ~/Robit_intelligence_turtlebot_team
git apply ~/turrlebot/team_patch/construction.patch
cd colcon_ws && colcon build --packages-up-to lane_follower turtle_gui
```

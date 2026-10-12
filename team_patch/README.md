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
3. 주차 표지판이 가까이 3번 연속 보이면 끝 -> LANE (주차 노드가 이어받음)
   (`exit_clear_time`, `zone_timeout` 을 0 보다 크게 하면 그 조건으로도 끝남. 기본 0 = 안 씀)

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

## lane_corner.patch - 커브에서 인코스로 파고드는 문제
lane_detection 은 offset 을 BEV 맨 아래에서 잰다 ("로봇은 BEV 맨 아래에 있다고 본다").
실제 바퀴 축은 그보다 약 335px(33cm) 뒤라서, 커브에서 앞 지점을 가운데에 맞추려고 미리 꺾는다.
`offset_eval_behind_px` (기본 0 = 기존) 만큼 곡선을 로봇 쪽으로 늘려서 offset 을 잰다.
150 -> 250 -> 335 순서로 키워 보며 맞춘다. (lane_method:=sliding 일 때만 해당)

## 주차 v4 - 주차 모드에서는 흰선 무시, 노란선만 따라가기 (파일 복사 방식)

주차 표지판을 보면 parking_node 가 직접 운전한다 (control_mode = PARKING, lane_follower 는 멈춤).
위에서 본 영상(image_bev)에서 노란색만 찾고 흰선은 아예 안 본다.

- 노란선이 양쪽에 보이면 가운데로, 한쪽만 보이면 그 선에서 차선 폭 절반만큼 떨어져서 간다.
- 노란선을 줄마다 모아 곡선으로 이어서 위치를 잰다 (가로로 누운 선 줄은 뺀다). 선이 잠깐 끊겨도 흔들리지 않는다.
- 노란선을 놓치면 마지막으로 본 쪽으로 천천히 돌며 찾는다.
- GUI STOP / MANUAL 을 누르면 멈추고 주차 모드 끝. 주차 칸에 들어가기 / 주차 모드 끝내기는 아직 없다.

| 파일 | 내용 |
|---|---|
| `parking_node.cpp` | 새로 작성 |
| `parking.yaml` | 새 파라미터 |
| `add_parking_vision_build.py` | CMakeLists.txt 의 parking_node 에 OpenCV / cv_bridge 연결 (이미 했으면 그대로 둠) |

적용 (젯슨, construction 까지 적용된 상태에서):

```bash
cd ~/Robit_intelligence_turtlebot_team
cp ~/turrlebot/team_patch/parking_node.cpp colcon_ws/src/lane_follower/src/
cp ~/turrlebot/team_patch/parking.yaml colcon_ws/src/lane_follower/config/
python3 ~/turrlebot/team_patch/add_parking_vision_build.py .
cd colcon_ws && MAKEFLAGS="-j1" colcon build --executor sequential --packages-select lane_follower
```

## 0단계: 오도메트리 확인 (odom_test.py)

엔코더 -> `/odom` 은 팀원이 `hybrid-drive` 브랜치에 이미 만들어 두었다 (커밋 2d84161, stm 패키지).
main 에는 아직 없으므로 stm 폴더만 그 브랜치에서 가져와 빌드한다 (interfaces 변경은 주석/다른 메시지라 그대로 둬도 됨).

```bash
cd ~/Robit_intelligence_turtlebot_team
git fetch origin
git checkout origin/hybrid-drive -- colcon_ws/src/stm
cd colcon_ws && MAKEFLAGS="-j1" colcon build --executor sequential --packages-select stm
```

시험 (터미널 2개, S1 켠 상태. 다른 노드는 끄고 stm_bridge 만):

```bash
# 터미널 1
ros2 launch stm stm_bridge.launch.py
# 터미널 2
python3 ~/turrlebot/team_patch/odom_test.py forward 1.0   # 엔코더 기준 1 m 직진 -> 줄자로 실제 거리
python3 ~/turrlebot/team_patch/odom_test.py turn 360      # 엔코더 기준 360도 -> 처음 방향과 비교
python3 ~/turrlebot/team_patch/odom_test.py show          # 움직이지 않고 거리/각도만 표시
```

틀리면 `colcon_ws/src/stm/config/stm_bridge.yaml` 의 `wheel_radius`(거리), `wheel_separation`(각도) 를 고친다.

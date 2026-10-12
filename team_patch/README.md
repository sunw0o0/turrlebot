# 팀 저장소용 주차 패치

팀 저장소(`lkh0320/Robit_intelligence_turtlebot_team`, `main` a2541e7 기준)에 적용하는 변경이다.

| 파일 | 내용 |
|---|---|
| `lane_follower/src/parking_node.cpp` (새 파일) | 주차 노드. 주차 표지판(또는 PARKING 모드)을 보면 접근 -> 90도 회전 -> 들어가기 -> 대기 -> 나오기 -> 원래 방향 회전 후 라인 주행으로 복귀 |
| `lane_follower/config/parking.yaml` (새 파일) | 주차 거리 / 각도 / 방향 / 속도 설정 |
| `lane_follower/CMakeLists.txt`, `package.xml` | parking_node 빌드, std_msgs 의존성 |
| `lane_follower/launch/lane_follower.launch.py` | lane_follower 와 같이 parking_node 실행 (`parking:=false` 로 끄기) |
| `task_planner/...` | TO_PARKING -> PARKING 을 주차 표지판으로 판단, `parking_done` 을 받으면 다음 구간으로 |

## 적용

```bash
cd ~/team
git pull                      # 먼저 최신(a2541e7 이후)으로
git apply ~/turrlebot/team_patch/parking.patch
cd colcon_ws && colcon build --packages-select lane_follower task_planner
```

`parking.patch` 안에 위 파일이 전부 들어 있다. `parking_node.cpp`, `parking.yaml` 은 읽기 편하라고 따로 둔 사본.

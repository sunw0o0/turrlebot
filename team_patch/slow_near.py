# (Jetson 버전 parking_node: param() / loadParams 방식)
# 주차: 일반 주행 속도로 가다가
#   (1) 통로에서 앞의 가로 흰 선이 가까워질수록 천천히 -> 회전 시작할 때는 예전 속도 (drive_speed)
#   (2) 칸에서 나와 길을 내려갈 때 왼쪽 노란선 끝이 가까워질수록 천천히 -> 끊김 볼 때는 예전 속도
#   + 나갈 때 leave_min_time 을 거리로도 본다 (빨리 가면 1초 안에 끊김을 지나쳐서 못 보던 문제)
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'slow_by_row' in ns:
    print('이미 들어가 있어요')
    sys.exit()

R = [
    ('    leave_min_time_ = param("leave_min_time", 1.0);\n',
     '    leave_min_time_ = param("leave_min_time", 1.0);\n'
     '    // 가로 흰 선 / 노란선 끝이 가까워질수록 일반 주행 속도 -> drive_speed 로 줄이기 (돌기 시작할 때는 예전처럼 drive_speed)\n'
     '    slow_near_ = param("slow_near", true);\n'
     '    stem_slow_start_y_ = param("stem_slow_start_y", 100.0);    // 가로 흰 선이 이 줄에 오면 줄이기 시작, white_trigger_y 에서 drive_speed\n'
     '    leave_slow_start_y_ = param("leave_slow_start_y", 100.0);  // 노란선 끝이 이 줄에 오면 줄이기 시작, 끊김 보는 칸 위에서 drive_speed\n'),
    ('  double leave_min_time_;\n',
     '  double leave_min_time_;\n'
     '  bool slow_near_;\n'
     '  double stem_slow_start_y_, leave_slow_start_y_;\n'
     '  double exit_travel_ = 0.0;       // EXIT_STEM 에서 간 거리 [m, 명령 단위]\n'
     '  bool slow_logged_ = false;\n'),
    ('    state_time_ += 0.1;\n',
     '    state_time_ += 0.1;\n'
     '    if (state_ == State::EXIT_STEM)\n'
     '    {\n'
     '      exit_travel_ += 0.1 * speed_;\n'
     '    }\n'),
    ('    leave_empty_ = 0;\n'
     '    RCLCPP_INFO(get_logger(), "들어왔던 길을 두 차선 가운데로 내려감',
     '    leave_empty_ = 0;\n'
     '    exit_travel_ = 0.0;\n'
     '    slow_logged_ = false;\n'
     '    RCLCPP_INFO(get_logger(), "들어왔던 길을 두 차선 가운데로 내려감'),
    ('    last_white_y_ = -1.0;\n'
     '    last_scene_angles_.clear();\n',
     '    last_white_y_ = -1.0;\n'
     '    last_scene_angles_.clear();\n'
     '    slow_logged_ = false;\n'),
    ('      if (side_turn_when_ == "white" && check_white_trigger(white))\n'
     '      {\n'
     '        return;                  // 회전 시작 (또는 정지)\n'
     '      }\n',
     '      if (side_turn_when_ == "white" && check_white_trigger(white))\n'
     '      {\n'
     '        return;                  // 회전 시작 (또는 정지)\n'
     '      }\n'
     '      if (side_turn_when_ == "white")\n'
     '      {\n'
     '        slow_by_row(last_white_y_, stem_slow_start_y_, static_cast<double>(white_trigger_y_), "가로 흰 선");\n'
     '      }\n'),
    ('      if (leave_turn_ && state_time_ >= leave_min_time_ && leave_gap_reached(yellow))\n',
     '      if (leave_turn_)\n'
     '      {\n'
     '        slow_by_row(yellow_end_y(yellow), leave_slow_start_y_,\n'
     '          static_cast<double>(watch_bottom_ - watch_rows_), "노란선 끝");\n'
     '      }\n'
     '      // 빨리 가면 leave_min_time 안에 끊김을 지나칠 수 있어서, 같은 거리 (leave_min_time x drive_speed) 를 가도 본다\n'
     '      const bool leave_min_ok = state_time_ >= leave_min_time_ || exit_travel_ >= leave_min_time_ * drive_speed_;\n'
     '      if (leave_turn_ && leave_min_ok && leave_gap_reached(yellow))\n'),
    ('  bool i_am_driving() const\n',
     '''  // 선이 화면 아래로 (가까이) 올수록 일반 주행 속도 (run_speed) -> drive_speed 로 줄인다. 다시 빨라지지는 않는다
  //   y: 선의 줄 번호 (-1 = 안 보임 -> 그대로). start_y 에서 줄이기 시작, end_y 에서 drive_speed
  void slow_by_row(double y, double start_y, double end_y, const char * what)
  {
    if (!slow_near_ || y < 0.0 || run_speed_ <= drive_speed_ || end_y <= start_y)
    {
      return;
    }
    const double r = std::clamp((y - start_y) / (end_y - start_y), 0.0, 1.0);
    const double target = run_speed_ + (drive_speed_ - run_speed_) * r;
    if (target < speed_)
    {
      speed_ = target;
      if (!slow_logged_)
      {
        slow_logged_ = true;
        RCLCPP_INFO(get_logger(), "%s 가까워짐 (y=%.0f) -> 천천히 (%.2f -> %.2f m/s)", what, y, run_speed_, drive_speed_);
      }
    }
  }

  // 왼쪽 칸 [0, watch_x_max) 에서 노란선이 아래 (watch_bottom) 부터 위로 이어지다 끝나는 줄 번호. 없으면 -1
  //   (길을 내려갈 때 길이 도로와 만나는 곳 = 노란선 끝이 화면 위에서 내려온다. park5 녹화로 확인: 0.1초에 약 25줄씩)
  double yellow_end_y(const cv::Mat & yellow) const
  {
    const int x1 = std::min(static_cast<int>(watch_x_max_), yellow.cols);
    double end = -1.0;
    int gap = 0;
    for (int row = std::min(static_cast<int>(watch_bottom_), yellow.rows - 1); row >= 0; row--)
    {
      if (cv::countNonZero(yellow.row(row).colRange(0, x1)) >= 3)
      {
        end = row;
        gap = 0;
      }
      else if (++gap > 15)
      {
        break;
      }
    }
    return end;
  }

  bool i_am_driving() const
'''),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:60]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)

YA = '    match_turn_speed:' if '    match_turn_speed:' in ys else '    match_drive_speed:'
if 'slow_near' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + ('    slow_near: true                # 가로 흰 선 / 나갈 때 노란선 끝이 가까워질수록 drive_speed 까지 천천히 (false = 안 함)\n'
                   '    stem_slow_start_y: 100.0        # 통로: 가로 흰 선이 화면 이 줄에 오면 줄이기 시작 (작을수록 일찍). white_trigger_y 에서 drive_speed\n'
                   '    leave_slow_start_y: 100.0       # 나갈 때: 왼쪽 노란선 끝이 이 줄에 오면 줄이기 시작. 끊김 보는 칸 위에서 drive_speed\n') + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('가까워지면 천천히 OK')

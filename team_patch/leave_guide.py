# parking_node: 도로로 나가는 왼쪽 곡선도 "보면서" (도로 바깥 흰 선까지 거리 / 각도로 반지름을 매번 다시 계산)
#   parking 폴더 (colcon_ws/src/parking) 에서 실행. exit_guide.py 가 먼저 들어가 있어야 한다
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns, ys = open(n).read(), open(y).read()
if 'leave_guide' in ns:
    print('이미 들어가 있어요. 바꾸지 않았어요'); sys.exit()
A = '''    else if (state_ == State::ENTRY_ARC)
    {
      cv::Mat lines_mask;
      cv::bitwise_or(yellow, white_mask(lab), lines_mask);
      entry_arc_step(yellow, lines_mask);
    }
'''
B = '  void finish_leave()\n'
C = '    exit_guide_max_radius_m_ = declare_parameter("exit_guide_max_radius_m", 2.0);\n'
D = '  bool exit_guide_active_ = false; // 이번 EXIT_ARC 에서 안내선을 보고 도는 중\n'
E = '''    entry_cam_lost_ = 0;
    entry_cam_failed_ = false;
    RCLCPP_INFO(get_logger(), "곡선으로 왼쪽 (반지름'''
F = '    leave_forward_m: '
for txt, key in ((ns, A), (ns, B), (ns, C), (ns, D), (ns, E), (ys, F)):
    if txt.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()

ns = ns.replace(A, '''    else if (state_ == State::ENTRY_ARC)
    {
      const cv::Mat white = white_mask(lab);
      // 도로로 나가는 곡선이면 도로 바깥 흰 선을 보고 돈다 (못 찾으면 원래 방식)
      if (!(leaving_ && leave_guide_ && leave_arc_by_guide(white)))
      {
        cv::Mat lines_mask;
        cv::bitwise_or(yellow, white, lines_mask);
        entry_arc_step(yellow, lines_mask);
      }
    }
''')

ns = ns.replace(B, '''  // 도로로 나가는 왼쪽 곡선 (ENTRY_ARC + leaving_, 영상마다): 도로 바깥 흰 선 (왼쪽으로 돌면 오른쪽에 보임) 까지
  //   거리 D 와 기운 각 phi 로 반지름 R = (D - leave_guide_offset_m) / (1 - cos phi) 를 매번 다시 계산한다
  //   (칸에서 나올 때 exit_arc_by_guide 와 같은 방법). 나란해지면 (또는 지나치면) 일반 주행으로 넘긴다
  bool leave_arc_by_guide(const cv::Mat & white)
  {
    const std::optional<cv::Vec2d> g = find_exit_guide(white, 1.0);
    if (!g)
    {
      if (leave_guide_active_)
      {
        drive(entry_arc_speed_, entry_w_);   // 잠깐 놓치면 마지막 곡선 그대로
        return true;
      }
      return false;
    }
    const double dist = (*g)[0];
    const double lean = (*g)[1];
    const double phi = std::abs(lean);
    // 왼쪽으로 돌고 있으니 선은 왼쪽으로 기울어 있다가 (-) 세로가 된다. + 면 지나침
    if (phi <= leave_guide_align_deg_ * M_PI / 180.0 || lean > 0.0)
    {
      RCLCPP_INFO(get_logger(), "도로 흰 선과 나란해짐 (선까지 %.0fcm, 기울기 %.1f도)", dist * 100.0, lean * 180.0 / M_PI);
      stop_wheels();
      finish_leave();
      return true;
    }
    const double need = dist - leave_guide_offset_m_;
    double radius = leave_guide_min_radius_m_;
    if (need > 0.0)
    {
      radius = std::clamp(need / (1.0 - std::cos(phi)), leave_guide_min_radius_m_, exit_guide_max_radius_m_);
    }
    if (!leave_guide_active_)
    {
      RCLCPP_INFO(get_logger(), "도로 흰 선 보임 (선까지 %.0fcm, 기울기 %.0f도) -> 보면서 도로로 (반지름 %.0fcm)",
        dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    }
    leave_guide_active_ = true;
    entry_w_ = entry_arc_speed_ / radius;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "보면서 도로로: 선까지 %.0fcm, 기울기 %.0f도 -> 반지름 %.0fcm",
      dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    drive(entry_arc_speed_, entry_w_);
    return true;
  }

''' + B)

ns = ns.replace(C, C + '''    // 9-3) 도로로 나가는 왼쪽 곡선도 보면서: 도로 바깥 흰 선에서 leave_guide_offset_m 떨어져 끝나게
    leave_guide_ = declare_parameter("leave_guide", true);
    leave_guide_offset_m_ = declare_parameter("leave_guide_offset_m", 0.15);   // 끝났을 때 흰 선에서 로봇 가운데까지 (= 차선 폭 / 2)
    leave_guide_align_deg_ = declare_parameter("leave_guide_align_deg", 4.0);
    leave_guide_min_radius_m_ = declare_parameter("leave_guide_min_radius_m", 0.10);
''')

ns = ns.replace(D, D + '''  bool leave_guide_;
  double leave_guide_offset_m_, leave_guide_align_deg_, leave_guide_min_radius_m_;
  bool leave_guide_active_ = false; // 이번 도로로 나가는 곡선에서 흰 선을 보고 도는 중
''')

ns = ns.replace(E, '''    entry_cam_lost_ = 0;
    entry_cam_failed_ = false;
    leave_guide_active_ = false;
    RCLCPP_INFO(get_logger(), "곡선으로 왼쪽 (반지름''')

lines = ys.split('\n')
for k, l in enumerate(lines):
    if l.startswith(F):
        lines[k + 1:k + 1] = [
            '    leave_guide: true               # 도로로 나가는 곡선을 도로 바깥 흰 선을 보며 (false = 예전 고정 곡선)',
            '    leave_guide_offset_m: 0.15      # [m] 끝났을 때 흰 선에서 로봇 가운데까지. 흰 선을 밟으면 키우기, 노란선 쪽으로 붙으면 줄이기',
            '    leave_guide_align_deg: 4.0      # [도] 흰 선 기울기가 이 이하면 일반 주행으로',
        ]
        break
open(n, 'w').write(ns); open(y, 'w').write('\n'.join(lines))
print('도로로 보면서 나가기 추가 OK')

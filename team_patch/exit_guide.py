# parking_node: 칸에서 나갈 때 "보면서 나가기" (안내 노란선과의 거리 / 각도로 곡선 반지름을 매번 다시 계산)
#   lane_follower 폴더에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns, ys = open(n).read(), open(y).read()
if 'exit_guide' in ns:
    print('이미 들어가 있어요. 바꾸지 않았어요'); sys.exit()
A = '    exit_arc_min_radius_m_ = declare_parameter("exit_arc_min_radius_m", 0.12);\n'
B = '''  void exit_arc_by_camera(const cv::Mat & yellow, const cv::Mat & lines_mask)
  {
'''
C = '''    if (state_time_ >= exit_arc_limit_)
    {
      give_up("곡선으로 도는 데 너무 오래 걸림");
      return;
    }
    if (exit_use_cam_)
'''
D = '''    exit_yellow_seen_ = false;
    RCLCPP_INFO(get_logger(), "%s으로 곡선 %.0f도 (반지름 %.0fcm, 속도 %.2fm/s, %s)",'''
E = '  double exit_arc_min_radius_m_;\n'
F = '    exit_arc_radius_m: '
for txt, key in ((ns, A), (ns, B), (ns, C), (ns, D), (ns, E), (ys, F)):
    if txt.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()

ns = ns.replace(A, A + '''    // 9-2) 보면서 나가기: 곡선 중 안내 노란선 (칸 모서리에 붙은 길의 노란선) 이 보이면
    //   그 선까지 거리 D 와 기운 각 phi 로 "이 반지름으로 돌면 선과 나란해질 때 선에서 exit_guide_offset_m 떨어진다"
    //   R = (D - offset) / (1 - cos phi) 를 매 영상 다시 계산해서 돈다. phi 가 exit_guide_align_deg 이하면 길 내려가기
    exit_guide_ = declare_parameter("exit_guide", true);
    exit_guide_offset_m_ = declare_parameter("exit_guide_offset_m", 0.15);     // 끝났을 때 선에서 로봇 가운데까지 (= 길 폭 / 2)
    exit_guide_align_deg_ = declare_parameter("exit_guide_align_deg", 4.0);
    exit_guide_max_radius_m_ = declare_parameter("exit_guide_max_radius_m", 2.0);
''')

ns = ns.replace(B, B + '''    if (exit_guide_ && exit_arc_by_guide(yellow))
    {
      return;                    // 안내선을 보고 돌았음
    }
''')

ns = ns.replace(C, '''    if (state_time_ >= exit_arc_limit_)
    {
      give_up("곡선으로 도는 데 너무 오래 걸림");
      return;
    }
    if (exit_guide_active_)
    {
      return;                    // 안내선을 본 뒤로는 영상이 올 때마다 exit_arc_by_guide() 가 바퀴 명령을 보낸다
    }
    if (exit_use_cam_)
''')

ns = ns.replace(D, '''    exit_yellow_seen_ = false;
    exit_guide_active_ = false;
    RCLCPP_INFO(get_logger(), "%s으로 곡선 %.0f도 (반지름 %.0fcm, 속도 %.2fm/s, %s)",''')

# 함수들: exit_arc_by_camera 바로 위에 넣는다
fn = '''  // 보면서 나가기 (EXIT_ARC, 영상마다). 안내선을 못 찾으면 false (원래 방식으로)
  //   안내선 = 나가는 쪽 반대편에 보이는 노란선 (왼쪽으로 나가면 오른쪽, 오른쪽으로 나가면 왼쪽)
  bool exit_arc_by_guide(const cv::Mat & yellow)
  {
    const std::optional<cv::Vec2d> g = find_exit_guide(yellow, side_direction_);
    if (!g)
    {
      if (exit_guide_active_)
      {
        drive(exit_arc_speed_, exit_w_);   // 잠깐 놓치면 마지막 곡선 그대로
        return true;
      }
      return false;
    }
    const double dist = (*g)[0];
    const double lean = (*g)[1];
    const double phi = std::abs(lean);
    // 나란해졌거나 지나쳤으면 (나가는 방향 x 기울기 > 0) 길 내려가기로
    if (phi <= exit_guide_align_deg_ * M_PI / 180.0 || side_direction_ * lean > 0.0)
    {
      RCLCPP_INFO(get_logger(), "안내 노란선과 나란해짐 (선까지 %.0fcm, 기울기 %.1f도) -> 길 내려가기",
        dist * 100.0, lean * 180.0 / M_PI);
      start_exit_stem();
      return true;
    }
    const double need = dist - exit_guide_offset_m_;
    double radius = exit_arc_min_radius_m_;
    if (need > 0.0)
    {
      radius = std::clamp(need / (1.0 - std::cos(phi)), exit_arc_min_radius_m_, exit_guide_max_radius_m_);
    }
    if (!exit_guide_active_)
    {
      RCLCPP_INFO(get_logger(), "안내 노란선 보임 (선까지 %.0fcm, 기울기 %.0f도) -> 보면서 나가기 (반지름 %.0fcm)",
        dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    }
    exit_guide_active_ = true;
    exit_w_ = side_direction_ * exit_arc_speed_ / radius;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "보면서 나가기: 선까지 %.0fcm, 기울기 %.0f도 -> 반지름 %.0fcm",
      dist * 100.0, lean * 180.0 / M_PI, radius * 100.0);
    drive(exit_arc_speed_, exit_w_);
    return true;
  }

  // 안내 노란선 찾기 (BEV, 1px = 1mm). 돌려주는 값: (바퀴 축에서 선까지 수직 거리 [m], 선이 기운 각 [rad, + = 앞쪽이 오른쪽])
  //   노란 덩어리마다 아래쪽 150줄로 직선 x = a + b*y 를 맞추고, 나가는 반대쪽에 있는 것 중 가장 가까운 것
  std::optional<cv::Vec2d> find_exit_guide(const cv::Mat & yellow, double direction) const
  {
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(yellow, labels, stats, centroids, 8, CV_32S);
    const double axle_y = yellow.rows + bev_near_m_ * 1000.0;   // 바퀴 축의 화면 y (화면 아래 밖)
    const double cx = bev_center_x_ * yellow.cols;
    std::optional<cv::Vec2d> best;
    for (int i = 1; i < n; ++i)
    {
      if (stats.at<int>(i, cv::CC_STAT_AREA) < 300)
      {
        continue;
      }
      std::vector<cv::Point> pts;
      cv::findNonZero(labels == i, pts);
      int y_max = 0;
      for (const auto & p : pts)
      {
        y_max = std::max(y_max, p.y);
      }
      // 최소제곱 직선 x = a + b*y (아래쪽 150줄만: 위쪽은 길이 휘는 곳일 수 있음)
      double sn = 0, sy = 0, sx = 0, syy = 0, sxy = 0;
      for (const auto & p : pts)
      {
        if (p.y < y_max - 150)
        {
          continue;
        }
        sn += 1; sy += p.y; sx += p.x; syy += double(p.y) * p.y; sxy += double(p.x) * p.y;
      }
      const double den = sn * syy - sy * sy;
      if (sn < 50 || std::abs(den) < 1e-6)
      {
        continue;
      }
      const double b = (sn * sxy - sy * sx) / den;
      const double a = (sx - b * sy) / sn;
      const double lean = std::atan(-b);                       // 앞으로 갈수록 x 가 커지면 +
      if (std::abs(lean) > 70.0 * M_PI / 180.0)
      {
        continue;
      }
      const double x_axle = (a + b * axle_y - cx) * 0.001;     // 바퀴 축 줄에서 선의 가로 위치 [m] (+ 오른쪽)
      if (x_axle * direction < 0.0)                            // 왼쪽으로 나가면 (+1) 선은 오른쪽 (+)
      {
        continue;
      }
      const double dist = std::abs(x_axle) * std::cos(lean);
      if (!best || dist < (*best)[0])
      {
        best = cv::Vec2d(dist, lean);
      }
    }
    return best;
  }

'''
ns = ns.replace(B, fn + B)

ns = ns.replace(E, E + '''  bool exit_guide_;
  double exit_guide_offset_m_, exit_guide_align_deg_, exit_guide_max_radius_m_;
  bool exit_guide_active_ = false; // 이번 EXIT_ARC 에서 안내선을 보고 도는 중
''')

lines = ys.split('\n')
for k, l in enumerate(lines):
    if l.startswith(F):
        add = [
            '    # 9-2) 보면서 나가기: 곡선 중 안내 노란선이 보이면 선까지 거리 / 각도로 반지름을 매번 다시 계산',
            '    exit_guide: true                # false = 예전 고정 곡선',
            '    exit_guide_offset_m: 0.15       # [m] 끝났을 때 노란선에서 로봇 가운데까지. 선을 밟으면 키우기, 반대쪽으로 붙으면 줄이기',
            '    exit_guide_align_deg: 4.0       # [도] 노란선 기울기가 이 이하면 길 내려가기',
        ]
        lines[k + 1:k + 1] = add
        break
open(n, 'w').write(ns); open(y, 'w').write('\n'.join(lines))
print('보면서 나가기 추가 OK')

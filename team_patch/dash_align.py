# parking_node: 흰 점선으로 후진 방향 맞추기 + 나가는 곡선 반지름. lane_follower 폴더에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns, ys = open(n).read(), open(y).read()
if 'dash_align' in ns:
    print('이미 들어가 있어요. 바꾸지 않았어요'); sys.exit()
A = '    exit_arc_min_radius_m_ = declare_parameter("exit_arc_min_radius_m", 0.12);\n'
B = '''    else if (state_ == State::BACKUP)
    {
      // 후진은 시간으로 한다. 곡선으로 돌기 시작할 때의 기준(직선 방향 분포)만 계속 저장해 둔다
      cv::Mat lines_mask;
      cv::bitwise_or(yellow, white_mask(lab), lines_mask);
      last_scene_angles_ = scene_line_angles(lines_mask);
    }
'''
C = '''    state_ = State::BACKUP;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "%.1f초 후진", backup_duration_);
    drive(-backup_speed_, 0.0);
  }
'''
D = '''      finish_backup();
      return;
    }
    drive(-backup_speed_, 0.0);
  }

  void finish_backup()
  {
    stop_wheels();
'''
E = '''  double exit_arc_radius()
  {
    double radius = exit_arc_radius_m_;
'''
F = '  double exit_arc_min_radius_m_;\n'
G = '  // 작은 점(잡음) 지우기\n'
H = '    exit_arc_radius_m: '
for txt, key in ((ns, A), (ns, B), (ns, C), (ns, D), (ns, E), (ns, F), (ns, G), (ys, H)):
    if txt.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()

ns = ns.replace(A, A + '''
    // 9-1) 흰 점선 (맞은편 칸 입구) 으로 후진 방향 맞추기 + 나가는 곡선 반지름
    //   후진하는 동안 점선의 기울기 x dash_gain 만큼 돌면서 후진 -> 점선이 화면에서 가로가 되게 (= 칸과 똑바로)
    //   나갈 때: 점선까지 거리 - 통로 폭 / 2 = 통로 가운데까지 거리 -> 이 반지름으로 돌면 통로 가운데로 나감
    dash_align_ = declare_parameter("dash_align", true);
    dash_gain_ = declare_parameter("dash_gain", 0.6);                // 기울기 [rad] x 이 값 = 회전 명령 [rad/s]
    dash_max_w_ = declare_parameter("dash_max_w", 0.4);              // [rad/s] 맞추는 회전 최대
    dash_max_deg_ = declare_parameter("dash_max_deg", 30.0);         // 이보다 기운 조각은 점선으로 안 봄
    dash_min_len_px_ = declare_parameter("dash_min_len_px", 35.0);   // 점선 한 조각 길이 범위 [px] (실제 약 110)
    dash_max_len_px_ = declare_parameter("dash_max_len_px", 130.0);
    dash_max_width_px_ = declare_parameter("dash_max_width_px", 30.0);   // 조각 두께 최대 [px] (실제 약 20)
    bev_near_m_ = declare_parameter("bev_near_m", 0.179);           // BEV 맨 아래 = 바퀴 축 앞 [m]
    aisle_width_m_ = declare_parameter("aisle_width_m", 0.31);       // 통로 양쪽 점선 사이 [m] (녹화로 잰 값)
    exit_dash_gain_ = declare_parameter("exit_dash_gain", 1.25);     // 실제 반지름 -> 명령 반지름 (0 = 점선 안 씀)
''')

ns = ns.replace(B, '''    else if (state_ == State::BACKUP)
    {
      // 후진은 시간으로 한다. 곡선으로 돌기 시작할 때의 기준(직선 방향 분포)도 계속 저장해 둔다
      const cv::Mat white = white_mask(lab);
      cv::Mat lines_mask;
      cv::bitwise_or(yellow, white, lines_mask);
      last_scene_angles_ = scene_line_angles(lines_mask);
      update_dash(white);        // 흰 점선 기울기 / 거리 (step_backup 이 보고 방향을 맞춤)
    }
''')

ns = ns.replace(C, '''    state_ = State::BACKUP;
    state_time_ = 0.0;
    dash_time_ = -1.0;           // 이번 후진에서 본 점선만 쓴다
    RCLCPP_INFO(get_logger(), "%.1f초 후진%s", backup_duration_, dash_align_ ? " (흰 점선이 보이면 가로가 되게 돌면서)" : "");
    drive(-backup_speed_, 0.0);
  }
''')

ns = ns.replace(D, '''      finish_backup();
      return;
    }
    drive(-backup_speed_, dash_turn());
  }

  // 후진 중 회전 명령: 점선이 최근 (0.3초 안) 보였으면 기울기에 비례해서, 아니면 0 (똑바로)
  //   점선이 화면에서 a 만큼 기울었으면 (y 아래 방향 기준, + = 오른쪽이 아래) 로봇이 -a 만큼 돌아야 가로가 된다
  double dash_turn()
  {
    if (!dash_align_ || !dash_fresh(0.3))
    {
      return 0.0;
    }
    const double w = std::clamp(-dash_gain_ * dash_angle_, -dash_max_w_, dash_max_w_);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "흰 점선 기울기 %.1f도 -> 돌면서 후진 (%.2f rad/s)",
      dash_angle_ * 180.0 / M_PI, w);
    return w;
  }

  bool dash_fresh(double within_s)
  {
    return dash_time_ >= 0.0 && now().seconds() - dash_time_ <= within_s;
  }

  // 흰 마스크 (BEV) 에서 점선 한 조각을 찾아 기울기 / 거리를 기억한다
  void update_dash(const cv::Mat & white)
  {
    const std::optional<cv::Vec2d> dash = find_dash(white);
    if (!dash)
    {
      return;
    }
    dash_angle_ = (*dash)[0];
    dash_dist_ = bev_near_m_ + (white.rows - (*dash)[1]) * 0.001;   // 1px = 1mm
    dash_time_ = now().seconds();
  }

  // 점선 조각 찾기: 길이 dash_min_len_px ~ dash_max_len_px, 두께 dash_max_width_px 이하, 가로에서 dash_max_deg 이내인
  //   흰 덩어리 중 로봇에 가장 가까운 것 (화면 아래). 돌려주는 값: (기울기 [rad, + = 오른쪽이 아래], 가운데 y [px])
  //   (칸 테두리 실선은 길어서 빠지고, 실선 끝에 붙은 짧은 선도 실선과 한 덩어리라 빠진다)
  std::optional<cv::Vec2d> find_dash(const cv::Mat & white) const
  {
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(white, labels, stats, centroids, 8, CV_32S);
    std::optional<cv::Vec2d> best;
    for (int i = 1; i < n; ++i)
    {
      if (stats.at<int>(i, cv::CC_STAT_AREA) < 150)
      {
        continue;
      }
      std::vector<cv::Point> pts;
      cv::findNonZero(labels == i, pts);
      const cv::RotatedRect box = cv::minAreaRect(pts);
      const double len = std::max(box.size.width, box.size.height);
      const double wid = std::min(box.size.width, box.size.height);
      if (len < dash_min_len_px_ || len > dash_max_len_px_ || wid > dash_max_width_px_ || len < 2.5 * wid)
      {
        continue;
      }
      cv::Vec4f fit;
      cv::fitLine(pts, fit, cv::DIST_L2, 0, 0.01, 0.01);
      double ang = std::atan2(fit[1], fit[0]);
      if (ang > M_PI / 2.0)
      {
        ang -= M_PI;
      }
      if (ang <= -M_PI / 2.0)
      {
        ang += M_PI;
      }
      if (std::abs(ang) > dash_max_deg_ * M_PI / 180.0)
      {
        continue;
      }
      const double cy = box.center.y;
      if (!best || cy > (*best)[1])
      {
        best = cv::Vec2d(ang, cy);
      }
    }
    return best;
  }

  void finish_backup()
  {
    stop_wheels();
    if (dash_align_ && dash_fresh(0.5))
    {
      RCLCPP_INFO(get_logger(), "후진 끝: 흰 점선 기울기 %.1f도, 점선까지 %.0fcm", dash_angle_ * 180.0 / M_PI,
        dash_dist_ * 100.0);
    }
''')

ns = ns.replace(E, '''  double exit_arc_radius()
  {
    // 흰 점선을 방금 봤으면: 통로 가운데까지 거리 (점선까지 - 통로 폭 / 2) 를 실제 반지름으로
    if (dash_align_ && exit_dash_gain_ > 0.0 && dash_fresh(1.0))
    {
      const double real = dash_dist_ - aisle_width_m_ / 2.0;
      const double cmd = std::clamp(real * exit_dash_gain_, exit_arc_min_radius_m_, 0.6);
      RCLCPP_INFO(get_logger(), "흰 점선까지 %.0fcm -> 통로 가운데까지 %.0fcm -> 명령 반지름 %.0fcm",
        dash_dist_ * 100.0, real * 100.0, cmd * 100.0);
      return cmd;
    }
    double radius = exit_arc_radius_m_;
''')

ns = ns.replace(F, F + '''  bool dash_align_;
  double dash_gain_, dash_max_w_, dash_max_deg_, dash_min_len_px_, dash_max_len_px_, dash_max_width_px_;
  double bev_near_m_, aisle_width_m_, exit_dash_gain_;
  double dash_angle_ = 0.0;        // 마지막으로 본 흰 점선 기울기 [rad] (+ = 오른쪽이 아래)
  double dash_dist_ = 0.0;         // 바퀴 축에서 그 점선까지 [m]
  double dash_time_ = -1.0;        // 그 점선을 본 시각 [s] (-1 = 못 봄)
''')

lines = ys.split('\\n') if False else ys.split('\n')
for i, l in enumerate(lines):
    if l.startswith(H):
        add = [
            '',
            '    # 9-1) 흰 점선 (맞은편 칸 입구) 으로 후진 방향 맞추기 + 나가는 곡선 반지름',
            '    dash_align: true                # false = 예전처럼 똑바로 후진, 나가는 반지름은 exit_arc_radius_m',
            '    dash_gain: 0.6                  # 점선 기울기 x 이 값 = 후진 중 회전. 덜 맞으면 키우기, 흔들리면 줄이기',
            '    dash_max_w: 0.4                 # [rad/s] 후진 중 맞추는 회전 최대',
            '    aisle_width_m: 0.31             # [m] 통로 양쪽 점선 사이 (녹화로 잰 값. 줄자로 재서 고쳐도 됨)',
            '    exit_dash_gain: 1.25            # 나갈 때 실제 반지름 x 이 값 = 명령 반지름. 크게 돌면 줄이기, 안쪽으로 붙으면 키우기 (0 = 끔)',
        ]
        lines[i + 1:i + 1] = add
        break
open(n, 'w').write(ns); open(y, 'w').write('\n'.join(lines))
print('흰 점선 기능 추가 OK')

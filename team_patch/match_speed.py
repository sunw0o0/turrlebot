# parking_node: 주차를 일반 주행 (grid_drive) 과 같은 속도로 (칸으로 들어가는 동안만 원래 속도)
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns, ys = open(n).read(), open(y).read()
if 'match_drive_speed' in ns:
    print('이미 들어가 있어요. 바꾸지 않았어요'); sys.exit()
A = '    exit_arc_speed_ = declare_parameter("exit_arc_speed", 0.08);\n'
B = '    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);\n'
C = '''    speed_ = drive_speed_;
    RCLCPP_INFO(get_logger(), "주차 직진 속도 %.2f m/s (drive_speed)", speed_);
'''
D = '''  void start_side_forward(double direction)
  {
    side_direction_ = direction;
'''
E = '    state_ = State::EXIT_STEM;\n'
F = '  double speed_ = 0.10;'
G = '    drive_speed: '
for txt, key in ((ns, A), (ns, B), (ns, C), (ns, D), (ns, E), (ns, F), (ys, G)):
    if txt.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()

ns = ns.replace(A, A + '''    // 일반 주행과 같은 속도로: 주차를 시작할 때 grid_drive 가 보내던 속도를 직진 / 입구 곡선 / 나오는 곡선 / 길 내려가기 /
    //   도로로 나가는 곡선에 쓴다. 칸으로 들어가는 동안 (흰 선 뒤 전진, 뒤로 곡선, 후진) 은 원래 값 그대로
    match_drive_speed_ = declare_parameter("match_drive_speed", true);
    match_speed_min_ = declare_parameter("match_speed_min", 0.05);   // 이 범위 밖이면 잘라서 씀 [m/s, 명령 단위]
    match_speed_max_ = declare_parameter("match_speed_max", 0.25);
    entry_arc_speed_base_ = entry_arc_speed_;
    exit_arc_speed_base_ = exit_arc_speed_;
''')

ns = ns.replace(B, B + '''    // 일반 주행 (grid_drive) 이 보내는 속도를 기억해 둔다 (주차를 같은 속도로 이어가려고)
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg)
      {
        if (!i_am_driving() && msg->linear.x > 0.0)
        {
          seen_drive_v_ = msg->linear.x;
          seen_drive_t_ = now().seconds();
        }
      });
''')

ns = ns.replace(C, '''    speed_ = drive_speed_;
    entry_arc_speed_ = entry_arc_speed_base_;
    exit_arc_speed_ = exit_arc_speed_base_;
    const bool fresh = seen_drive_t_ >= 0.0 && now().seconds() - seen_drive_t_ < 1.0;
    if (match_drive_speed_ && fresh)
    {
      // 일반 주행 속도 그대로 (직진 / 입구 곡선 / 나오는 곡선 모두). 곡선 반지름은 속도와 상관없이 그대로다
      speed_ = std::clamp(seen_drive_v_, match_speed_min_, match_speed_max_);
      entry_arc_speed_ = speed_;
      exit_arc_speed_ = speed_;
      RCLCPP_INFO(get_logger(), "주차 속도 = 일반 주행 속도 %.2f m/s (칸으로 들어가는 동안만 원래 속도)", speed_);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "주차 직진 속도 %.2f m/s (drive_speed)", speed_);
    }
    run_speed_ = speed_;
''')

ns = ns.replace(D, D + '''    speed_ = drive_speed_;       // 칸으로 들어가는 동안은 원래 속도 (흰 선 뒤 전진 거리가 맞춰 둔 그대로 되게)
''')

ns = ns.replace(E, E + '''    speed_ = run_speed_;         // 칸에서 나왔으니 다시 일반 주행 속도
''')

i = ns.index(F)
j = ns.index('\n', i) + 1
ns = ns[:j] + '''  bool match_drive_speed_;
  double match_speed_min_, match_speed_max_;
  double entry_arc_speed_base_ = 0.08, exit_arc_speed_base_ = 0.08;   // yaml 값 (일반 주행 속도로 바꾸기 전)
  double run_speed_ = 0.10;        // 이번 주차에서 쓰는 "일반 주행 속도"
  double seen_drive_v_ = 0.0;      // 마지막으로 본 일반 주행 직진 속도 [m/s, 명령 단위]
  double seen_drive_t_ = -1.0;     // 그 시각 [s]
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
''' + ns[j:]

lines = ys.split('\n')
for k, l in enumerate(lines):
    if l.startswith(G):
        lines.insert(k + 1, '    match_drive_speed: true         # 주차를 일반 주행 (grid_drive) 과 같은 속도로 (칸으로 들어가는 동안만 위 값들). false = drive_speed 등 그대로')
        break
open(n, 'w').write(ns); open(y, 'w').write('\n'.join(lines))
print('일반 주행 속도로 주차 추가 OK')

# (Jetson 버전 parking_node, leave_fast.py 다음에) 주차 중 바퀴 명령을 부드럽게:
#   (1) 속도는 smooth_accel 만큼씩만 바뀐다 (툭 줄거나 늘지 않음). 앞뒤가 바뀔 때도 서서히 멈췄다가 반대로
#   (2) 앞으로 가며 곡선으로 들어갈 때 smooth_curve_m 거리 동안 서서히 꺾는다 (반지름 20cm 곡선 기준). 대신 그 절반만큼
#       일찍 꺾기 시작해서 지나가는 길은 예전과 같게 (입구 / 도로로 나가기)
#   (3) 곡선 -> 직진으로 바뀔 때 잠깐 0 으로 멈추던 것 없앰 (입구 곡선 끝, 도로로 나간 뒤 일반 주행에 넘길 때)
#   (4) 뒤로 곡선 전에 서서히 멈추는 동안 더 가는 만큼 전진을 일찍 끝냄 (칸 안 위치가 예전과 같게)
#   정면 막힘 / 영상 끊김 / 그만둠 / 명령이 0.3초 안 옴 은 예전처럼 바로 정지.  smooth: false = 예전 그대로
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'smooth_step' in ns:
    print('이미 들어가 있어요')
    sys.exit()
if 'leave_turn_match_' not in ns:
    print('leave_fast.py 를 먼저 넣어 주세요')
    sys.exit()

R = [
    # 파라미터
    ('    image_timeout_ = param("image_timeout", 0.5);\n',
     '    image_timeout_ = param("image_timeout", 0.5);\n'
     '    // 바퀴 명령 부드럽게 (속도는 smooth_accel 씩, 곡선은 smooth_curve_m 동안 서서히)\n'
     '    smooth_ = param("smooth", true);\n'
     '    smooth_accel_ = param("smooth_accel", 0.4);       // [m/s^2 명령] 속도 바뀌는 빠르기 (0.2 -> 0 이 0.5초)\n'
     '    smooth_curve_m_ = param("smooth_curve_m", 0.02);  // [m 명령] 반지름 20cm 곡선으로 다 꺾을 때까지 가는 거리\n'),
    # 50Hz 타이머
    ('    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]\n'
     '      {\n'
     '        watchdog();\n'
     '      });\n',
     '    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]\n'
     '      {\n'
     '        watchdog();\n'
     '      });\n'
     '    // 0.02초마다: 부드럽게 바꾼 바퀴 명령 보내기 (smooth)\n'
     '    smooth_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]\n'
     '      {\n'
     '        smooth_step();\n'
     '      });\n'),
    # 주차 시작: 일반 주행 속도에서 이어받기
    ('    run_speed_ = speed_;\n',
     '    run_speed_ = speed_;\n'
     '    v_out_ = fresh ? std::clamp(seen_drive_v_, 0.0, 0.5) : 0.0;   // 일반 주행이 가던 속도에서 이어서\n'
     '    w_out_ = 0.0;\n'
     '    k_out_ = 0.0;\n'
     '    tgt_v_ = v_out_;           // 첫 영상이 오기 전에 "명령 끊김" 으로 멈추지 않게 (그동안 지금 속도로 직진)\n'
     '    tgt_w_ = 0.0;\n'
     '    tgt_t_ = now().seconds();\n'),
    # 입구 곡선: 서서히 꺾는 만큼 일찍
    ('    const double extra_m = std::max(0.0, entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_ - latency_m);\n',
     '    const double extra_m = std::max(0.0, entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_ - latency_m -\n'
     '      curve_ramp_shift(entry_radius_cmd()));   // 서서히 꺾는 만큼 일찍\n'),
    # 도로로 나가는 곡선: 서서히 꺾는 만큼 일찍
    ('      std::max(0.0, speed_ - drive_speed_) * std::max(0.0, entry_latency_s_));',
     '      std::max(0.0, speed_ - drive_speed_) * std::max(0.0, entry_latency_s_) -\n'
     '      curve_ramp_shift(entry_radius_cmd()));'),
    # 입구 곡선 끝: 0 으로 안 멈추고 바로 직진으로
    ('        stop_wheels();\n'
     '        start_stem(*pair_lean);\n',
     '        end_curve();\n'
     '        start_stem(*pair_lean);\n'),
    ('          std::abs(entry_cam_sum_) * 180.0 / M_PI);\n'
     '        stop_wheels();\n'
     '        if (leaving_)\n',
     '          std::abs(entry_cam_sum_) * 180.0 / M_PI);\n'
     '        end_curve();\n'
     '        if (leaving_)\n'),
    ('      stop_wheels();\n'
     '      finish_leave();\n'
     '      return true;\n',
     '      end_curve();\n'
     '      finish_leave();\n'
     '      return true;\n'),
    # 칸에서 나오는 곡선 끝 -> 길 내려가기: 곡선은 바로 끝
    ('    state_ = State::EXIT_STEM;\n',
     '    state_ = State::EXIT_STEM;\n'
     '    if (smooth_)\n'
     '    {\n'
     '      end_curve();\n'
     '    }\n'),
    # 도로로 나와 일반 주행에 넘길 때: 0 대신 지금 속도로 직진 한 번
    ('  void finish_leave()\n'
     '  {\n',
     '  void finish_leave()\n'
     '  {\n'
     '    if (smooth_)\n'
     '    {\n'
     '      geometry_msgs::msg::Twist cmd;\n'
     '      cmd.linear.x = v_out_;\n'
     '      cmd_pub_->publish(cmd);    // 일반 주행이 이어받을 때까지 0 으로 서지 않게\n'
     '      v_out_ = w_out_ = k_out_ = 0.0;\n'
     '      tgt_v_ = tgt_w_ = 0.0;\n'
     '    }\n'),
    # 영상 끊김: 바로 정지
    ('      stop_wheels();\n'
     '      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상이 안 옴 -> 정지");\n',
     '      hard_stop();\n'
     '      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상이 안 옴 -> 정지");\n'),
    # 뒤로 곡선 전 전진: 서서히 멈추며 더 가는 만큼 일찍
    ('    side_forward_time_ = extra_time > 0.0 ? extra_time : extra_m / speed_;\n',
     '    side_forward_time_ = extra_time > 0.0 ? extra_time : extra_m / speed_;\n'
     '    if (smooth_ && smooth_accel_ > 0.0)\n'
     '    {\n'
     '      side_forward_time_ = std::max(0.0, side_forward_time_ - speed_ / (2.0 * smooth_accel_));   // 서서히 멈추며 더 가는 만큼\n'
     '    }\n'),
    # drive / stop_wheels
    ('    if (v > 0.0 && front_closer_than(front_stop_distance_))\n'
     '    {\n'
     '      stop_wheels();\n',
     '    if (v > 0.0 && front_closer_than(front_stop_distance_))\n'
     '    {\n'
     '      hard_stop();\n'),
    ('    geometry_msgs::msg::Twist cmd;\n'
     '    cmd.linear.x = v;\n'
     '    cmd.angular.z = w;\n'
     '    cmd_pub_->publish(cmd);\n'
     '  }\n',
     '    if (smooth_)\n'
     '    {\n'
     '      tgt_v_ = v;                // 실제로 보내는 건 smooth_step (0.02초마다)\n'
     '      tgt_w_ = w;\n'
     '      tgt_t_ = now().seconds();\n'
     '      return;\n'
     '    }\n'
     '    geometry_msgs::msg::Twist cmd;\n'
     '    cmd.linear.x = v;\n'
     '    cmd.angular.z = w;\n'
     '    cmd_pub_->publish(cmd);\n'
     '  }\n'),
    ('  void stop_wheels()\n'
     '  {\n'
     '    cmd_pub_->publish(geometry_msgs::msg::Twist());\n'
     '  }\n',
     '''  void stop_wheels()
  {
    if (smooth_ && i_am_driving())
    {
      tgt_v_ = 0.0;              // 서서히 멈춤 (지금 곡선 그대로)
      tgt_w_ = 0.0;
      tgt_t_ = now().seconds();
      return;
    }
    hard_stop();
  }

  // 바로 정지 (정면 막힘 / 영상 끊김 / 그만둠)
  void hard_stop()
  {
    v_out_ = w_out_ = k_out_ = 0.0;
    tgt_v_ = tgt_w_ = 0.0;
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  // 곡선 끝 -> 직진: 회전은 바로 멈추고 속도는 그대로 (smooth 가 아니면 예전처럼 정지)
  void end_curve()
  {
    if (!smooth_)
    {
      stop_wheels();
      return;
    }
    k_out_ = 0.0;
    w_out_ = 0.0;
    tgt_w_ = 0.0;
  }

  static double move_toward(double x, double target, double step)
  {
    return x + std::clamp(target - x, -step, step);
  }

  // 입구 / 도로로 나가는 곡선의 명령 반지름 [m]
  double entry_radius_cmd() const
  {
    return std::max(entry_arc_radius_m_ * entry_arc_radius_gain_, entry_arc_min_radius_m_);
  }

  // 곡선을 서서히 꺾으면 그 거리의 절반만큼 늦게 꺾은 것과 같다 -> 그만큼 일찍 시작 [m 명령]
  double curve_ramp_shift(double radius) const
  {
    if (!smooth_ || smooth_curve_m_ <= 0.0 || radius <= 0.0)
    {
      return 0.0;
    }
    return 0.5 * smooth_curve_m_ * (0.2 / radius);
  }

  // 0.02초마다: 목표 (tgt_v_, tgt_w_) 쪽으로 속도는 smooth_accel 씩, 곡률은 간 거리만큼씩 바꿔서 보낸다
  void smooth_step()
  {
    if (!smooth_)
    {
      return;
    }
    if (!i_am_driving() || now().seconds() - tgt_t_ > 0.3)
    {
      if (v_out_ != 0.0 || w_out_ != 0.0)
      {
        hard_stop();             // 주차 끝 / 그만둠 / 명령이 끊김 -> 바로 정지
      }
      return;
    }
    const double dt = 0.02;
    const double dv = std::max(0.0, smooth_accel_) * dt;
    if (std::abs(tgt_v_) < 1e-4 && std::abs(tgt_w_) > 1e-4)
    {
      v_out_ = move_toward(v_out_, 0.0, dv);            // 제자리 회전
      w_out_ = move_toward(w_out_, tgt_w_, 3.0 * dt);
      k_out_ = 0.0;
    }
    else
    {
      if (std::abs(tgt_v_) >= 1e-4 && v_out_ * tgt_v_ >= 0.0)
      {
        const double kt = tgt_w_ / tgt_v_;
        if (std::abs(v_out_) < 0.02 || smooth_curve_m_ <= 0.0 || tgt_v_ < 0.0)
        {
          k_out_ = kt;           // 거의 서 있을 때 (길이 안 바뀜) / 뒤로 갈 때 (점선 맞추기가 바로 반응하게) 는 바로
        }
        else
        {
          const double rate = (1.0 / 0.2) / smooth_curve_m_;   // [1/m^2] 반지름 20cm 를 smooth_curve_m 동안
          k_out_ = move_toward(k_out_, kt, rate * std::abs(v_out_) * dt);
        }
      }
      // 멈추는 중이거나 앞뒤가 바뀌는 중이면 지금 곡선 그대로 줄인다
      v_out_ = move_toward(v_out_, tgt_v_, dv);
      w_out_ = k_out_ * v_out_;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_out_;
    cmd.angular.z = w_out_;
    cmd_pub_->publish(cmd);
  }
'''),
    ('  double image_timeout_;\n',
     '  double image_timeout_;\n'
     '  bool smooth_;\n'
     '  double smooth_accel_, smooth_curve_m_;\n'
     '  double tgt_v_ = 0.0, tgt_w_ = 0.0, tgt_t_ = 0.0;   // 바퀴 명령 목표 (drive)\n'
     '  double v_out_ = 0.0, w_out_ = 0.0, k_out_ = 0.0;   // 실제로 보내는 값, 곡률 [1/m]\n'
     '  rclcpp::TimerBase::SharedPtr smooth_timer_;\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:70]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)

YA = '    leave_slow:'
if 'smooth:' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + ('    smooth: true                    # 바퀴 명령 부드럽게 (속도 / 꺾기 서서히, 곡선 끝에 0 으로 안 멈춤). false = 예전 그대로\n'
                   '    smooth_accel: 0.4               # [m/s^2] 속도 바뀌는 빠르기 (작게 = 더 부드럽게, 0.2 -> 0 이 0.5초)\n'
                   '    smooth_curve_m: 0.02            # [m] 곡선으로 서서히 꺾는 거리 (크게 = 더 부드럽게. 그 절반만큼 일찍 꺾어서 길은 그대로)\n') + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('부드럽게 OK')

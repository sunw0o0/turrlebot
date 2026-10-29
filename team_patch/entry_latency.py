# parking_node: 입구 곡선 시작을 "카메라 지연 x 속도" 만큼 앞당김 (속도를 바꿔도 같은 곳에서 꺾게)
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns, ys = open(n).read(), open(y).read()
if 'entry_latency_s' in ns:
    print('이미 들어가 있어요. 바꾸지 않았어요'); sys.exit()
A = '    entry_forward_m_ = declare_parameter("entry_forward_m", 0.0);\n'
B = '''    const double extra_m = std::max(0.0, entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_);
    extra_time_ = extra_m / speed_;
    state_ = State::EXTRA;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "입구 시작 (노란선 끊김) -> %.1fcm 더 직진한 뒤 곡선으로 왼쪽", extra_m * 100.0);
'''
C = '  double entry_forward_m_;\n'
D = '    entry_forward_m: '
for txt, key in ((ns, A), (ns, B), (ns, C), (ys, D)):
    if txt.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()
ns = ns.replace(A, A + '''    // 노란선 끊김을 알아챘을 때 로봇은 이미 (카메라 + 계산 지연) x 속도 만큼 지나와 있다 -> 그만큼 덜 직진
    //   (park5 녹화: 3번 모두 곡선 끝이 3~4cm 늦음 = 실제 속도 약 0.25 m/s x 0.14초)
    entry_latency_s_ = declare_parameter("entry_latency_s", 0.14);
''')
ns = ns.replace(B, '''    const double latency_m = speed_ * std::max(0.0, entry_latency_s_);   // 알아채는 동안 이미 간 거리
    const double extra_m = std::max(0.0, entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_ - latency_m);
    extra_time_ = extra_m / speed_;
    state_ = State::EXTRA;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "입구 시작 (노란선 끊김) -> %.1fcm 더 직진한 뒤 곡선으로 왼쪽 (지연 보정 -%.1fcm)",
      extra_m * 100.0, latency_m * 100.0);
''')
ns = ns.replace(C, C + '  double entry_latency_s_;\n')
lines = ys.split('\n')
for k, l in enumerate(lines):
    if l.startswith(D):
        lines.insert(k + 1, '    entry_latency_s: 0.14           # [s] 노란선 끊김을 알아채는 지연. 속도 x 이 시간만큼 일찍 꺾음 (속도 바꿔도 그대로). 아직 바깥(오른쪽) 노란선 밟으면 키우기')
        break
open(n, 'w').write(ns); open(y, 'w').write('\n'.join(lines))
print('입구 지연 보정 추가 OK')

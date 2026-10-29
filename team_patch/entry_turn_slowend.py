# (entry_turn_fast.py 다음에) 입구 곡선: 앞부분만 빠르게, entry_turn_slow_deg 넘게 돈 뒤에는 원래 빠르기로.
#   park7 녹화: 끝까지 빠르게 돌면 왼쪽으로 조금 더 돌아서 (올라가며 오른쪽으로 바로잡는 양 3배) 왼쪽 노란선에 가까워짐.
#   끝나는 부분을 예전 빠르기로 돌면 멈추는 각도가 예전과 같다. 곡선 반지름은 그대로
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'entry_turn_slow_deg_' in ns:
    print('이미 들어가 있어요')
    sys.exit()
if 'entry_turn_match_' not in ns:
    print('entry_turn_fast.py 를 먼저 넣어 주세요')
    sys.exit()
R = [
    ('    entry_turn_w_ = param("entry_turn_w", 0.6);',
     '    entry_turn_slow_deg_ = param("entry_turn_slow_deg", 45.0);   // 카메라로 이만큼 돈 뒤에는 원래 빠르기 (entry_arc_speed)\n'
     '    entry_turn_w_ = param("entry_turn_w", 0.6);'),
    ('    const double cam_deg = std::abs(entry_cam_sum_) * 180.0 / M_PI;\n',
     '    const double cam_deg = std::abs(entry_cam_sum_) * 180.0 / M_PI;\n'
     '    // 빠르게 도는 중이면 entry_turn_slow_deg 넘게 돈 뒤 원래 빠르기로 (반지름은 그대로). 끝까지 빠르면 더 돌아버림\n'
     '    if (!leaving_ && entry_v_ > entry_arc_speed_ && (entry_cam_failed_ || cam_deg >= entry_turn_slow_deg_))\n'
     '    {\n'
     '      entry_w_ *= entry_arc_speed_ / entry_v_;\n'
     '      entry_v_ = entry_arc_speed_;\n'
     '      RCLCPP_INFO(get_logger(), "입구 곡선 %.0f도 돔 -> 원래 빠르기 (%.2fm/s, %.2frad/s)", cam_deg, entry_v_, entry_w_);\n'
     '      drive(entry_v_, entry_w_);\n'
     '    }\n'),
    ('  double entry_turn_w_;\n',
     '  double entry_turn_w_;\n'
     '  double entry_turn_slow_deg_;\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:60]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
YA = '    entry_turn_w:'
if 'entry_turn_slow_deg' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + '    entry_turn_slow_deg: 45.0       # [도] 이만큼 돈 뒤에는 원래 빠르기로 (끝나는 각도가 예전과 같게). 크게 = 더 오래 빠르게 (왼쪽 노란선에 가까워짐)\n' + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('입구 곡선 끝부분 원래 빠르기 OK')

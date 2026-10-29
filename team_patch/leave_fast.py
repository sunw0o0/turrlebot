# (slow_near.py, entry_turn_fast.py 다음에) 주차에서 나올 때 감속 없이:
#   (1) 도로까지 직진: 노란선 끝이 가까워져도 안 줄임 (leave_slow: false). 대신 빨리 가는 만큼
#       끊김을 늦게 알아채니 그만큼 덜 직진 (entry_latency_s 와 같은 계산, 원래 속도 drive_speed 와의 차이만)
#   (2) 도로로 나가는 곡선: 입구 곡선처럼 일반 주행 코너 빠르기 (entry_turn_w) 까지. 길은 그대로.
#       반지름이 작은 (급한) 곡선은 예전보다 빨리 돌지 않게
#   (3) 노란선 "보임" 은 처음부터 세고, "끊김" 만 leave_min 뒤부터 센다
#       (빨리 가면 leave_min 이 지난 뒤 노란선이 보이는 영상이 1~2장뿐이라 "보임" 을 못 채우고 끊김을 놓칠 수 있음)
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'leave_turn_match_' in ns:
    print('이미 들어가 있어요')
    sys.exit()
for need, script in (('slow_by_row', 'slow_near.py'), ('entry_turn_match_', 'entry_turn_fast.py')):
    if need not in ns:
        print(script, '를 먼저 넣어 주세요')
        sys.exit()
R = [
    ('    entry_turn_w_ = param("entry_turn_w", 0.6);',
     '    leave_turn_match_ = param("leave_turn_match", true);   // 도로로 나가는 곡선도 일반 주행 코너 빠르기로\n'
     '    leave_slow_ = param("leave_slow", false);              // 나갈 때 노란선 끝이 가까워지면 줄이기\n'
     '    entry_turn_w_ = param("entry_turn_w", 0.6);'),
    ('    if (!leaving_ && entry_turn_match_ && run_speed_ > entry_v_)\n',
     '    if (((!leaving_ && entry_turn_match_) || (leaving_ && leave_turn_match_)) && run_speed_ > entry_v_)\n'),
    ('    entry_w_ = entry_v_ / radius_cmd;\n',
     '    entry_w_ = entry_v_ / radius_cmd;\n'
     '    entry_v_max_ = entry_v_;\n'),
    ('    entry_w_ = entry_v_ / radius;\n',
     '    // 빠르게 나가는 중이어도 급한 곡선 (작은 반지름) 은 회전이 entry_turn_w 를 넘지 않게 (예전보다 빨리 돌지는 않음)\n'
     '    entry_v_ = std::min(entry_v_max_, std::max(entry_arc_speed_, entry_turn_w_ * radius));\n'
     '    entry_w_ = entry_v_ / radius;\n'),
    ('      if (leave_turn_)\n'
     '      {\n'
     '        slow_by_row(yellow_end_y(yellow)',
     '      if (leave_turn_ && leave_slow_)\n'
     '      {\n'
     '        slow_by_row(yellow_end_y(yellow)'),
    ('      entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_ + leave_forward_m_);\n',
     '      entry_gap_to_center_m_ - entry_arc_radius_m_ + entry_forward_m_ + leave_forward_m_ -\n'
     '      std::max(0.0, speed_ - drive_speed_) * std::max(0.0, entry_latency_s_));   // 원래 속도보다 빠른 만큼 늦게 알아챔\n'),
    ('  double entry_turn_w_;\n',
     '  double entry_turn_w_;\n'
     '  bool leave_turn_match_, leave_slow_;\n'
     '  double entry_v_max_ = 0.08;      // 이번 ENTRY_ARC 의 최고 직진 속도 [m/s]\n'),
    ('      if (leave_turn_ && leave_min_ok && leave_gap_reached(yellow))\n',
     '      if (leave_turn_ && leave_gap_reached(yellow, leave_min_ok))\n'),
    ('  bool leave_gap_reached(const cv::Mat & yellow)\n',
     '  bool leave_gap_reached(const cv::Mat & yellow, bool allow_break)\n'),
    ('      return false;\n'
     '    }\n'
     '    leave_empty_ = count <= empty_pixels_ ? leave_empty_ + 1 : 0;\n',
     '      return false;\n'
     '    }\n'
     '    if (!allow_break)\n'
     '    {\n'
     '      leave_empty_ = 0;          // 곡선 직후 (leave_min) 에는 끊김을 세지 않음 ("보임" 은 미리 세 둠)\n'
     '      return false;\n'
     '    }\n'
     '    leave_empty_ = count <= empty_pixels_ ? leave_empty_ + 1 : 0;\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:60]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
YA = '    entry_turn_w:'
if 'leave_turn_match' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + ('    leave_turn_match: true          # 도로로 나가는 곡선도 일반 주행 코너 빠르기로 (길은 그대로). false = entry_arc_speed\n'
                   '    leave_slow: false               # true = 나갈 때 노란선 끝이 가까워지면 drive_speed 까지 줄임\n') + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('감속 없이 나가기 OK')

# 노란선 두 개 사이 (주차장 입구) 로 들어가는 곡선만 원래 속도 (entry_arc_speed) 로.
#   나머지 (접근 / 나오는 곡선 / 도로로 나가는 곡선) 는 그대로 일반 주행 속도.
#   parking 폴더 (colcon_ws/src/parking) 에서 실행. match_speed.py 를 먼저 넣어야 한다. 끝나면 live_params.py 다시 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'match_entry_arc_' in ns:
    print('이미 들어가 있어요'); sys.exit()
if 'entry_arc_speed_base_' not in ns:
    print('match_speed.py 를 먼저 넣어 주세요'); sys.exit()
R = [
    ('    match_speed_max_ = declare_parameter("match_speed_max", 0.25);\n',
     '    match_speed_max_ = declare_parameter("match_speed_max", 0.25);\n'
     '    match_entry_arc_ = declare_parameter("match_entry_arc", false);   // false = 입구 곡선만 entry_arc_speed\n'),
    ('    exit_arc_speed_ = exit_arc_speed_base_;\n',
     '    exit_arc_speed_ = exit_arc_speed_base_;\n'
     '    speed_matched_ = false;\n'),
    ('      exit_arc_speed_ = speed_;\n',
     '      exit_arc_speed_ = speed_;\n'
     '      speed_matched_ = true;\n'),
    ('    const double radius_cmd = std::max(entry_arc_radius_m_ * entry_arc_radius_gain_, entry_arc_min_radius_m_);\n',
     '    if (speed_matched_)\n'
     '    {\n'
     '      // 노란선 사이로 들어가는 곡선은 원래 속도 (match_entry_arc false), 도로로 나가는 곡선은 일반 주행 속도\n'
     '      entry_arc_speed_ = (leaving_ || match_entry_arc_) ? run_speed_ : entry_arc_speed_base_;\n'
     '    }\n'
     '    const double radius_cmd = std::max(entry_arc_radius_m_ * entry_arc_radius_gain_, entry_arc_min_radius_m_);\n'),
    ('  double entry_arc_speed_;\n',
     '  double entry_arc_speed_;\n'
     '  bool match_entry_arc_;\n'
     '  bool speed_matched_ = false;   // 이번 주차가 일반 주행 속도를 따라가는 중\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:60]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
YA = '    match_drive_speed: true'
if 'match_entry_arc' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + '    match_entry_arc: false          # false = 노란선 두 개 사이로 들어가는 곡선만 entry_arc_speed (원래 속도). true = 거기도 일반 주행 속도\n' \
        + '    entry_arc_speed: 0.08           # [m/s] 노란선 사이로 들어가는 곡선 속도\n' * ('    entry_arc_speed:' not in ys) + ys[i:]
    open(y, 'w').write(ys)
open(n, 'w').write(ns)
print('입구 곡선 원래 속도 OK -> 이제 live_params.py 다시 실행')

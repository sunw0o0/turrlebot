# (Jetson 버전 parking_node: param() / loadParams 방식)
# 주차장 입구 (노란선 두 개 사이) 로 들어가는 곡선만 일반 주행이 코너에서 도는 빠르기로.
#   곡선 반지름 (= 지나가는 길, 도는 양) 은 그대로, 회전 속도와 직진 속도만 같은 비율로 올린다.
#   기본: 회전 0.4 -> 0.6 rad/s (park6 녹화의 일반 주행 코너 회전 0.55 ~ 0.66), 직진 0.08 -> 0.12
#   도로로 나가는 곡선은 그대로 (entry_arc_speed)
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'entry_turn_match_' in ns:
    print('이미 들어가 있어요')
    sys.exit()

R = [
    ('    entry_arc_speed_base_ = param("entry_arc_speed", 0.08);\n',
     '    entry_arc_speed_base_ = param("entry_arc_speed", 0.08);\n'
     '    // 주차장으로 들어가는 곡선만 일반 주행 코너 빠르기로 (곡선 모양은 그대로)\n'
     '    entry_turn_match_ = param("entry_turn_match", true);\n'
     '    entry_turn_w_ = param("entry_turn_w", 0.6);   // [rad/s 명령] 일반 주행 코너 회전 (grid_drive planner.max_angular 1.5 실제 / 2.45)\n'),
    ('    entry_w_ = entry_arc_speed_ / radius_cmd;\n',
     '    entry_v_ = entry_arc_speed_;\n'
     '    if (!leaving_ && entry_turn_match_ && run_speed_ > entry_v_)\n'
     '    {\n'
     '      // 주차장으로 들어가는 곡선: 일반 주행이 코너에서 도는 빠르기 (entry_turn_w) 로. 반지름(길)은 그대로라 직진 속도도 같이 오른다\n'
     '      entry_v_ = std::max(entry_v_, std::min(run_speed_, entry_turn_w_ * radius_cmd));\n'
     '    }\n'
     '    entry_w_ = entry_v_ / radius_cmd;\n'),
    ('      entry_arc_radius_m_ * 100.0, radius_cmd * 100.0, entry_arc_speed_);\n',
     '      entry_arc_radius_m_ * 100.0, radius_cmd * 100.0, entry_v_);\n'),
    ('  double entry_arc_speed_;\n',
     '  double entry_arc_speed_;\n'
     '  double entry_v_ = 0.08;          // 이번 ENTRY_ARC 의 직진 속도 [m/s]\n'
     '  bool entry_turn_match_;\n'
     '  double entry_turn_w_;\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:60]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
if ns.count('drive(entry_arc_speed_, entry_w_)') != 4 or ns.count('entry_w_ = entry_arc_speed_ / radius;') != 1:
    print('파일이 예상과 달라요. 바꾸지 않았어요: drive(entry_arc_speed_, entry_w_)'); sys.exit()
ns = ns.replace('drive(entry_arc_speed_, entry_w_)', 'drive(entry_v_, entry_w_)')
ns = ns.replace('entry_w_ = entry_arc_speed_ / radius;', 'entry_w_ = entry_v_ / radius;')

YA = '    entry_arc_speed:'
if 'entry_turn_match' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + ('    entry_turn_match: true          # 노란선 사이로 들어가는 곡선만 일반 주행 코너 빠르기로 (곡선 모양은 그대로). false = entry_arc_speed\n'
                   '    entry_turn_w: 0.6               # [rad/s] 그때 회전 속도 (원래 0.4). 일반 주행 코너 0.55 ~ 0.66\n') + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('입구 곡선 빠르게 OK')

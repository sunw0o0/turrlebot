# (Jetson 버전 parking_node: param() / loadParams 방식) 주차: 직진은 일반 주행 속도, 곡선(회전)은 원래 속도로.
#   일반 주행 속도: 칸 앞까지 직진 / 입구 뒤 통로 직진 / 칸에서 나와 도로까지 직진
#   원래 속도:     노란선 사이로 들어가는 곡선 + 도로로 나가는 곡선 (entry_arc_speed)
#                  칸 안으로 (side_arc_speed, backup_speed) / 칸에서 나오는 곡선 (exit_arc_speed)
#   속도 값은 바꾸기 전으로: drive_speed 0.10, entry_arc_speed 0.08, max_angular 0.6
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import re, sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()

if 'match_turn_speed_' in ns:
    print('코드는 이미 들어가 있어요')
else:
    R = [
        ('      entry_arc_speed_ = speed_;\n'
         '      exit_arc_speed_ = speed_;\n',
         '      if (match_turn_speed_)   // false 면 곡선은 entry_arc_speed / exit_arc_speed 그대로\n'
         '      {\n'
         '        entry_arc_speed_ = speed_;\n'
         '        exit_arc_speed_ = speed_;\n'
         '      }\n'),
        ('    match_speed_max_ = param("match_speed_max", 0.25);\n',
         '    match_speed_max_ = param("match_speed_max", 0.25);\n'
         '    match_turn_speed_ = param("match_turn_speed", false);   // false = 곡선은 원래 속도 (entry_arc_speed / exit_arc_speed)\n'),
        ('    if (!i_am_driving())\n'
         '    {\n'
         '      entry_arc_speed_ = entry_arc_speed_base_;\n',
         '    if (!i_am_driving() || !match_turn_speed_)\n'
         '    {\n'
         '      entry_arc_speed_ = entry_arc_speed_base_;\n'),
        ('  double match_speed_min_, match_speed_max_;\n',
         '  double match_speed_min_, match_speed_max_;\n'
         '  bool match_turn_speed_ = false;\n'),
    ]
    for a, _ in R:
        if ns.count(a) != 1:
            print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:60]); sys.exit()
    for a, b in R:
        ns = ns.replace(a, b)
    ns = ns.replace('"주차 속도 = 일반 주행 속도 %.2f m/s (칸으로 들어가는 동안만 원래 속도)"',
                    '"주차 직진 속도 = 일반 주행 속도 %.2f m/s (곡선 / 칸 안은 원래 속도)"')
    ns = ns.replace('      // 일반 주행 속도 그대로 (직진 / 입구 곡선 / 나오는 곡선 모두). 곡선 반지름은 속도와 상관없이 그대로다\n',
                    '      // 직진은 일반 주행 속도 그대로. 곡선은 match_turn_speed 일 때만. 곡선 반지름은 속도와 상관없이 그대로다\n')
    open(n, 'w').write(ns)
    print('직진 = 일반 주행 속도, 곡선 = 원래 속도 OK')

YA = '    match_drive_speed: true'
if 'match_turn_speed' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + '    match_turn_speed: false         # false = 곡선 (입구 / 칸 나오기 / 도로로 나가기) 은 entry_arc_speed / exit_arc_speed 그대로. true = 곡선도 일반 주행 속도\n' + ys[i:]
for key, val in (('drive_speed', '0.10'), ('entry_arc_speed', '0.08'), ('max_angular', '0.6')):
    m = re.search(r'^    ' + key + r': *([0-9.]+)', ys, re.M)
    if m and float(m.group(1)) != float(val):
        ys = ys[:m.start(1)] + val + ys[m.end(1):]
        print(f'{key} {m.group(1)} -> {val}')
open(y, 'w').write(ys)
print('yaml OK')

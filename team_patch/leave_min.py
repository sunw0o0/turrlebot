# parking_node: 나가는 길 (EXIT_STEM) 을 leave_min_time 초 내려간 뒤부터 노란선 끊김을 봄 + 나가는 곡선 반지름 키우기
import re, sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns, ys = open(n).read(), open(y).read()
if 'leave_min_time' in ns:
    print('이미 들어가 있어요. 바꾸지 않았어요'); sys.exit()
a = '      if (leave_turn_ && leave_gap_reached(yellow))\n'
b = '    leave_rearm_delay_ = declare_parameter("leave_rearm_delay"'
c = '  double exit_dash_gain_;' if '  double exit_dash_gain_;' in ns else '  double bev_near_m_, aisle_width_m_, exit_dash_gain_;\n'
d = '    leave_rearm_delay: '
for txt, key in ((ns, a), (ns, b), (ns, c), (ys, d)):
    if txt.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()
ns = ns.replace(a, '''      // 곡선이 막 끝나 길 가운데로 오는 동안은 끊김을 안 봄 (한쪽 노란선이 화면 밖으로 잠깐 빠져서 착각함)
      if (leave_turn_ && state_time_ >= leave_min_time_ && leave_gap_reached(yellow))
''')
i = ns.index(b)
ns = ns[:i] + '''    // 나가는 길을 이 시간 [s] 내려간 뒤부터 왼쪽 노란선 끊김을 본다
    leave_min_time_ = declare_parameter("leave_min_time", 1.0);
''' + ns[i:]
ns = ns.replace(c, c + '  double leave_min_time_;\n')
ys = re.sub(r'(\n    exit_dash_gain: )[0-9.]+', r'\g<1>1.45', ys)
lines = ys.split('\n')
for k, l in enumerate(lines):
    if l.startswith(d):
        lines.insert(k + 1, '    leave_min_time: 1.0             # [s] 나가는 길을 이만큼 내려간 뒤부터 노란선 끊김을 봄 (곡선 직후 착각 막기)')
        break
open(n, 'w').write(ns); open(y, 'w').write('\n'.join(lines))
print('나가기 수정 OK')

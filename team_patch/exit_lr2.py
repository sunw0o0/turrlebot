# parking_node: 나가는 곡선 반지름 비율을 왼쪽 / 오른쪽 따로 (exit_dash_gain_left / _right). lane_follower 폴더에서 실행
# (코드만 고침. yaml 에 exit_dash_gain_left / _right 줄은 직접 넣는다)
import re, sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns, ys = open(n).read(), open(y).read()
if 'exit_dash_gain_left' in ns:
    print('이미 들어가 있어요. 바꾸지 않았어요'); sys.exit()
a = '    exit_dash_gain_ = declare_parameter("exit_dash_gain", 1.25);     // 실제 반지름 -> 명령 반지름 (0 = 점선 안 씀)\n'
b = '''    if (dash_align_ && exit_dash_gain_ > 0.0 && dash_fresh(1.0))
    {
      const double real = dash_dist_ - aisle_width_m_ / 2.0;
      const double cmd = std::clamp(real * exit_dash_gain_, exit_arc_min_radius_m_, 0.6);
'''
c = '  double bev_near_m_, aisle_width_m_, exit_dash_gain_;\n'
for txt, key in ((ns, a), (ns, b), (ns, c)):
    if txt.count(key) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', key.strip()[:50]); sys.exit()
ns = ns.replace(a, a + '''    // 왼쪽 / 오른쪽으로 나갈 때 따로 (0 보다 작으면 exit_dash_gain 을 씀)
    exit_dash_gain_left_ = declare_parameter("exit_dash_gain_left", -1.0);
    exit_dash_gain_right_ = declare_parameter("exit_dash_gain_right", -1.0);
''')
ns = ns.replace(b, '''    // 이번에 쓸 비율: 오른쪽으로 나가면 exit_dash_gain_right, 왼쪽이면 _left (0 보다 작으면 exit_dash_gain)
    const double side_gain = side_direction_ < 0.0 ? exit_dash_gain_right_ : exit_dash_gain_left_;
    const double gain = side_gain >= 0.0 ? side_gain : exit_dash_gain_;
    if (dash_align_ && gain > 0.0 && dash_fresh(1.0))
    {
      const double real = dash_dist_ - aisle_width_m_ / 2.0;
      const double cmd = std::clamp(real * gain, exit_arc_min_radius_m_, 0.6);
''')
ns = ns.replace(c, c + '  double exit_dash_gain_left_, exit_dash_gain_right_;\n')
open(n, 'w').write(ns)
print('왼쪽/오른쪽 따로 (코드) OK')

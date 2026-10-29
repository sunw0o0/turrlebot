# (leave_turn_pos.py 다음에) 도로로 나가는 곡선을 처음부터 더 작게 (흰 선 밟지 않게)
#   park13: 나간 뒤 도로 흰 선까지 9~13cm (목표 15, park12 는 11~16) -> 오른쪽 바퀴가 흰 선 위.
#   도로 흰 선이 보이기 전까지 반지름 20cm 로 크게 돌다가, 선이 보이면 이미 바깥으로 나가 있어서
#   가장 작은 반지름 (10cm) 으로도 못 돌아옴 (로그: 매번 "반지름 10cm").
#   -> 나갈 때만 처음 곡선 반지름을 leave_arc_radius_m (12cm) 로. 꺾기 시작하는 곳은 그대로라
#      같은 곳에서 꺾어도 더 안쪽으로 돈다. 그 뒤는 예전처럼 흰 선을 보며 맞춤
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'leave_arc_radius_m_' in ns:
    print('이미 들어가 있어요'); sys.exit()
if 'leave_latency_s_' not in ns:
    print('leave_turn_pos.py 를 먼저 넣어 주세요'); sys.exit()
R = [
    ('    const double radius_cmd = std::max(entry_arc_radius_m_ * entry_arc_radius_gain_, entry_arc_min_radius_m_);\n',
     '    const double radius_cmd = leaving_ ? leave_radius_cmd() :\n'
     '      std::max(entry_arc_radius_m_ * entry_arc_radius_gain_, entry_arc_min_radius_m_);\n'),
    ('(기본 0)\n      curve_ramp_shift(entry_radius_cmd()));',
     '(기본 0)\n      curve_ramp_shift(leave_radius_cmd()));'),
    ('  double entry_radius_cmd() const\n',
     '''  // 도로로 나가는 곡선의 처음 명령 반지름 [m] (leave_arc_radius_m, 0 이면 들어갈 때와 같게)
  double leave_radius_cmd() const
  {
    if (leave_arc_radius_m_ > 0.0)
    {
      return std::max(leave_arc_radius_m_, entry_arc_min_radius_m_);
    }
    return entry_radius_cmd();
  }

  double entry_radius_cmd() const
'''),
    ('    leave_latency_s_ = param("leave_latency_s", 0.0);',
     '    leave_arc_radius_m_ = param("leave_arc_radius_m", 0.12);   // 도로로 나가는 곡선 처음 반지름 (흰 선 밟으면 줄이기, 노란선 쪽으로 붙으면 키우기)\n'
     '    leave_latency_s_ = param("leave_latency_s", 0.0);'),
    ('  double leave_latency_s_ = 0.0;\n',
     '  double leave_latency_s_ = 0.0;\n'
     '  double leave_arc_radius_m_ = 0.12;\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:70]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
YA = '    leave_forward_m:'
if 'leave_arc_radius_m' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + '    leave_arc_radius_m: 0.12        # [m] 도로로 나가는 곡선 처음 반지름 (원래 0.20). 흰 선 밟으면 줄이기 (최소 0.10), 노란선 쪽으로 붙으면 키우기\n' + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('나갈 때 작게 돌기 OK')

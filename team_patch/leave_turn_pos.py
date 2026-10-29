# 주차에서 나갈 때 도로로 꺾는 위치가 예전보다 일찍 (앞에서) 꺾던 것 되돌림
#   park6 (예전, 0.10 m/s): 노란선 끝이 끊김 칸을 지나고 14.1cm (실제) 뒤에 꺾기 시작
#   park12 (지금, 0.19 m/s): 8.2cm 뒤 -> 약 6cm 일찍
#   leave_fast.py 에서 "빨리 가면 끊김을 늦게 알아챈다" 고 보고 (속도 - drive_speed) x 0.14초 만큼 덜 직진했는데,
#   park12 를 보면 빨리 가도 끊김을 늦게 알아채지 않았다 (화면에서 끊긴 때 20.02초, 로그 20.023초)
#   -> 그 보정을 leave_latency_s (기본 0) 로 따로 떼어 냄. 서서히 꺾는 만큼 일찍 시작하는 것은 그대로
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'leave_latency_s_' in ns:
    print('이미 들어가 있어요'); sys.exit()
R = [
    ('      std::max(0.0, speed_ - drive_speed_) * std::max(0.0, entry_latency_s_) -\n'
     '      curve_ramp_shift(entry_radius_cmd()));   // 원래 속도보다 빠른 만큼 늦게 알아챔\n',
     '      std::max(0.0, speed_ - drive_speed_) * std::max(0.0, leave_latency_s_) -   // 빠른 만큼 늦게 알아채면 (기본 0)\n'
     '      curve_ramp_shift(entry_radius_cmd()));   // 서서히 꺾는 만큼 일찍\n'),
    ('    leave_forward_m_ = param("leave_forward_m", 0.0);\n',
     '    leave_forward_m_ = param("leave_forward_m", 0.0);\n'
     '    leave_latency_s_ = param("leave_latency_s", 0.0);   // 나갈 때 끊김을 늦게 알아채는 시간 (park12: 늦지 않음 -> 0)\n'),
    ('  double leave_min_time_;\n',
     '  double leave_min_time_;\n'
     '  double leave_latency_s_ = 0.0;\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:70]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
YA = '    leave_forward_m:'
if 'leave_latency_s' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + '    leave_latency_s: 0.0            # [s] 나갈 때 빨리 가는 만큼 일찍 꺾는 시간 (0 = 안 함). 늦게 꺾으면 0.05 씩 올리기\n' + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('나갈 때 꺾는 위치 되돌림 OK')

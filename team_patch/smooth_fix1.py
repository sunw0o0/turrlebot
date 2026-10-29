# (smooth_drive.py 고침) 주차 시작할 때 첫 영상이 오기 전에 "명령 끊김" 으로 0 이 되고 0 부터 다시 올라가던 것
#   park9: 8번 중 6번, 주차 시작 때 0.19 -> 0.01 -> 0.19 (0.45초). 일반 주행 속도를 목표로 이어받게
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
ns = open(n).read()
A = ('    v_out_ = fresh ? std::clamp(seen_drive_v_, 0.0, 0.5) : 0.0;   // 일반 주행이 가던 속도에서 이어서\n'
     '    w_out_ = 0.0;\n'
     '    k_out_ = 0.0;\n')
B = ('    tgt_v_ = v_out_;           // 첫 영상이 오기 전에 "명령 끊김" 으로 멈추지 않게 (그동안 지금 속도로 직진)\n'
     '    tgt_w_ = 0.0;\n'
     '    tgt_t_ = now().seconds();\n')
if B in ns:
    print('이미 들어가 있어요'); sys.exit()
if ns.count(A) != 1:
    print('smooth_drive.py 를 먼저 넣어 주세요 (또는 파일이 예상과 달라요)'); sys.exit()
ns = ns.replace(A, A + B)
open(n, 'w').write(ns)
print('주차 시작 때 멈춤 고침 OK')

# 도로로 나가는 곡선 (보면서 도로로) 이 흰 상자 같은 덩어리를 "도로 흰 선" 으로 잘못 봐서 일찍 끝나던 것
#   park10 로그: 7번 중 3번 "기울기 -70도 -> 0.3초 뒤 +8~+13도 (지나침)" 로 끝남. 실제로는 덜 돈 상태 (일반 주행이 제자리로 마저 돎).
#   그때마다 화면에 도로 옆 흰 상자 덩어리가 있었고, 그 덩어리에 직선을 맞추면 기울기 +11~+27도가 나옴.
#   (1) 직선에서 벗어난 정도 (두께) 가 leave_guide_max_rms_px 보다 큰 덩어리는 선이 아님 (park6 녹화: 도로 흰 선 3.3~5.6px, 상자는 그 10배 이상)
#   (2) 기울기가 멀리 (-25도 아래) 있다가 갑자기 + (지나침) 로 바뀌면 잘못 본 것으로 치고 하던 곡선 그대로
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'leave_guide_max_rms_px_' in ns:
    print('이미 들어가 있어요'); sys.exit()
R = [
    ('  std::optional<cv::Vec2d> find_exit_guide(const cv::Mat & yellow, double direction) const\n',
     '  std::optional<cv::Vec2d> find_exit_guide(const cv::Mat & yellow, double direction, double max_rms_px = 0.0) const\n'),
    ('      const double lean = std::atan(-b);                       // 앞으로 갈수록 x 가 커지면 +\n',
     '      const double lean = std::atan(-b);                       // 앞으로 갈수록 x 가 커지면 +\n'
     '      if (max_rms_px > 0.0)\n'
     '      {\n'
     '        // 선이 아닌 덩어리 (도로 옆 흰 상자 등) 는 뺀다: 직선에서 벗어난 정도 (= 두께) 가 크면\n'
     '        double se = 0.0;\n'
     '        for (const auto & p : pts)\n'
     '        {\n'
     '          if (p.y >= y_max - 150)\n'
     '          {\n'
     '            const double e = p.x - (a + b * p.y);\n'
     '            se += e * e;\n'
     '          }\n'
     '        }\n'
     '        if (std::sqrt(se / sn) * std::cos(lean) > max_rms_px)\n'
     '        {\n'
     '          continue;\n'
     '        }\n'
     '      }\n'),
    ('find_exit_guide(white, 1.0)', 'find_exit_guide(white, 1.0, leave_guide_max_rms_px_)'),
    ('    if (phi <= leave_guide_align_deg_ * M_PI / 180.0 || lean > 0.0)\n',
     '    // 멀리 기울어 있다가 (-25도 아래) 갑자기 + (지나침) 로 바뀜 = 잘못 봄 -> 하던 곡선 그대로\n'
     '    if (lean > 0.0 && leave_last_lean_ <= -25.0 * M_PI / 180.0)\n'
     '    {\n'
     '      drive(entry_v_, entry_w_);\n'
     '      return true;\n'
     '    }\n'
     '    leave_last_lean_ = lean;\n'
     '    if (phi <= leave_guide_align_deg_ * M_PI / 180.0 || lean > 0.0)\n'),
    ('    leave_guide_active_ = false;\n',
     '    leave_guide_active_ = false;\n'
     '    leave_last_lean_ = -M_PI / 2.0;\n'),
    ('    leave_guide_min_radius_m_ = param("leave_guide_min_radius_m", 0.10);\n',
     '    leave_guide_min_radius_m_ = param("leave_guide_min_radius_m", 0.10);\n'
     '    leave_guide_max_rms_px_ = param("leave_guide_max_rms_px", 10.0);   // 이보다 두꺼운 흰 덩어리는 선이 아님 (상자 등)\n'),
    ('  double leave_guide_offset_m_, leave_guide_align_deg_, leave_guide_min_radius_m_;\n',
     '  double leave_guide_offset_m_, leave_guide_align_deg_, leave_guide_min_radius_m_;\n'
     '  double leave_guide_max_rms_px_ = 10.0;\n'
     '  double leave_last_lean_ = -M_PI / 2.0;   // 보면서 도로로: 바로 전 영상의 흰 선 기울기 [rad]\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:70]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
YA = '    leave_guide_align_deg:'
if 'leave_guide_max_rms_px' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + '    leave_guide_max_rms_px: 10.0    # [px] 이보다 두꺼운 흰 덩어리 (도로 옆 상자 등) 는 도로 흰 선으로 안 봄 (선은 3~6)\n' + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('흰 상자 착각 고침 OK')

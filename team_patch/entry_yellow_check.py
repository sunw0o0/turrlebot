# 주차 표지판을 늦게 봐서 (입구를 이미 지나서) 엉뚱한 곳에서 왼쪽으로 돌던 것 막기
#   park14 / park11: 표지판 인식 0.05~0.18초 뒤 "입구 시작" -> 굽은 도로에서 돌다가 (명령상 235도) 엉뚱한 길로 올라가
#   길 끝을 못 찾고 그만둠. 정상일 때는 끊기기 전에 왼쪽 아래 노란선이 0.5초 넘게 계속 보였는데 (420~440개),
#   실패 때는 그 전에 이미 0 과 300~400 을 오갔다 (park14: 노란선 0.2초, park11: 0.1초).
#   -> 표지판을 기다리는 동안에도 왼쪽 아래 칸의 노란선을 보고, 끊기기 전에 entry_yellow_min_s (0.5초) 넘게
#      계속 보였을 때만 입구로 친다. 아니면 이번엔 주차 안 하고 일반 주행에 넘김 (다음 바퀴에 다시)
#   녹화 확인 (park10~14): 정상 32번은 1.18~2.60초, 실패 2번은 0.00 / 0.11초
#   parking 폴더 (colcon_ws/src/parking) 에서 실행
import sys
n = 'src/parking_node.cpp'
y = 'config/parking.yaml'
ns = open(n).read()
ys = open(y).read()
if 'entry_yellow_min_s_' in ns:
    print('이미 들어가 있어요'); sys.exit()
R = [
    # 기다리는 동안에도 왼쪽 아래 노란선만 본다
    ('  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)\n'
     '  {\n'
     '    if (!i_am_driving())\n'
     '    {\n'
     '      return;\n'
     '    }\n',
     '  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)\n'
     '  {\n'
     '    if (!i_am_driving())\n'
     '    {\n'
     '      if (state_ == State::WAITING && entry_yellow_min_s_ > 0.0)\n'
     '      {\n'
     '        watch_left_yellow(msg);   // 표지판 기다리는 동안: 왼쪽 아래 노란선이 계속 보이는지만\n'
     '      }\n'
     '      return;\n'
     '    }\n'),
    # 입구 판단: 끊기기 전에 노란선을 충분히 봤는지
    ('    const int count = cv::countNonZero(yellow(cv::Range(y0, y1), cv::Range(0, x1)));\n'
     '\n'
     '    if (gap_ == Gap::BEFORE)\n'
     '    {\n'
     '      gap_frames_ = count <= empty_pixels_ ? gap_frames_ + 1 : 0;\n'
     '      if (gap_frames_ >= gap_frames_needed_)\n'
     '      {\n',
     '    const int count = cv::countNonZero(yellow(cv::Range(y0, y1), cv::Range(0, x1)));\n'
     '\n'
     '    if (gap_ == Gap::BEFORE)\n'
     '    {\n'
     '      track_left_yellow(count);\n'
     '      gap_frames_ = count <= empty_pixels_ ? gap_frames_ + 1 : 0;\n'
     '      if (gap_frames_ >= gap_frames_needed_ && entry_yellow_min_s_ > 0.0 && left_yellow_run() < entry_yellow_min_s_)\n'
     '      {\n'
     '        RCLCPP_WARN(get_logger(), "끊기기 전에 왼쪽 노란선을 %.2f초만 봄 (< %.2f초) -> 입구가 아님 (표지판을 늦게 봄). 이번엔 주차 안 함",\n'
     '          left_yellow_run(), entry_yellow_min_s_);\n'
     '        finish_leave();          // 일반 주행에 넘기고 다시 표지판 기다리기\n'
     '        return false;\n'
     '      }\n'
     '      if (gap_frames_ >= gap_frames_needed_)\n'
     '      {\n'),
    ('  bool entrance_reached(const cv::Mat & yellow)\n',
     '''  // 왼쪽 아래 칸 (watch_*) 의 노란 점 수로 "노란선이 계속 보인 시간" 을 잰다
  //   present_pixels 이상이면 보임 (이어짐), empty_pixels 이하가 0.15초 넘게 이어지면 끊김 (새로 잼)
  void track_left_yellow(int count)
  {
    const double t = now().seconds();
    if (count >= present_pixels_)
    {
      if (yellow_run_start_ < 0.0)
      {
        yellow_run_start_ = t;
      }
      yellow_run_last_ = t;
    }
    else if (count <= empty_pixels_ && yellow_run_start_ >= 0.0 && t - yellow_run_last_ > 0.15)
    {
      yellow_run_start_ = -1.0;
    }
  }

  // 마지막으로 노란선이 계속 보인 시간 [s] (없으면 0)
  double left_yellow_run() const
  {
    return yellow_run_start_ < 0.0 ? 0.0 : yellow_run_last_ - yellow_run_start_;
  }

  // 표지판 기다리는 동안: 영상의 왼쪽 아래 칸만 잘라서 노란 점 수를 센다 (가볍게)
  void watch_left_yellow(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv;
    try
    {
      cv = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception &)
    {
      return;
    }
    const cv::Mat & img = cv->image;
    const int y1 = std::min(static_cast<int>(watch_bottom_) + 1, img.rows);
    const int y0 = std::max(0, y1 - static_cast<int>(watch_rows_));
    const int x1 = std::min(static_cast<int>(watch_x_max_), img.cols);
    if (y1 <= y0 || x1 <= 0)
    {
      return;
    }
    cv::Mat lab;
    cv::cvtColor(img(cv::Range(y0, y1), cv::Range(0, x1)), lab, cv::COLOR_BGR2Lab);
    track_left_yellow(cv::countNonZero(yellow_mask(lab)));
  }

  bool entrance_reached(const cv::Mat & yellow)
'''),
    ('    leave_latency_s_ = param("leave_latency_s", 0.0);',
     '    entry_yellow_min_s_ = param("entry_yellow_min_s", 0.5);   // 끊기기 전에 왼쪽 노란선이 이만큼 계속 보여야 입구 (0 = 안 봄)\n'
     '    leave_latency_s_ = param("leave_latency_s", 0.0);'),
    ('  double leave_latency_s_ = 0.0;\n',
     '  double leave_latency_s_ = 0.0;\n'
     '  double entry_yellow_min_s_ = 0.5;\n'
     '  double yellow_run_start_ = -1.0, yellow_run_last_ = -1.0;   // 왼쪽 아래 노란선이 계속 보이기 시작한 / 마지막 시각\n'),
]
for a, _ in R:
    if ns.count(a) != 1:
        print('파일이 예상과 달라요. 바꾸지 않았어요:', a.strip()[:70]); sys.exit()
for a, b in R:
    ns = ns.replace(a, b)
YA = '    gap_frames_needed:'
if 'entry_yellow_min_s' not in ys and ys.count(YA) == 1:
    i = ys.index('\n', ys.index(YA)) + 1
    ys = ys[:i] + '    entry_yellow_min_s: 0.5         # [s] 끊기기 전에 왼쪽 노란선이 이만큼 계속 보여야 입구로 침 (표지판을 늦게 봤을 때 엉뚱한 곳에서 안 돌게). 0 = 안 봄\n' + ys[i:]
open(n, 'w').write(ns)
open(y, 'w').write(ys)
print('늦게 본 표지판 막기 OK')

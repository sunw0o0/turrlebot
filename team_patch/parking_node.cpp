// ============================================================================
//  parking_node.cpp - 주차 구간 (카메라만 보고: 입구에서 왼쪽으로 꺾기 -> 주차장 길 올라가기)
// ============================================================================
//
// [트랙 모양] (위에서 본 지도, 로봇은 아래 도로를 왼쪽 -> 오른쪽으로 달린다)
//
//              ┌──────┬──────┬──────┐
//              │ 왼쪽 │ 통로 │오른쪽│   <- 주차장
//              └──────┴╮    ╭┴──────┘
//                      │    │           <- 주차장 가는 길 (양옆 노란선)
//      노란선 ─────────╯ ★  ╰────────── 노란선  (입구 모서리는 둥글다)
//      ─────►  로봇 진행 방향
//      흰선   ──────────────────────────  (입구를 지나서도 곧게 이어진다)
//
// [전체 흐름]
//
//   (1) WAITING  : 주차 표지판을 기다린다 (운전은 lane_follower)
//          │  주차 표지판이 가까이 보임 (크기 sign_area_ratio_threshold 이상)
//          ▼
//   (2) APPROACH : 이 노드가 운전한다 (control_mode = PARKING -> lane_follower 멈춤)
//          │        오른쪽 흰선만 보고 똑바로 간다 (왼쪽 노란선이 입구로 휘어도 안 따라감)
//          │        동시에 BEV 왼쪽 아래 칸을 본다:
//          │          노란선 있음 -> 없음 (입구의 빈 곳이 지나감) -> 다시 있음 (입구 반대쪽 모서리)
//          │  "다시 있음" = 반대쪽 노란선이 화면 맨 아래에 닿음
//          │  입구 가운데는 반대쪽 선에서 차선 폭 절반(약 14.7cm) 앞.
//          │  화면 맨 아래가 바퀴 축에서 그보다 멀면(카메라 각도에 따라 14.1cm / 17.9cm),
//          │  바퀴 축이 아직 입구 가운데에 못 온 것 -> 남은 거리만큼 더 직진 (EXTRA)
//          ▼
//   (2-1) EXTRA  : 남은 거리 / 속도 만큼의 시간 동안 흰선 보고 더 직진 (남은 거리가 0 이면 건너뜀)
//          ▼
//   (3) TURN     : 제자리에서 왼쪽으로 돈다.
//          │        BEV 에 입구 길의 노란선 두 개(왼쪽, 오른쪽)가 세로로 똑바로 보이면 다음으로
//          ▼
//   (4) STEM     : 주차장 길을 올라간다. 양쪽이 노란선 -> 두 노란선의 가운데로 (흰선은 안 봄)
//          │        한쪽만 보이면 그 선에서 차선 폭 절반만큼 떨어져서 간다
//          │  노란선이 둘 다 연속으로 안 보임 = 길 끝 (주차장 입구)
//          ▼
//   (5) DONE     : 멈추고 control_mode = STOP ("parking_node:stem_end")
//                  (다음 단계 - 빈 칸 고르기, 주차 - 는 아직 없음)
//
// [위에서 본 영상(BEV)]  400 x 400 px, 1px = 1mm. 화면 맨 아래 = 바퀴 축 17.9cm 앞 (2026-10-13 카메라 각도)
//
//        x=0                    x=400
//    y=0  ┌──────────────────────┐
//         │  ╭──                 │   <- 입구 반대쪽 모서리 (내려온다)
//         │  │       흰선 ┃      │
//         │  ╰──          ┃      │   <- 입구 이쪽 모서리
//         │┌────┐         ┃      │
//  y=399  └┴────┴─────────┴──────┘
//          └ 왼쪽 아래 칸 (watch_x_max x watch_rows)
//
// [안전]
//   - 직진 중 정면 PSD 가 front_stop_distance 보다 가까우면 멈추고 기다린다
//   - 영상이 image_timeout 초 동안 안 오면 멈춘다
//   - GUI 에서 STOP / MANUAL 을 누르면 바로 멈추고 주차를 그만둔다
//   - approach_timeout / turn_timeout / stem_timeout 이 지나면 멈추고 그만둔다

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// 이 노드가 보내는 control_mode 의 mission_state 앞에 붙는 이름 (내가 보낸 걸 다시 받으면 무시하려고)
static const char * const MY_TAG = "parking_node";

// 화면 위의 직선 하나: x = a + b * y  (y 는 화면 줄 번호, 아래로 갈수록 커짐)
struct Line
{
  double a = 0.0;
  double b = 0.0;

  // 줄 y 에서의 가로 위치 [px]
  double x_at(double y) const
  {
    return a + b * y;
  }

  // 선이 로봇 정면에서 왼쪽으로 기운 각도 [rad]
  //   위(앞)로 갈수록 x 가 작아지면(b > 0) 선이 왼쪽으로 향한다
  double lean_left() const
  {
    return std::atan(b);
  }
};

class ParkingNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;

  enum class State
  {
    WAITING,     // 주차 표지판 기다리는 중 (운전은 lane_follower)
    APPROACH,    // 흰선 보고 직진하며 입구 찾는 중 (이 노드가 운전)
    EXTRA,       // 입구 가운데까지 남은 거리만큼 더 직진 (이 노드가 운전)
    TURN,        // 입구에서 왼쪽으로 도는 중 (이 노드가 운전)
    STEM,        // 주차장 길을 두 노란선 가운데로 올라가는 중 (이 노드가 운전)
    DONE,        // 끝 (또는 그만둠)
  };

  // 왼쪽 아래 칸의 노란선 변화 단계
  enum class Gap
  {
    BEFORE,      // 아직 입구 전 (노란선 있음)
    IN_GAP,      // 입구의 빈 곳 (노란선 없음)
  };

public:
  ParkingNode()
  : Node("parking")
  {
    declare_all_parameters();

    // ---- 받는 토픽 ----
    const auto qos = rclcpp::SensorDataQoS();
    sign_sub_ = create_subscription<Sign>(
      "sign", qos, [this](Sign::ConstSharedPtr msg)
      {
        on_sign(*msg);
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg)
      {
        on_image(msg);
      });
    psd_sub_ = create_subscription<interfaces::msg::PsdArray>(
      "psd", qos, [this](interfaces::msg::PsdArray::ConstSharedPtr msg)
      {
        psd_ = *msg;
        psd_received_ = true;
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg)
      {
        on_mode(*msg);
      });

    // ---- 보내는 토픽 ----
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);

    // ---- 0.1초마다: 모드 알림, 영상 끊김 / 시간 초과 확인 ----
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]
      {
        watchdog();
      });

    // 시험용: 표지판 없이 바로 시작
    if (skip_sign_)
    {
      RCLCPP_INFO(get_logger(), "주차 노드 시작: skip_sign -> 표지판 없이 바로 입구 찾기");
      start_approach();
      return;
    }
    RCLCPP_INFO(get_logger(), "주차 노드 시작: 주차 표지판 기다림");
  }

private:
  // ==================== 파라미터 (config/parking.yaml) ====================

  void declare_all_parameters()
  {
    // 1) 표지판
    min_confidence_ = declare_parameter("min_confidence", 0.5);
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.02);
    sign_hits_needed_ = declare_parameter("sign_hits_needed", 2);
    skip_sign_ = declare_parameter("skip_sign", false);

    // 2) 색 / 화면
    image_topic_ = declare_parameter("image_topic", std::string("image_bev"));
    yellow_l_min_ = declare_parameter("yellow_l_min", 80);
    yellow_b_min_ = declare_parameter("yellow_b_min", 150);
    yellow_a_max_ = declare_parameter("yellow_a_max", 135);
    white_l_min_ = declare_parameter("white_l_min", 190);
    white_ab_dev_ = declare_parameter("white_ab_dev", 15);
    bev_center_x_ = declare_parameter("bev_center_x", 0.5);
    lane_width_px_ = declare_parameter("lane_width_px", 295.0);
    line_min_rows_ = declare_parameter("line_min_rows", 40);
    flat_min_pixels_ = declare_parameter("flat_min_pixels", 35);

    // 3) 흰선 따라 직진
    approach_speed_ = declare_parameter("approach_speed", 0.06);
    heading_gain_ = declare_parameter("heading_gain", 1.0);
    offset_gain_ = declare_parameter("offset_gain", 1.0);
    offset_soft_px_ = declare_parameter("offset_soft_px", 300.0);
    max_angular_ = declare_parameter("max_angular", 0.6);
    robot_y_offset_px_ = declare_parameter("robot_y_offset_px", 179.0);

    // 4) 입구 찾기 (왼쪽 아래 칸)
    watch_x_max_ = declare_parameter("watch_x_max", 120);
    watch_rows_ = declare_parameter("watch_rows", 20);
    watch_bottom_ = declare_parameter("watch_bottom", 390);
    present_pixels_ = declare_parameter("present_pixels", 80);
    empty_pixels_ = declare_parameter("empty_pixels", 15);
    gap_frames_needed_ = declare_parameter("gap_frames_needed", 2);
    extra_forward_m_ = declare_parameter("extra_forward_m", 0.02);

    // 5) 회전
    turn_speed_ = declare_parameter("turn_speed", 0.4);
    turn_kp_ = declare_parameter("turn_kp", 1.5);
    turn_min_speed_ = declare_parameter("turn_min_speed", 0.08);
    aligned_deg_ = declare_parameter("aligned_deg", 1.5);
    parallel_deg_ = declare_parameter("parallel_deg", 8.0);
    pair_gap_min_px_ = declare_parameter("pair_gap_min_px", 200.0);
    pair_gap_max_px_ = declare_parameter("pair_gap_max_px", 380.0);
    turn_min_time_ = declare_parameter("turn_min_time", 1.0);
    turn_fit_rows_ = declare_parameter("turn_fit_rows", 230);

    // 6) 주차장 길 올라가기
    stem_speed_ = declare_parameter("stem_speed", 0.06);
    stem_end_frames_ = declare_parameter("stem_end_frames", 3);
    stem_min_time_ = declare_parameter("stem_min_time", 2.0);
    stem_timeout_ = declare_parameter("stem_timeout", 20.0);

    // 안전
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);
    image_timeout_ = declare_parameter("image_timeout", 0.5);
    approach_timeout_ = declare_parameter("approach_timeout", 30.0);
    turn_timeout_ = declare_parameter("turn_timeout", 8.0);
  }

  // ==================== 표지판 / 모드 ====================

  // 주차 표지판이 가까이 보이면 APPROACH 시작
  void on_sign(const Sign & sign)
  {
    const bool parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;

    sign_hits_ = parking_sign ? sign_hits_ + 1 : 0;

    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 (크기 %.3f, 신뢰도 %.2f) -> 흰선 보고 직진하며 입구 찾기",
        sign.area_ratio, sign.confidence);
      start_approach();
    }
  }

  // GUI 에서 STOP / MANUAL 을 누르면 그만둔다
  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state.rfind(MY_TAG, 0) == 0)
    {
      return;                    // 내가 보낸 것
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    if (stop_request && i_am_driving() && msg.mission_state == "turtle_gui")
    {
      give_up("GUI 에서 STOP/MANUAL");
    }
  }

  // ==================== 영상 한 장마다 ====================

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!i_am_driving())
    {
      return;
    }
    last_image_time_ = now();

    cv_bridge::CvImageConstPtr cv;
    try
    {
      cv = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception & e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상 변환 실패: %s", e.what());
      return;
    }

    cv::Mat lab;
    cv::cvtColor(cv->image, lab, cv::COLOR_BGR2Lab);
    const cv::Mat yellow = yellow_mask(lab);

    if (state_ == State::APPROACH)
    {
      if (entrance_reached(yellow))
      {
        start_extra();
        return;
      }
      drive_along_white(white_mask(lab));
    }
    else if (state_ == State::EXTRA)
    {
      drive_along_white(white_mask(lab));
    }
    else if (state_ == State::TURN)
    {
      turn_until_aligned(yellow);
    }
    else
    {
      drive_between_yellow(yellow);
    }
  }

  // ==================== 색 찾기 ====================

  // 노란색인 곳만 255
  cv::Mat yellow_mask(const cv::Mat & lab) const
  {
    cv::Mat mask;
    cv::inRange(lab, cv::Scalar(yellow_l_min_, 0, yellow_b_min_),
      cv::Scalar(255, yellow_a_max_, 255), mask);
    return clean(mask);
  }

  // 흰색인 곳만 255 (밝고, a/b 가 128 근처 = 색이 없음)
  cv::Mat white_mask(const cv::Mat & lab) const
  {
    cv::Mat mask;
    cv::inRange(lab, cv::Scalar(white_l_min_, 128 - white_ab_dev_, 128 - white_ab_dev_),
      cv::Scalar(255, 128 + white_ab_dev_, 128 + white_ab_dev_), mask);
    return clean(mask);
  }

  // 작은 점(잡음) 지우기
  static cv::Mat clean(const cv::Mat & mask)
  {
    cv::Mat out;
    const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, out, cv::MORPH_OPEN, k3);
    return out;
  }

  // 가로 범위 [x0, x1), 세로 줄 [0, y_end) 안의 선을 직선 x = a + b*y 로 잇는다
  //   줄마다 점들의 가운데 x 를 모은다. 점이 너무 많은 줄(가로로 누운 부분)은 뺀다.
  //   점이 있는 줄이 line_min_rows 보다 적으면 "선 없음"
  std::optional<Line> fit_line(const cv::Mat & mask, int x0, int x1, int y_end) const
  {
    double n = 0.0, sy = 0.0, sx = 0.0, syy = 0.0, sxy = 0.0;   // 최소제곱 합
    for (int y = 0; y < std::min(y_end, mask.rows); y++)
    {
      const cv::Moments m = cv::moments(mask.row(y).colRange(x0, x1), true);   // m00 = 점 개수
      if (m.m00 < 1.0 || m.m00 >= flat_min_pixels_)
      {
        continue;
      }
      const double x = x0 + m.m10 / m.m00;
      n += 1.0;
      sy += y;
      sx += x;
      syy += static_cast<double>(y) * y;
      sxy += x * y;
    }
    const double denom = n * syy - sy * sy;
    if (n < line_min_rows_ || std::abs(denom) < 1e-9)
    {
      return std::nullopt;
    }
    Line line;
    line.b = (n * sxy - sx * sy) / denom;
    line.a = (sx - line.b * sy) / n;
    return line;
  }

  // ==================== (2) APPROACH: 흰선 보고 직진 ====================

  void start_approach()
  {
    state_ = State::APPROACH;
    gap_ = Gap::BEFORE;
    gap_frames_ = 0;
    state_time_ = 0.0;
    last_image_time_ = now();
  }

  // 오른쪽 흰선에서 차선 폭 절반만큼 왼쪽 = 차선 가운데를 따라간다. 흰선이 없으면 직진
  void drive_along_white(const cv::Mat & white)
  {
    const int mid = static_cast<int>(bev_center_x_ * white.cols);
    const std::optional<Line> line = fit_line(white, mid, white.cols, white.rows);
    if (!line)
    {
      drive(approach_speed_, 0.0);
      return;
    }

    Line center = *line;
    center.a -= lane_width_px_ / 2.0;      // 흰선에서 차선 폭 절반만큼 왼쪽 = 차선 가운데
    drive(approach_speed_, steer_to(center, white.rows, white.cols));
  }

  // 차선 가운데 선(center)을 따라가는 회전 속도 [rad/s]. + 왼쪽, - 오른쪽
  //   방향 오차: 가운데 선이 왼쪽으로 기운 만큼 왼쪽으로 돈다
  //   위치 오차: 가운데 선을 바퀴 축 높이까지 이어서, 로봇보다 오른쪽에 얼마나 있는지
  double steer_to(const Line & center, int rows, int cols) const
  {
    const double robot_x = bev_center_x_ * cols;
    const double bottom_y = rows - 1;
    const double center_at_axle = center.x_at(bottom_y) + center.b * robot_y_offset_px_;
    const double offset_right = center_at_axle - robot_x;
    const double w = heading_gain_ * center.lean_left() -
      offset_gain_ * std::atan2(offset_right, offset_soft_px_);
    return std::clamp(w, -max_angular_, max_angular_);
  }

  // 왼쪽 아래 칸의 노란 점 개수로 "입구 반대쪽 모서리가 화면 맨 아래에 닿았는지" 판단
  //   BEFORE: 노란선 있음. 연속으로 비면 -> IN_GAP
  //   IN_GAP: 연속으로 다시 차면 -> 도착 (true)
  bool entrance_reached(const cv::Mat & yellow)
  {
    const int y1 = std::min(static_cast<int>(watch_bottom_) + 1, yellow.rows);
    const int y0 = std::max(0, y1 - static_cast<int>(watch_rows_));
    const int x1 = std::min(static_cast<int>(watch_x_max_), yellow.cols);
    const int count = cv::countNonZero(yellow(cv::Range(y0, y1), cv::Range(0, x1)));

    if (gap_ == Gap::BEFORE)
    {
      gap_frames_ = count <= empty_pixels_ ? gap_frames_ + 1 : 0;
      if (gap_frames_ >= gap_frames_needed_)
      {
        gap_ = Gap::IN_GAP;
        gap_frames_ = 0;
        RCLCPP_INFO(get_logger(), "입구의 빈 곳 지나는 중 (왼쪽 아래 노란 점 %d개)", count);
      }
      return false;
    }

    gap_frames_ = count >= present_pixels_ ? gap_frames_ + 1 : 0;
    if (gap_frames_ >= gap_frames_needed_)
    {
      RCLCPP_INFO(get_logger(), "입구 반대쪽 노란선이 화면 맨 아래에 닿음 (%d개) -> 왼쪽으로 회전", count);
      return true;
    }
    return false;
  }

  // ==================== (2-1) EXTRA: 입구 가운데까지 남은 만큼 더 직진 ====================

  // 반대쪽 노란선이 watch_bottom 줄에 닿았을 때, 바퀴 축 ~ 그 선 거리 = (화면 맨 아래 - watch_bottom) + robot_y_offset
  // 입구 가운데 ~ 그 선 거리 = 차선 폭 / 2. 그 차이가 더 가야 할 거리 (음수면 0)
  void start_extra()
  {
    const double to_line_px = (399.0 - watch_bottom_) + robot_y_offset_px_;
    const double extra_m = std::max(0.0, (to_line_px - lane_width_px_ / 2.0) * 0.001 + extra_forward_m_);
    extra_time_ = extra_m / approach_speed_;
    if (extra_time_ <= 0.0)
    {
      start_turn();
      return;
    }
    state_ = State::EXTRA;
    state_time_ = 0.0;
    RCLCPP_INFO(get_logger(), "입구 가운데까지 %.1fcm 더 직진 (%.1f초)", extra_m * 100.0, extra_time_);
  }

  // ==================== (3) TURN: 왼쪽으로 돌기 ====================

  void start_turn()
  {
    state_ = State::TURN;
    state_time_ = 0.0;
    drive(0.0, turn_speed_);
  }

  // 왼쪽/오른쪽 노란선이 둘 다 보이고, 간격이 길 폭 정도이고, 같은 쪽으로 기울어 있으면
  // (= 같은 길의 두 선) 기운 만큼 더 돈다. 둘 다 거의 세로(aligned_deg 이내)가 되면 멈춘다.
  // 그 전에는 turn_speed 로 계속 왼쪽으로
  void turn_until_aligned(const cv::Mat & yellow)
  {
    // 화면 아래쪽에는 입구의 둥근 모서리가 보여서 선이 바깥으로 휘어 보인다 -> 위쪽 줄만 쓴다
    const int mid = static_cast<int>(bev_center_x_ * yellow.cols);
    const int y_end = static_cast<int>(turn_fit_rows_);
    const std::optional<Line> left = fit_line(yellow, 0, mid, y_end);
    const std::optional<Line> right = fit_line(yellow, mid, yellow.cols, y_end);

    const double bottom_y = yellow.rows - 1;
    const bool pair = left && right &&
      right->x_at(bottom_y) - left->x_at(bottom_y) >= pair_gap_min_px_ &&
      right->x_at(bottom_y) - left->x_at(bottom_y) <= pair_gap_max_px_;

    if (!pair || state_time_ < turn_min_time_)
    {
      drive(0.0, turn_speed_);
      return;
    }

    const double aligned = aligned_deg_ * M_PI / 180.0;
    const double lean_l = left->lean_left();
    const double lean_r = right->lean_left();
    if (std::abs(lean_l - lean_r) > parallel_deg_ * M_PI / 180.0)
    {
      drive(0.0, turn_speed_);   // 두 선이 나란하지 않음 = 아직 길을 제대로 못 봄
      return;
    }
    const double lean = (lean_l + lean_r) / 2.0;   // + 면 더 왼쪽으로
    if (std::abs(lean_l) <= aligned && std::abs(lean_r) <= aligned)
    {
      start_stem(lean);
      return;
    }
    double w = turn_kp_ * lean;
    w = std::copysign(std::clamp(std::abs(w), turn_min_speed_, turn_speed_), lean);
    drive(0.0, w);
  }

  // ==================== (4) STEM: 두 노란선 가운데로 길 올라가기 ====================

  void start_stem(double lean)
  {
    state_ = State::STEM;
    state_time_ = 0.0;
    lines_missing_ = 0;
    RCLCPP_INFO(get_logger(), "입구 길을 똑바로 봄 (남은 기울기 %.1f도) -> 두 노란선 가운데로 올라감",
      lean * 180.0 / M_PI);
  }

  // 왼쪽/오른쪽 노란선을 잇고 그 가운데를 따라간다.
  //   둘 다 보이고 간격이 길 폭 정도 -> 두 선의 가운데
  //   하나만 보임                      -> 그 선에서 차선 폭 절반만큼 안쪽
  //   둘 다 안 보임이 연속              -> 길 끝 (주차장 입구) -> 멈춤
  void drive_between_yellow(const cv::Mat & yellow)
  {
    const int mid = static_cast<int>(bev_center_x_ * yellow.cols);
    std::optional<Line> left = fit_line(yellow, 0, mid, yellow.rows);
    std::optional<Line> right = fit_line(yellow, mid, yellow.cols, yellow.rows);

    // 두 선 간격이 길 폭과 너무 다르면 (잘못 찾은 선) 둘 다 믿지 않고 직진
    const double bottom_y = yellow.rows - 1;
    if (left && right)
    {
      const double gap = right->x_at(bottom_y) - left->x_at(bottom_y);
      if (gap < pair_gap_min_px_ || gap > pair_gap_max_px_)
      {
        left.reset();
        right.reset();
        drive(stem_speed_, 0.0);
        return;
      }
    }

    if (!left && !right)
    {
      lines_missing_++;
      if (lines_missing_ >= stem_end_frames_ && state_time_ >= stem_min_time_)
      {
        finish();
        return;
      }
      drive(stem_speed_, 0.0);     // 잠깐 안 보이면 직진
      return;
    }
    lines_missing_ = 0;

    Line center;
    if (left && right)
    {
      center.a = (left->a + right->a) / 2.0;
      center.b = (left->b + right->b) / 2.0;
    }
    else if (left)
    {
      center = *left;
      center.a += lane_width_px_ / 2.0;
    }
    else
    {
      center = *right;
      center.a -= lane_width_px_ / 2.0;
    }
    drive(stem_speed_, steer_to(center, yellow.rows, yellow.cols));
  }

  // ==================== 끝내기 ====================

  void finish()
  {
    stop_wheels();
    state_ = State::DONE;
    publish_mode(ControlMode::STOP, "stem_end");
    RCLCPP_INFO(get_logger(), "노란선이 끝남 (주차장 입구, %.1f초 올라감) -> 정지 (다음 단계는 아직 없음)",
      state_time_);
  }

  void give_up(const char * reason)
  {
    stop_wheels();
    state_ = State::DONE;
    RCLCPP_WARN(get_logger(), "주차 그만둠: %s", reason);
  }

  // ==================== 0.1초마다 ====================

  void watchdog()
  {
    if (!i_am_driving())
    {
      return;
    }

    // lane_follower 가 멈춰 있도록 "주차 중" 을 계속 알린다
    publish_mode(ControlMode::PARKING, state_name());

    state_time_ += 0.1;
    if (state_ == State::EXTRA && state_time_ >= extra_time_)
    {
      start_turn();
      return;
    }
    if (state_ == State::APPROACH && approach_timeout_ > 0.0 && state_time_ >= approach_timeout_)
    {
      give_up("입구를 못 찾음 (approach_timeout)");
      return;
    }
    if (state_ == State::TURN && turn_timeout_ > 0.0 && state_time_ >= turn_timeout_)
    {
      give_up("입구 길이 똑바로 안 보임 (turn_timeout)");
      return;
    }
    if (state_ == State::STEM && stem_timeout_ > 0.0 && state_time_ >= stem_timeout_)
    {
      give_up("길 끝을 못 찾음 (stem_timeout)");
      return;
    }

    // 영상이 끊기면 멈춤
    if ((now() - last_image_time_).seconds() > image_timeout_)
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상이 안 옴 -> 정지");
    }
  }

  // ==================== 도우미 ====================

  bool i_am_driving() const
  {
    return state_ == State::APPROACH || state_ == State::EXTRA || state_ == State::TURN ||
           state_ == State::STEM;
  }

  // GUI 에 보일 단계 이름
  const char * state_name() const
  {
    switch (state_)
    {
      case State::APPROACH:
        return "approach";
      case State::EXTRA:
        return "extra";
      case State::TURN:
        return "turn";
      case State::STEM:
        return "stem";
      default:
        return "-";
    }
  }

  // 바퀴 명령. 앞으로 갈 때 앞이 막혔으면 멈춤
  void drive(double v, double w)
  {
    if (v > 0.0 && front_closer_than(front_stop_distance_))
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  // 정면 PSD 가 distance [m] 보다 가까우면 true
  bool front_closer_than(double distance) const
  {
    if (distance <= 0.0 || !psd_received_)
    {
      return false;
    }
    return psd_.front > 0.0 && psd_.front <= distance;
  }

  void stop_wheels()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publish_mode(uint8_t mode, const std::string & detail)
  {
    ControlMode msg;
    msg.header.stamp = now();
    msg.mode = mode;
    msg.mission_state = std::string(MY_TAG) + ":" + detail;
    mode_pub_->publish(msg);
  }

  // ==================== 변수 ====================

  // 파라미터
  double min_confidence_;
  double sign_area_ratio_threshold_;
  int64_t sign_hits_needed_;
  bool skip_sign_;
  std::string image_topic_;
  int64_t yellow_l_min_;
  int64_t yellow_b_min_;
  int64_t yellow_a_max_;
  int64_t white_l_min_;
  int64_t white_ab_dev_;
  double bev_center_x_;
  double lane_width_px_;
  int64_t line_min_rows_;
  int64_t flat_min_pixels_;
  double approach_speed_;
  double heading_gain_;
  double offset_gain_;
  double offset_soft_px_;
  double max_angular_;
  double robot_y_offset_px_;
  int64_t watch_x_max_;
  int64_t watch_rows_;
  int64_t watch_bottom_;
  int64_t present_pixels_;
  int64_t empty_pixels_;
  int64_t gap_frames_needed_;
  double extra_forward_m_;
  double turn_speed_;
  double turn_kp_;
  double turn_min_speed_;
  double aligned_deg_;
  double parallel_deg_;
  double pair_gap_min_px_;
  double pair_gap_max_px_;
  double turn_min_time_;
  int64_t turn_fit_rows_;
  double stem_speed_;
  int64_t stem_end_frames_;
  double stem_min_time_;
  double stem_timeout_;
  double front_stop_distance_;
  double image_timeout_;
  double approach_timeout_;
  double turn_timeout_;

  // 상태
  State state_ = State::WAITING;
  Gap gap_ = Gap::BEFORE;
  int64_t gap_frames_ = 0;         // 지금 조건이 연속으로 맞은 프레임 수
  int64_t sign_hits_ = 0;
  double state_time_ = 0.0;        // 지금 단계에 들어온 뒤 지난 시간 [s]
  double extra_time_ = 0.0;        // EXTRA 단계에서 더 직진할 시간 [s]
  int64_t lines_missing_ = 0;      // STEM 에서 노란선이 둘 다 연속으로 안 보인 프레임 수
  rclcpp::Time last_image_time_;
  interfaces::msg::PsdArray psd_;
  bool psd_received_ = false;

  // ROS 연결
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}

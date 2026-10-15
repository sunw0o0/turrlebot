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
//   (4) STEM     : 주차장 길을 올라간다. 양쪽이 노란선 -> 두 노란선의 가운데로
//          │        노란선이 끝나면 같은 줄로 흰 점선(주차장 통로와 칸을 나누는 선)이 이어진다
//          │        -> 노란선과 흰선을 둘 다 차선으로 보고 그 가운데로 계속 직진 (stem_use_white)
//          │        한쪽만 보이면 그 선에서 차선 폭 절반만큼 떨어져서 간다
//          │  차선(노란선 / 흰 점선)이 둘 다 연속으로 안 보임 = 통로 끝 -> (5)
//          │
//          │  옆 PSD 가 side_psd_threshold (0.15m) 이하로 연속 side_hits_needed 번 = 그쪽에 장애물 (기억).
//          │  계속 직진하다가 앞의 가로 흰 선(주차장 끝 테두리)이 화면 맨 아래 근처(white_trigger_y)에 오면:
//          │    왼쪽 장애물  -> 왼쪽으로 side_turn_deg (40도) (SIDE_TURN)
//          │    오른쪽 장애물 -> 오른쪽으로 side_turn_deg (40도)
//          │    장애물을 못 봤으면 -> 그냥 멈춤 ("parking_node:stem_end")
//          │  side_turn_when 으로 도는 순간을 바꿀 수 있다:
//          │    white = 가로 흰 선이 화면 아래에 올 때 (기본), seen = PSD 로 보자마자,
//          │    clear = 장애물을 다 지나가서 그쪽 PSD 가 side_clear_threshold 보다 멀어질 때
//          │    (side_turn_toward: false 면 반대쪽으로)
//          │  (각도는 엔코더 odom 으로 실제 돈 각도를 재서 멈춘다. odom 이 안 오면 "회전 속도 x 시간")
//          │  돌고 나면 멈추고 STOP ("parking_node:turned_right" / "turned_left")
//          ▼
//   (5) DONE     : 멈추고 control_mode = STOP ("parking_node:stem_end")
//                  (다음 단계 - 빈 칸 고르기, 주차 - 는 아직 없음)
//          │  GUI 에서 LANE 을 누르면 (1) WAITING 으로 돌아가 다음 바퀴에 다시 주차한다
//          │  (skip_sign 이면 바로 (2) APPROACH 부터)
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
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
    STEM,        // 주차장 길 -> 통로를 두 차선(노란선 / 흰 점선) 가운데로 올라가는 중 (이 노드가 운전)
    SIDE_TURN,   // 옆 PSD 가 가까워서 반대쪽으로 90도 도는 중 (이 노드가 운전)
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
        check_side_psd();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
      {
        on_odom(*msg);
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg)
      {
        on_mode(*msg);
      });

    // ---- lane_follower 의 linear_speed 를 물어볼 때 쓴다 ----
    lane_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "lane_follower");

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
    // 직진 속도 [m/s]. use_lane_speed 가 true 면 주차를 시작할 때 lane_follower 의 linear_speed
    // (GUI 선속도 슬라이더 값) 를 받아와서 그 속도로 간다. 못 받으면 drive_speed 를 쓴다
    drive_speed_ = declare_parameter("drive_speed", 0.10);
    use_lane_speed_ = declare_parameter("use_lane_speed", true);
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
    extra_forward_m_ = declare_parameter("extra_forward_m", -0.04);

    // 5) 회전
    turn_speed_ = declare_parameter("turn_speed", 0.4);
    turn_min_speed_ = declare_parameter("turn_min_speed", 0.08);
    turn_slow_angle_deg_ = declare_parameter("turn_slow_angle_deg", 60.0);
    turn_slow_power_ = declare_parameter("turn_slow_power", 2.0);
    aligned_deg_ = declare_parameter("aligned_deg", 1.5);
    parallel_deg_ = declare_parameter("parallel_deg", 8.0);
    pair_gap_min_px_ = declare_parameter("pair_gap_min_px", 200.0);
    pair_gap_max_px_ = declare_parameter("pair_gap_max_px", 380.0);
    turn_min_time_ = declare_parameter("turn_min_time", 1.0);
    turn_fit_rows_ = declare_parameter("turn_fit_rows", 230);

    // 6) 주차장 길 올라가기
    stem_use_white_ = declare_parameter("stem_use_white", true);
    stem_end_frames_ = declare_parameter("stem_end_frames", 3);
    stem_min_time_ = declare_parameter("stem_min_time", 2.0);
    stem_timeout_ = declare_parameter("stem_timeout", 20.0);

    // 7) 옆 PSD 가 가까우면 반대쪽으로 90도
    side_psd_threshold_ = declare_parameter("side_psd_threshold", 0.15);
    side_hits_needed_ = declare_parameter("side_hits_needed", 3);
    side_turn_deg_ = declare_parameter("side_turn_deg", 40.0);
    side_turn_toward_ = declare_parameter("side_turn_toward", true);
    side_turn_when_ = declare_parameter("side_turn_when", std::string("white"));
    white_trigger_y_ = declare_parameter("white_trigger_y", 350);
    side_cam_ = declare_parameter("side_turn_by_camera", true);
    side_cam_min_area_ = declare_parameter("side_cam_min_area", 800);
    side_cam_lost_frames_ = declare_parameter("side_cam_lost_frames", 5);
    side_cam_max_extra_deg_ = declare_parameter("side_cam_max_extra_deg", 15.0);
    white_row_min_pixels_ = declare_parameter("white_row_min_pixels", 150);
    white_center_ratio_ = declare_parameter("white_center_ratio", 0.6);
    white_arm_y_ = declare_parameter("white_arm_y", 300);
    side_clear_threshold_ = declare_parameter("side_clear_threshold", 0.20);
    side_turn_speed_ = declare_parameter("side_turn_speed", 0.6);
    side_min_speed_ = declare_parameter("side_min_speed", 0.08);
    side_slow_angle_deg_ = declare_parameter("side_slow_angle_deg", 30.0);
    side_slow_power_ = declare_parameter("side_slow_power", 2.0);
    side_tolerance_deg_ = declare_parameter("side_tolerance_deg", 1.0);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);

    // 안전
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);
    image_timeout_ = declare_parameter("image_timeout", 0.5);
    approach_timeout_ = declare_parameter("approach_timeout", 30.0);
    turn_timeout_ = declare_parameter("turn_timeout", 15.0);
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

  // GUI 버튼
  //   STOP / MANUAL : 주차 중이면 멈추고 그만둔다
  //   LANE          : 주차가 끝났거나 그만둔 상태면 처음(표지판 기다리기)으로 돌아간다
  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state != "turtle_gui")
    {
      return;                    // GUI 가 보낸 것만 본다 (내가 보낸 것 등은 무시)
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    if (stop_request && i_am_driving())
    {
      give_up("GUI 에서 STOP/MANUAL");
      return;
    }
    if (msg.mode == ControlMode::LANE && state_ == State::DONE)
    {
      reset();
    }
  }

  // 처음 상태로 돌아가기 (다음 바퀴에 다시 주차)
  void reset()
  {
    sign_hits_ = 0;
    if (skip_sign_)
    {
      RCLCPP_INFO(get_logger(), "GUI LANE -> 다시 시작: skip_sign 이라 바로 입구 찾기");
      start_approach();
      return;
    }
    state_ = State::WAITING;
    RCLCPP_INFO(get_logger(), "GUI LANE -> 다시 시작: 주차 표지판 기다림");
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
    else if (state_ == State::SIDE_TURN)
    {
      if (side_use_cam_)
      {
        side_turn_by_camera(white_mask(lab));
      }
    }
    else if (state_ == State::STEM)
    {
      const cv::Mat white = white_mask(lab);
      if (side_turn_when_ == "white" && check_white_trigger(white))
      {
        return;                  // 회전 시작 (또는 정지)
      }
      // 노란선 + (흰 점선). 가로로 누운 흰선(주차장 테두리)은 선을 이을 때 빠진다
      cv::Mat lines = yellow;
      if (stem_use_white_)
      {
        cv::bitwise_or(yellow, white, lines);
      }
      drive_between_lines(lines);
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
    choose_speed();
    state_ = State::APPROACH;
    gap_ = Gap::BEFORE;
    gap_frames_ = 0;
    state_time_ = 0.0;
    last_image_time_ = now();
  }

  // 직진 속도 정하기: 우선 drive_speed 로 시작하고, lane_follower 가 답하면 그 linear_speed 로 바꾼다
  void choose_speed()
  {
    speed_ = drive_speed_;
    if (!use_lane_speed_ || !lane_params_->service_is_ready())
    {
      RCLCPP_INFO(get_logger(), "주차 직진 속도 %.2f m/s (drive_speed)", speed_);
      return;
    }
    lane_params_->get_parameters({"linear_speed"},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future)
      {
        const auto params = future.get();
        if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
          params[0].as_double() > 0.0)
        {
          speed_ = params[0].as_double();
        }
        RCLCPP_INFO(get_logger(), "주차 직진 속도 %.2f m/s (lane_follower 와 같게)", speed_);
      });
  }

  // 오른쪽 흰선에서 차선 폭 절반만큼 왼쪽 = 차선 가운데를 따라간다. 흰선이 없으면 직진
  void drive_along_white(const cv::Mat & white)
  {
    const int mid = static_cast<int>(bev_center_x_ * white.cols);
    const std::optional<Line> line = fit_line(white, mid, white.cols, white.rows);
    if (!line)
    {
      drive(speed_, 0.0);
      return;
    }

    Line center = *line;
    center.a -= lane_width_px_ / 2.0;      // 흰선에서 차선 폭 절반만큼 왼쪽 = 차선 가운데
    drive(speed_, steer_to(center, white.rows, white.cols));
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
    extra_time_ = extra_m / speed_;
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

  // 회전 속도: 길의 노란선이 왼쪽으로 많이 기울어 있으면(아직 많이 돌아야 함) 빠르게(turn_speed),
  //   세로에 가까워질수록 느리게(turn_min_speed). turn_slow_angle_deg 보다 많이 기울면 최고 속도
  //   ratio = 기운 각도 / turn_slow_angle_deg (0 ~ 1) 를 turn_slow_power 제곱해서 쓴다
  //     power 1 = 각도에 비례해서 고르게 느려짐, 2 = 처음엔 조금, 세로에 가까울수록 훨씬 많이 느려짐
  //   lean: 선이 왼쪽으로 기운 각도 [rad]. 음수(오른쪽으로 기움 = 지나침)도 크기로 계산
  double turn_speed_for(double lean) const
  {
    const double ratio = std::min(1.0, std::abs(lean) / (turn_slow_angle_deg_ * M_PI / 180.0));
    return turn_min_speed_ + (turn_speed_ - turn_min_speed_) * std::pow(ratio, turn_slow_power_);
  }

  // 왼쪽으로 돌면서:
  //   - 처음 turn_min_time 초는 최고 속도 (막 돌기 시작할 때 보이는 도로 노란선은 세로라서 헷갈리므로)
  //   - 그 뒤 왼쪽으로 기운 노란선(= 길의 선, 도는 쪽에서 들어옴)이 보이면 그 기울기에 맞춰 점점 느리게
  //   - 왼쪽/오른쪽 노란선이 둘 다 보이고, 간격이 길 폭 정도이고, 같은 쪽으로 기울어 있으면
  //     (= 같은 길의 두 선) 둘 다 거의 세로(aligned_deg 이내)가 되면 멈춘다
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

    if (state_time_ < turn_min_time_)
    {
      drive(0.0, turn_speed_);
      return;
    }
    if (!pair)
    {
      drive(0.0, turn_speed_for(leaning_left(left, right)));
      return;
    }

    const double aligned = aligned_deg_ * M_PI / 180.0;
    const double lean_l = left->lean_left();
    const double lean_r = right->lean_left();
    if (std::abs(lean_l - lean_r) > parallel_deg_ * M_PI / 180.0)
    {
      // 두 선이 나란하지 않음 = 아직 길을 제대로 못 봄
      drive(0.0, turn_speed_for(leaning_left(left, right)));
      return;
    }
    const double lean = (lean_l + lean_r) / 2.0;   // + 면 더 왼쪽으로
    if (std::abs(lean_l) <= aligned && std::abs(lean_r) <= aligned)
    {
      start_stem(lean);
      return;
    }
    drive(0.0, std::copysign(turn_speed_for(lean), lean));   // 지나쳤으면(lean < 0) 오른쪽으로
  }

  // 보이는 노란선 중 왼쪽으로 기운 것(= 아직 더 돌아야 하는 길의 선)의 기울기 [rad].
  // 왼쪽으로 기운 선이 없으면 아주 큰 값 (= 최고 속도)
  static double leaning_left(const std::optional<Line> & left, const std::optional<Line> & right)
  {
    double lean = 10.0;
    for (const auto & line : {left, right})
    {
      if (line && line->lean_left() > 0.0)
      {
        lean = std::min(lean, line->lean_left());
      }
    }
    return lean;
  }

  // ==================== (4) STEM: 두 차선 가운데로 길 / 통로 올라가기 ====================

  void start_stem(double lean)
  {
    state_ = State::STEM;
    state_time_ = 0.0;
    lines_missing_ = 0;
    side_seen_ = 0.0;
    left_hits_ = 0;
    right_hits_ = 0;
    clear_hits_ = 0;
    last_white_y_ = -1.0;
    last_white_tilt_.reset();
    side_cam_tilt0_.reset();
    RCLCPP_INFO(get_logger(), "입구 길을 똑바로 봄 (남은 기울기 %.1f도) -> 두 차선 가운데로 올라감",
      lean * 180.0 / M_PI);
  }

  // 왼쪽/오른쪽 차선(노란선, 흰 점선)을 잇고 그 가운데를 따라간다.
  //   둘 다 보이고 간격이 길 폭 정도 -> 두 선의 가운데
  //   하나만 보임                      -> 그 선에서 차선 폭 절반만큼 안쪽
  //   둘 다 안 보임이 연속              -> 통로 끝 -> 멈춤
  void drive_between_lines(const cv::Mat & lines)
  {
    const int mid = static_cast<int>(bev_center_x_ * lines.cols);
    std::optional<Line> left = fit_line(lines, 0, mid, lines.rows);
    std::optional<Line> right = fit_line(lines, mid, lines.cols, lines.rows);

    // 두 선 간격이 길 폭과 너무 다르면 (잘못 찾은 선) 둘 다 믿지 않고 직진
    const double bottom_y = lines.rows - 1;
    if (left && right)
    {
      const double gap = right->x_at(bottom_y) - left->x_at(bottom_y);
      if (gap < pair_gap_min_px_ || gap > pair_gap_max_px_)
      {
        left.reset();
        right.reset();
        drive(speed_, 0.0);
        return;
      }
    }

    if (!left && !right)
    {
      lines_missing_++;
      // side_turn_when: white 면 차선이 끝나도 멈추지 않고, 가로 흰 선이 화면 아래에 올 때까지 직진
      //   (그 흰 선을 보고 돌거나 멈춘다. 끝내 안 보이면 stem_timeout 에서 멈춤)
      const bool wait_white = side_turn_when_ == "white";
      if (!wait_white && lines_missing_ >= stem_end_frames_ && state_time_ >= stem_min_time_)
      {
        finish();
        return;
      }
      drive(speed_, 0.0);     // 안 보이면 직진
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
    drive(speed_, steer_to(center, lines.rows, lines.cols));
  }

  // ==================== 옆 PSD 로 90도 돌기 ====================

  // STEM 중 PSD 가 올 때마다
  //   (1) 옆 PSD 가 side_psd_threshold 이하로 연속 side_hits_needed 번 -> 그쪽에 장애물 있음 (기억만, 계속 직진)
  //   (2) 그 뒤 그쪽 PSD 가 side_clear_threshold 보다 멀어진 게 연속 side_hits_needed 번
  //       -> 장애물을 다 지나감 -> 그쪽으로 (side_turn_toward: false 면 반대쪽으로) 돌기 시작
  //   side_turn_when: seen 이면 (1) 에서 바로 돈다, white 면 (1) 에서 기억만 하고
  //   가로 흰 선이 화면 아래에 올 때 돈다 (on_image -> check_white_trigger)
  void check_side_psd()
  {
    if (state_ != State::STEM)
    {
      left_hits_ = 0;
      right_hits_ = 0;
      return;
    }

    // (1) 아직 장애물을 못 봤으면: 가까운 쪽 찾기
    if (side_seen_ == 0.0)
    {
      left_hits_ = is_close(psd_.left) ? left_hits_ + 1 : 0;
      right_hits_ = is_close(psd_.right) ? right_hits_ + 1 : 0;
      if (left_hits_ >= side_hits_needed_)
      {
        side_seen_ = +1.0;
        RCLCPP_INFO(get_logger(), "왼쪽 PSD %.2fm (<= %.2f) -> 왼쪽에 장애물", psd_.left, side_psd_threshold_);
      }
      else if (right_hits_ >= side_hits_needed_)
      {
        side_seen_ = -1.0;
        RCLCPP_INFO(get_logger(), "오른쪽 PSD %.2fm (<= %.2f) -> 오른쪽에 장애물", psd_.right,
          side_psd_threshold_);
      }
      if (side_seen_ == 0.0)
      {
        return;
      }
      clear_hits_ = 0;
      if (side_turn_when_ == "seen")
      {
        turn_toward_seen_side();
      }
      else if (side_turn_when_ == "clear")
      {
        RCLCPP_INFO(get_logger(), "장애물 옆을 지나갈 때까지 계속 직진");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "기억함. 앞의 가로 흰 선이 화면 아래에 올 때까지 계속 직진");
      }
      return;
    }

    // (2) side_turn_when: clear 일 때만 - 그쪽 PSD 가 멀어질 때까지 기다리기
    if (side_turn_when_ != "clear")
    {
      return;
    }
    const double value = side_seen_ > 0.0 ? psd_.left : psd_.right;
    clear_hits_ = (value > side_clear_threshold_) ? clear_hits_ + 1 : 0;
    if (clear_hits_ >= side_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "%s PSD %.2fm (> %.2f) -> 장애물을 지나감", side_seen_ > 0.0 ? "왼쪽" : "오른쪽",
        value, side_clear_threshold_);
      turn_toward_seen_side();
    }
  }

  // PSD 값이 정상(0 보다 큼)이고 side_psd_threshold 이하인지
  bool is_close(double value) const
  {
    return value > 0.0 && value <= side_psd_threshold_;
  }

  // 장애물을 본 쪽으로 (toward = false 면 반대쪽으로) 돌기 시작
  void turn_toward_seen_side()
  {
    const double toward = side_turn_toward_ ? 1.0 : -1.0;
    start_side_turn(side_seen_ * toward);
  }

  // 앞의 가로 흰 선(주차장 끝 테두리)이 화면에서 사라지기 직전인지 보고, 그렇다면
  //   장애물을 본 쪽이 있으면 그쪽으로 돌기 시작, 없으면 멈춤. 무언가 했으면 true
  //   "사라지기 직전" = (1) 흰 선이 white_trigger_y 줄보다 아래에 보임, 또는
  //                     (2) 바로 전 영상에서 white_arm_y 줄보다 아래에 있던 흰 선이 이번 영상에서 안 보임
  //   (카메라가 초당 몇 장이라, 흰 선이 맨 아래 몇 줄을 지나가는 걸 못 찍고 지나칠 수 있어서 (2) 도 본다)
  bool check_white_trigger(const cv::Mat & white)
  {
    const std::optional<double> line_y = lowest_horizontal_white(white);
    bool fire = false;
    double fire_y = 0.0;
    if (line_y)
    {
      last_white_y_ = *line_y;
      last_white_tilt_ = white_line_tilt(white);
      fire = *line_y >= white_trigger_y_;
      fire_y = *line_y;
    }
    else if (last_white_y_ >= white_arm_y_)
    {
      fire = true;                 // 바로 전에 화면 아래쪽에 있던 흰 선이 이제 안 보임 = 막 지나감
      fire_y = last_white_y_;
    }
    else
    {
      last_white_y_ = -1.0;
    }
    if (!fire)
    {
      return false;
    }

    if (side_seen_ != 0.0)
    {
      RCLCPP_INFO(get_logger(), "가로 흰 선이 화면에서 사라지기 직전 (y=%.0f) -> 기억한 %s으로 회전", fire_y,
        side_seen_ > 0.0 ? "왼쪽" : "오른쪽");
      side_cam_tilt0_ = last_white_tilt_;   // 돌기 전 흰 선 기울기 (거의 0 = 가로)
      turn_toward_seen_side();
    }
    else
    {
      RCLCPP_INFO(get_logger(), "가로 흰 선이 화면에서 사라지기 직전 (y=%.0f), 옆 장애물은 못 봄 -> 정지", fire_y);
      finish();
    }
    return true;
  }

  // 화면을 가로지르는 흰 선 중 가장 아래(= 로봇에 가장 가까운) 것의 줄 번호. 없으면 비어 있음
  //   한 줄에 흰 점이 white_row_min_pixels 이상이고, 화면 가운데 40% 구간도 white_center_ratio 이상 채워야 한다
  //   (주차장 입구 쪽 테두리는 길이 들어오는 가운데가 뚫려 있어서 빠진다)
  std::optional<double> lowest_horizontal_white(const cv::Mat & white) const
  {
    const int c0 = static_cast<int>(white.cols * 0.3);
    const int c1 = static_cast<int>(white.cols * 0.7);
    const double center_need = (c1 - c0) * white_center_ratio_;
    for (int y = white.rows - 1; y >= 0; y--)
    {
      const int all = cv::countNonZero(white.row(y));
      if (all < white_row_min_pixels_)
      {
        continue;
      }
      if (cv::countNonZero(white.row(y).colRange(c0, c1)) >= center_need)
      {
        return static_cast<double>(y);
      }
    }
    return std::nullopt;
  }

  // direction: +1 왼쪽, -1 오른쪽
  //   odom 이 최근에 왔으면: odom 으로 실제 돈 각도를 재며 돈다 (정확)
  //   아니면:               회전 시간 = 각도 / 회전 속도 만큼 돈다
  void start_side_turn(double direction)
  {
    stop_wheels();
    state_ = State::SIDE_TURN;
    state_time_ = 0.0;
    side_direction_ = direction;
    side_turn_time_ = (side_turn_deg_ * M_PI / 180.0) / side_turn_speed_;          // odom 도 카메라도 없을 때
    side_turn_limit_ = (side_turn_deg_ * M_PI / 180.0) / side_min_speed_ * 1.5;    // 너무 오래 걸리면 포기
    left_hits_ = 0;
    right_hits_ = 0;
    side_use_odom_ = odom_received_ && (now() - last_odom_time_).seconds() < odom_timeout_;
    side_start_yaw_ = odom_yaw_;
    side_turned_ = 0.0;
    // 가로 흰 선을 보고 돌기 시작했으면: 그 흰 선이 기운 각도로 돈 각도를 잰다 (카메라)
    side_use_cam_ = side_cam_ && side_cam_tilt0_.has_value();
    side_cam_lost_ = 0;
    side_cam_turned_ = 0.0;
    const char * how = side_use_cam_ ? "카메라 (가로 흰 선 기울기) 로 각도 잼" :
      (side_use_odom_ ? "odom 으로 각도 잼" : "odom 없음 -> 시간으로");
    RCLCPP_INFO(get_logger(), "%s으로 %.0f도 회전 시작 (%s)", direction < 0.0 ? "오른쪽" : "왼쪽",
      side_turn_deg_, how);
  }

  // 영상이 올 때마다 (SIDE_TURN + 카메라 사용 중): 가로였던 흰 선이 기운 각도 = 돈 각도
  //   목표 각도까지 남은 만큼 천천히 돌고, 다 돌면 멈춤.
  //   흰 선을 연속 side_cam_lost_frames 번 못 찾으면 odom 으로 넘어간다 (odom 도 없으면 시간)
  void side_turn_by_camera(const cv::Mat & white)
  {
    const std::optional<double> tilt = white_line_tilt(white);
    if (!tilt)
    {
      side_cam_lost_++;
      if (side_cam_lost_ >= side_cam_lost_frames_)
      {
        side_use_cam_ = false;
        RCLCPP_WARN(get_logger(), "회전 중 흰 선을 놓침 (카메라로 %.1f도) -> %s 으로 이어서 잼",
          side_cam_turned_ * 180.0 / M_PI, side_use_odom_ ? "odom" : "시간");
      }
      return;                    // 바퀴는 마지막 명령대로 계속 돈다
    }
    side_cam_lost_ = 0;
    side_cam_turned_ = std::abs(*tilt - *side_cam_tilt0_);
    const double remain = side_turn_deg_ * M_PI / 180.0 - side_cam_turned_;
    if (remain <= side_tolerance_deg_ * M_PI / 180.0)
    {
      end_side_turn();
      return;
    }
    const double w = side_speed_for(remain);
    drive(0.0, side_direction_ * w);
  }

  // 옆 회전 속도: 남은 각도가 side_slow_angle_deg 보다 크면 최고 속도(side_turn_speed),
  //   가까워질수록 (남은 각도 / side_slow_angle_deg) 의 side_slow_power 제곱만큼 느려져 side_min_speed 까지
  double side_speed_for(double remain) const
  {
    const double ratio = std::clamp(remain / (side_slow_angle_deg_ * M_PI / 180.0), 0.0, 1.0);
    return side_min_speed_ + (side_turn_speed_ - side_min_speed_) * std::pow(ratio, side_slow_power_);
  }

  // 흰색 덩어리 중 가장 큰 것(= 주차장 끝 테두리)의 기울기 [rad]. 0 = 가로, pi/2 = 세로
  //   덩어리가 side_cam_min_area 보다 작으면 (흰 선이 거의 안 보임) 비어 있음
  std::optional<double> white_line_tilt(const cv::Mat & white) const
  {
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(white, labels, stats, centroids, 8, CV_32S);
    int best = 0;
    int best_area = 0;
    for (int i = 1; i < n; i++)                // 0 은 배경
    {
      const int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area > best_area)
      {
        best = i;
        best_area = area;
      }
    }
    if (best == 0 || best_area < side_cam_min_area_)
    {
      return std::nullopt;
    }
    std::vector<cv::Point> points;
    cv::findNonZero(labels == best, points);
    cv::Vec4f line;                            // (방향 x, 방향 y, 지나는 점 x, y)
    cv::fitLine(points, line, cv::DIST_HUBER, 0, 0.01, 0.01);
    return std::atan2(std::abs(line[1]), std::abs(line[0]));
  }

  // odom 이 올 때마다 (SIDE_TURN + odom 사용 중일 때): 돈 각도를 재고, 남은 각도에 맞춰 속도 조절
  void step_side_turn_odom()
  {
    side_turned_ = std::abs(wrap_angle(odom_yaw_ - side_start_yaw_));
    const double remain = side_turn_deg_ * M_PI / 180.0 - side_turned_;
    if (remain <= side_tolerance_deg_ * M_PI / 180.0)
    {
      end_side_turn();
      return;
    }
    // 남은 각도에 비례해서 느려진다 (지나치지 않게). 너무 느리면 안 돌아서 최소 속도
    const double w = side_speed_for(remain);
    drive(0.0, side_direction_ * w);
  }

  // 0.1초마다: odom 을 안 쓰면 시간으로 돌기. odom 을 써도 너무 오래 걸리면 (odom 끊김 등) 멈춤
  void step_side_turn()
  {
    if (side_use_cam_)
    {
      if (state_time_ >= side_turn_limit_)
      {
        stop_wheels();
        give_up("카메라로 회전 중 너무 오래 걸림");
      }
      return;                    // 바퀴 명령은 영상이 올 때마다 side_turn_by_camera() 가 보낸다
    }
    if (side_use_odom_)
    {
      if ((now() - last_odom_time_).seconds() > odom_timeout_ || state_time_ >= side_turn_limit_)
      {
        stop_wheels();
        give_up("회전 중 odom 이 끊김 / 너무 오래 걸림");
      }
      return;                    // 바퀴 명령은 odom 이 올 때마다 step_side_turn_odom() 이 보낸다
    }
    if (state_time_ >= side_turn_time_)
    {
      end_side_turn();
      return;
    }
    drive(0.0, side_direction_ * side_turn_speed_);
  }

  void end_side_turn()
  {
    stop_wheels();
    state_ = State::DONE;
    const bool right = side_direction_ < 0.0;
    publish_mode(ControlMode::STOP, right ? "turned_right" : "turned_left");
    if (side_use_cam_)
    {
      RCLCPP_INFO(get_logger(), "%s으로 %.1f도 돌았음 (카메라, odom %.1f도) -> 정지 (다음 단계는 아직 없음)",
        right ? "오른쪽" : "왼쪽", side_cam_turned_ * 180.0 / M_PI, side_turned_ * 180.0 / M_PI);
    }
    else if (side_use_odom_)
    {
      RCLCPP_INFO(get_logger(), "%s으로 %.1f도 돌았음 (odom) -> 정지 (다음 단계는 아직 없음)",
        right ? "오른쪽" : "왼쪽", side_turned_ * 180.0 / M_PI);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "%s으로 %.0f도 돌았음 (시간) -> 정지 (다음 단계는 아직 없음)",
        right ? "오른쪽" : "왼쪽", side_turn_deg_);
    }
  }

  // odom: 로봇 방향(yaw) 저장
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_received_ = true;
    last_odom_time_ = now();
    if (state_ != State::SIDE_TURN || !side_use_odom_)
    {
      return;
    }
    if (!side_use_cam_)
    {
      step_side_turn_odom();
      return;
    }
    // 카메라로 도는 중: odom 으로 잰 각도가 목표 + side_cam_max_extra_deg 를 넘으면 무조건 멈춤 (안전)
    side_turned_ = std::abs(wrap_angle(odom_yaw_ - side_start_yaw_));
    if (side_turned_ >= (side_turn_deg_ + side_cam_max_extra_deg_) * M_PI / 180.0)
    {
      RCLCPP_WARN(get_logger(), "카메라 각도 %.1f도인데 odom 으로 %.1f도 -> 안전하게 멈춤",
        side_cam_turned_ * 180.0 / M_PI, side_turned_ * 180.0 / M_PI);
      end_side_turn();
    }
  }

  // 각도를 -pi ~ pi 사이로
  static double wrap_angle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  // ==================== 끝내기 ====================

  void finish()
  {
    stop_wheels();
    state_ = State::DONE;
    publish_mode(ControlMode::STOP, "stem_end");
    RCLCPP_INFO(get_logger(), "차선이 끝남 (통로 끝, %.1f초 올라감) -> 정지 (다음 단계는 아직 없음)",
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
    if (state_ == State::SIDE_TURN)
    {
      step_side_turn();          // 시간으로 도는 거라 영상이 없어도 계속
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
           state_ == State::STEM || state_ == State::SIDE_TURN;
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
      case State::SIDE_TURN:
        return side_direction_ < 0.0 ? "side_turn_right" : "side_turn_left";
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
  double drive_speed_;
  bool use_lane_speed_;
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
  double turn_min_speed_;
  double turn_slow_angle_deg_;
  double turn_slow_power_;
  double aligned_deg_;
  double parallel_deg_;
  double pair_gap_min_px_;
  double pair_gap_max_px_;
  double turn_min_time_;
  int64_t turn_fit_rows_;
  bool stem_use_white_;
  int64_t stem_end_frames_;
  double stem_min_time_;
  double stem_timeout_;
  double side_psd_threshold_;
  int64_t side_hits_needed_;
  double side_turn_deg_;
  bool side_turn_toward_;
  std::string side_turn_when_;
  bool side_cam_;
  int64_t side_cam_min_area_;
  int64_t side_cam_lost_frames_;
  double side_cam_max_extra_deg_;
  int64_t white_trigger_y_;
  int64_t white_row_min_pixels_;
  double white_center_ratio_;
  int64_t white_arm_y_;
  double side_clear_threshold_;
  double side_turn_speed_;
  double side_slow_angle_deg_;
  double side_slow_power_;
  double side_min_speed_;
  double side_tolerance_deg_;
  double odom_timeout_;
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
  double speed_ = 0.10;            // 지금 쓰는 직진 속도 [m/s] (주차 시작할 때 정함)
  int64_t lines_missing_ = 0;      // STEM 에서 차선이 둘 다 연속으로 안 보인 프레임 수
  int64_t left_hits_ = 0;          // 왼쪽 PSD 가 연속으로 가까웠던 횟수
  int64_t right_hits_ = 0;         // 오른쪽 PSD 가 연속으로 가까웠던 횟수
  double side_seen_ = 0.0;         // 장애물을 본 쪽 (+1 왼쪽, -1 오른쪽, 0 아직 못 봄)
  int64_t clear_hits_ = 0;         // 장애물을 본 쪽 PSD 가 연속으로 멀었던 횟수
  double last_white_y_ = -1.0;     // 바로 전 영상에서 가로 흰 선의 줄 번호 (-1 = 없음)
  std::optional<double> last_white_tilt_;  // 바로 전 영상에서 흰 선 기울기 [rad]
  double side_direction_ = 0.0;    // SIDE_TURN 방향 (+1 왼쪽, -1 오른쪽)
  double side_turn_time_ = 0.0;    // SIDE_TURN 할 시간 [s] (odom 없을 때)
  double side_turn_limit_ = 0.0;   // SIDE_TURN 최대 시간 [s] (넘으면 포기)
  bool side_use_odom_ = false;     // SIDE_TURN 을 odom 으로 재는지
  bool side_use_cam_ = false;      // SIDE_TURN 을 카메라 (가로 흰 선 기울기) 로 재는지
  std::optional<double> side_cam_tilt0_;   // 돌기 시작할 때 흰 선 기울기 [rad]
  int64_t side_cam_lost_ = 0;      // 흰 선을 연속으로 못 찾은 프레임 수
  double side_cam_turned_ = 0.0;   // 카메라로 잰 돈 각도 [rad]
  double side_start_yaw_ = 0.0;    // SIDE_TURN 시작할 때 방향 [rad]
  double side_turned_ = 0.0;       // 지금까지 돈 각도 [rad]
  double odom_yaw_ = 0.0;          // odom 방향 [rad]
  bool odom_received_ = false;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_image_time_;
  interfaces::msg::PsdArray psd_;
  bool psd_received_ = false;

  // ROS 연결
  rclcpp::Subscription<Sign>::SharedPtr sign_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<interfaces::msg::PsdArray>::SharedPtr psd_sub_;
  rclcpp::Subscription<ControlMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ControlMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::AsyncParametersClient::SharedPtr lane_params_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}

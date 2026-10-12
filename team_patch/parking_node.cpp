// ============================================================================
//  parking_node.cpp - 주차 구간 담당 (TB3 AutoRace 맵 기준)
// ============================================================================
//
// [트랙 모양] (위에서 본 지도, 로봇은 아래 도로를 왼쪽 -> 오른쪽(동쪽)으로 달린다)
//
//              ┌──────┬──────┬──────┐
//              │ 왼쪽 │ 통로 │오른쪽│   <- 주차장. 왼쪽/오른쪽 칸 중 한 곳에 다른 로봇이 서 있다
//              │  칸  │      │  칸  │
//              └──────┴┐    ┌┴──────┘
//                      │    │           <- 주차장으로 올라가는 길 (양옆이 노란선)
//      노란선 ─────────┘    └────────── 노란선
//      ─────►  로봇 진행 방향 (동쪽)
//      흰선   ──────────────────────────
//
//   아래 도로의 왼쪽 노란선은 주차장 길 입구에서 왼쪽(위)으로 꺾인다.
//   카메라(위에서 본 영상 BEV)로 보면 이 꺾인 부분이 "가로로 누운 노란선" 으로 보인다.
//   -> 가로 노란선이 하나라도 보이면, 오른쪽에 흰선이 있어도 무시하고 노란선을 따라 왼쪽으로 들어간다.
//
// [전체 흐름]
//
//   (1) WAITING   : 주차 표지판을 기다린다
//          │  주차 표지판이 보임
//          ▼
//   (2) APPROACH  : 운전은 lane_follower 가 계속 한다 (평소처럼 차선 따라가기)
//          │        이 노드는 카메라 영상 왼쪽에서 "가로 노란선" 을 찾는다
//          │  가로 노란선이 보임 -> 길 가운데까지 남은 거리를 계산
//          ▼
//   (3) 여기부터 이 노드가 직접 운전한다 (lane_follower 는 멈춤)
//          │   ① align      : 길 가운데가 바퀴 축 옆에 올 때까지 직진
//          │   ② turn_stem  : 왼쪽으로 90도 (주차장 쪽을 봄)
//          │   ③ follow     : 왼쪽 노란선을 따라 길을 올라간다 (오른쪽은 안 봄)
//          │                  노란선이 끝나면 = 주차장 입구
//          │   ④ to_lot     : 주차장 통로 가운데까지 직진
//          │   ⑤ check      : 멈춰서 왼쪽/오른쪽 PSD 로 어느 칸에 로봇이 있는지 본다
//          │                  왼쪽이 가까움 -> 오른쪽 칸에 주차 / 오른쪽이 가까움 -> 왼쪽 칸
//          │   ⑥ turn_in    : 빈 칸 쪽으로 90도
//          │   ⑦ enter      : 칸 안으로 직진
//          │   ⑧ wait       : 서 있기 (주차 완료)
//          │   ⑨ back_out   : 들어간 만큼 후진 (통로 가운데로)
//          │   ⑩ turn_out   : ⑥ 과 같은 방향으로 90도 더 -> 아래(도로 쪽)를 봄
//          │   ⑪ down_stem  : 올라온 만큼 직진 (③ + ④ 에서 간 거리)
//          │   ⑫ turn_road  : 왼쪽으로 90도 -> 원래 진행 방향(동쪽)
//          ▼
//   (4) FINISHED  : parking_done 을 보내고 끝. 다시 lane_follower 가 평소처럼 운전
//
// [거리 재는 법]
//   이 노드는 바퀴 엔코더를 아직 안 쓰고 "속도 x 시간" 으로 거리를 정한다.
//   예) 0.08 m/s 로 0.40 m 가려면 5초. 회전도 마찬가지 (90도 = (pi/2) / 회전속도 초)
//
// [위에서 본 영상(BEV) 좌표]  400 x 400 px, 1px ≈ 1mm
//
//        x=0                    x=400
//    y=0  ┌──────────────────────┐   <- 멀리 (로봇 앞쪽)
//         │                      │
//         │━━━━━┓                │   <- 가로 노란선 (차선이 왼쪽으로 꺾인 곳)
//         │     ┃                │
//         │     ┃(차선 노란선)   │
//  y=400  └──────────────────────┘   <- 로봇 바로 앞
//                    ▲ 로봇 (x = 200). 바퀴 축은 화면 맨 아래보다 약 335px 뒤에 있다
//
// [task_planner 와 함께 쓸 때] (use_task_planner: true)
//   control_mode 는 task_planner 가 보낸다. 이 노드는 parking_active (지금 직접 운전 중인지)와
//   parking_done (주차 구간 끝) 만 알려주고, task_planner 가 그걸 보고 모드를 정한다.
//
// [안전]
//   - 전진 중 정면 PSD 가 front_stop_distance 보다 가까우면 멈추고 기다린다.
//   - GUI 에서 STOP / MANUAL 을 누르면 바로 멈추고 주차를 포기한다.
//   - 노란선 따라가기 중 영상이 image_timeout 초 동안 안 오면 멈춘다.
//   - 표지판을 본 뒤 approach_timeout 초가 지나도 가로선을 못 찾으면 주차를 포기한다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/control_mode.hpp"
#include "interfaces/msg/psd_array.hpp"
#include "interfaces/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

// 이 노드가 보내는 control_mode 의 mission_state 앞에 붙는 이름 (내가 보낸 걸 다시 받으면 무시하려고)
static const char * const MY_TAG = "parking_node";

// 회전 방향 (cmd_vel 의 angular.z 부호): + 왼쪽, - 오른쪽
static const double LEFT = +1.0;
static const double RIGHT = -1.0;

// 주차 동작 한 단계
struct Step
{
  std::string name;          // 단계 이름 (로그, GUI 표시용)
  double v;                  // 앞뒤 속도 [m/s]   + 앞, - 뒤
  double w;                  // 회전 속도 [rad/s] + 왼쪽, - 오른쪽
  double duration;           // 이 시간 [s] 동안 한다
};

class ParkingNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;

  enum class State
  {
    WAITING,     // 주차 표지판 기다리는 중
    APPROACH,    // 표지판 봤음. lane_follower 가 운전, 이 노드는 가로 노란선을 찾는 중
    MANEUVER,    // 이 노드가 정해진 단계(직진/회전/정지)를 시간대로 하는 중
    FOLLOW,      // 이 노드가 왼쪽 노란선을 보면서 길을 올라가는 중 (③)
    FINISHED,    // 주차 구간 끝 (또는 포기)
  };

  // 단계 목록은 세 번에 나눠 만든다 (앞 단계가 끝나야 알 수 있는 값이 있어서)
  //   TO_STEM  : ①② 길 입구에서 왼쪽으로 돌기          -> 끝나면 ③ FOLLOW
  //   TO_LOT   : ④⑤ 통로 가운데까지 가서 PSD 보기     -> 끝나면 빈 칸을 정함
  //   PARK_OUT : ⑥~⑫ 주차하고 도로로 돌아가기        -> 끝나면 FINISHED
  enum class Phase
  {
    TO_STEM,
    TO_LOT,
    PARK_OUT,
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
        on_psd(*msg);
      });
    mode_sub_ = create_subscription<ControlMode>(
      "control_mode", 10, [this](ControlMode::ConstSharedPtr msg)
      {
        on_mode(*msg);
      });

    // ---- 보내는 토픽 ----
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    mode_pub_ = create_publisher<ControlMode>("control_mode", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("parking_active", 10);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("parking_done", 10);

    // ---- 0.05초마다 step() ----
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]
      {
        step();
      });

    RCLCPP_INFO(get_logger(), "주차 노드 시작: 주차 표지판 기다림 (task_planner %s)",
      use_task_planner_ ? "사용" : "없음");
  }

private:
  // ==================== 파라미터 (config/parking.yaml) ====================

  void declare_all_parameters()
  {
    use_task_planner_ = declare_parameter("use_task_planner", false);

    // 1) 표지판
    min_confidence_ = declare_parameter("min_confidence", 0.5);
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.0);
    sign_hits_needed_ = declare_parameter("sign_hits_needed", 1);

    // 2) 영상 / 노란선
    image_topic_ = declare_parameter("image_topic", std::string("image_bev"));
    yellow_l_min_ = declare_parameter("yellow_l_min", 80);
    yellow_b_min_ = declare_parameter("yellow_b_min", 150);
    yellow_a_max_ = declare_parameter("yellow_a_max", 135);
    bev_center_x_ = declare_parameter("bev_center_x", 0.5);
    robot_y_offset_px_ = declare_parameter("robot_y_offset_px", 335.0);
    bev_m_per_px_ = declare_parameter("bev_m_per_px", 0.001);
    lane_width_px_ = declare_parameter("lane_width_px", 267.0);
    hline_min_pixels_ = declare_parameter("hline_min_pixels", 35);
    hline_min_rows_ = declare_parameter("hline_min_rows", 3);
    hline_hits_needed_ = declare_parameter("hline_hits_needed", 2);
    approach_timeout_ = declare_parameter("approach_timeout", 40.0);

    // 3) 노란선 따라 길 올라가기
    follow_speed_ = declare_parameter("follow_speed", 0.06);
    follow_ref_ratio_ = declare_parameter("follow_ref_ratio", 0.6);
    line_band_px_ = declare_parameter("line_band_px", 20);
    line_min_pixels_ = declare_parameter("line_min_pixels", 30);
    steer_gain_ = declare_parameter("steer_gain", 1.5);
    max_angular_ = declare_parameter("max_angular", 1.0);
    yellow_end_frames_ = declare_parameter("yellow_end_frames", 3);
    follow_min_time_ = declare_parameter("follow_min_time", 2.0);
    follow_timeout_ = declare_parameter("follow_timeout", 20.0);
    image_timeout_ = declare_parameter("image_timeout", 0.5);

    // 4) 주차 동작 (거리는 실제 맵을 재서 넣기)
    linear_speed_ = declare_parameter("linear_speed", 0.08);
    angular_speed_ = declare_parameter("angular_speed", 0.6);
    align_extra_m_ = declare_parameter("align_extra_m", 0.0);
    lot_extra_m_ = declare_parameter("lot_extra_m", 0.25);
    spot_depth_m_ = declare_parameter("spot_depth_m", 0.25);
    check_time_ = declare_parameter("check_time", 1.0);
    side_psd_threshold_ = declare_parameter("side_psd_threshold", 0.30);
    default_side_ = declare_parameter("default_side", std::string("right"));
    park_wait_time_ = declare_parameter("park_wait_time", 2.0);
    down_stem_m_ = declare_parameter("down_stem_m", 0.0);
    down_extra_m_ = declare_parameter("down_extra_m", 0.0);

    // 안전
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);
    const double rate = declare_parameter("control_rate", 20.0);
    dt_ = 1.0 / rate;
  }

  // ==================== 받은 메시지 처리 ====================

  // 표지판: 주차 표지판이 보이면 APPROACH 시작
  void on_sign(const Sign & sign)
  {
    const bool parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;

    sign_hits_ = parking_sign ? sign_hits_ + 1 : 0;

    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 (신뢰도 %.2f) -> 가로 노란선 찾기 시작",
        sign.confidence);
      state_ = State::APPROACH;
      approach_elapsed_ = 0.0;
      hline_hits_ = 0;
      if (!use_task_planner_)
      {
        publish_mode(ControlMode::LANE, "approach");   // GUI 에 주차 중인 게 보이게 (운전은 lane_follower)
      }
    }
  }

  // 영상: APPROACH 에서는 가로 노란선 찾기, FOLLOW 에서는 왼쪽 노란선 따라가기
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (state_ != State::APPROACH && state_ != State::FOLLOW)
    {
      return;
    }

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

    const cv::Mat yellow = yellow_mask(cv->image);
    if (state_ == State::APPROACH)
    {
      look_for_turn(yellow);
    }
    else
    {
      last_image_time_ = now();
      follow_yellow(yellow);
    }
  }

  // PSD: 저장해 두고, check 단계에서는 모아서 평균을 낸다
  void on_psd(const interfaces::msg::PsdArray & psd)
  {
    psd_ = psd;
    psd_received_ = true;

    if (in_check_step())
    {
      add_sample(left_samples_, psd.left);
      add_sample(right_samples_, psd.right);
    }
  }

  // control_mode: GUI 에서 STOP / MANUAL 을 누르면 포기
  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state.rfind(MY_TAG, 0) == 0)
    {
      return;                    // 내가 보낸 것
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    const bool working = state_ == State::APPROACH || state_ == State::MANEUVER ||
      state_ == State::FOLLOW;
    if (stop_request && working && msg.mission_state == "turtle_gui")
    {
      abort("GUI 에서 STOP/MANUAL");
    }
  }

  // ==================== 노란색 찾기 도우미 ====================

  // 영상에서 노란색인 곳만 255, 나머지 0 인 흑백 그림을 만든다
  cv::Mat yellow_mask(const cv::Mat & bgr) const
  {
    cv::Mat lab;
    cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

    cv::Mat yellow;
    cv::inRange(lab, cv::Scalar(yellow_l_min_, 0, yellow_b_min_),
      cv::Scalar(255, yellow_a_max_, 255), yellow);

    // 작은 점(잡음) 지우기
    const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(yellow, yellow, cv::MORPH_OPEN, k3);
    return yellow;
  }

  // 로봇 왼쪽 절반의 폭 [px]
  int left_width(const cv::Mat & img) const
  {
    return static_cast<int>(bev_center_x_ * img.cols);
  }

  // 로봇 왼쪽 절반에서, 줄(가로 한 줄)마다 노란 점이 몇 개인지 센다
  //   세로로 서 있는 차선은 한 줄에 15~20개 정도, 가로로 누운 선은 훨씬 많다
  std::vector<int> count_left_yellow_per_row(const cv::Mat & yellow) const
  {
    std::vector<int> counts(yellow.rows, 0);
    for (int y = 0; y < yellow.rows; y++)
    {
      counts[y] = cv::countNonZero(yellow.row(y).colRange(0, left_width(yellow)));
    }
    return counts;
  }

  // 노란 점이 많은 줄들이 연속으로 붙어 있는 덩어리 = 가로 노란선 하나.
  // 가장 아래(= 로봇에 가장 가까운) 가로선의 가운데 줄 번호(y)를 돌려준다. 없으면 비어 있음
  std::optional<double> nearest_horizontal_line(const cv::Mat & yellow) const
  {
    const std::vector<int> counts = count_left_yellow_per_row(yellow);
    const int rows = static_cast<int>(counts.size());
    std::optional<double> nearest;
    int start = -1;              // 지금 덩어리가 시작한 줄 (-1 = 덩어리 밖)

    for (int y = 0; y <= rows; y++)
    {
      const bool full = y < rows && counts[y] >= hline_min_pixels_;
      if (full && start < 0)
      {
        start = y;               // 덩어리 시작
      }
      else if (!full && start >= 0)
      {
        if (y - start >= hline_min_rows_)    // 너무 얇으면 잡음
        {
          nearest = (start + y - 1) / 2.0;   // 아래로 갈수록 덮어써서, 마지막 = 가장 아래
        }
        start = -1;              // 덩어리 끝
      }
    }
    return nearest;
  }

  // 기준 줄(ref_y) 위아래 line_band_px 줄 안에서 왼쪽 노란선의 가로 위치 [px]. 없으면 비어 있음
  std::optional<double> left_yellow_x(const cv::Mat & yellow, int ref_y) const
  {
    const int y0 = std::max(0, ref_y - static_cast<int>(line_band_px_));
    const int y1 = std::min(yellow.rows, ref_y + static_cast<int>(line_band_px_) + 1);
    const cv::Mat band = yellow(cv::Range(y0, y1), cv::Range(0, left_width(yellow)));

    const cv::Moments m = cv::moments(band, true);   // 노란 점들의 개수(m00)와 x 합(m10)
    if (m.m00 < line_min_pixels_)
    {
      return std::nullopt;
    }
    return m.m10 / m.m00;
  }

  // ==================== (2) APPROACH: 가로 노란선 찾기 ====================

  void look_for_turn(const cv::Mat & yellow)
  {
    const std::optional<double> line_y = nearest_horizontal_line(yellow);
    hline_hits_ = line_y ? hline_hits_ + 1 : 0;
    if (hline_hits_ < hline_hits_needed_)
    {
      return;
    }

    // 가로선은 길의 가까운 쪽 노란선. 길 가운데는 거기서 차선 폭 절반만큼 더 앞이다.
    // 남은 거리 = (화면 맨 아래 - 길 가운데) + (바퀴 축 ~ 화면 맨 아래)
    const double stem_center_y = *line_y - lane_width_px_ / 2.0;
    const double remain_px = (yellow.rows - 1 - stem_center_y) + robot_y_offset_px_;
    const double remain_m = remain_px * bev_m_per_px_ + align_extra_m_;
    RCLCPP_INFO(get_logger(), "가로 노란선 발견 (화면 y=%.0f) -> %.2fm 더 가서 왼쪽으로",
      *line_y, remain_m);

    steps_.clear();
    add_forward("align", remain_m);              // ①
    add_turn("turn_stem", LEFT, 90.0);           // ②
    begin_steps(Phase::TO_STEM);
  }

  // ==================== (3) FOLLOW: 왼쪽 노란선 따라 길 올라가기 ====================

  void start_follow()
  {
    state_ = State::FOLLOW;
    follow_elapsed_ = 0.0;
    follow_distance_ = 0.0;
    yellow_missing_ = 0;
    last_v_ = 0.0;
    last_image_time_ = now();
    RCLCPP_INFO(get_logger(), "주차 단계: follow (왼쪽 노란선 따라가기)");
  }

  void follow_yellow(const cv::Mat & yellow)
  {
    const int ref_y = static_cast<int>(follow_ref_ratio_ * yellow.rows);
    const std::optional<double> yx = left_yellow_x(yellow, ref_y);
    yellow_missing_ = yx ? 0 : yellow_missing_ + 1;

    // 노란선이 끝남 = 주차장 입구. 끝난 곳은 기준 줄 위치 (바퀴 축에서 이만큼 앞)
    if (follow_elapsed_ >= follow_min_time_ && yellow_missing_ >= yellow_end_frames_)
    {
      const double ref_ahead_m = ((yellow.rows - 1 - ref_y) + robot_y_offset_px_) * bev_m_per_px_;
      RCLCPP_INFO(get_logger(), "노란선 끝 (%.2fm 올라옴) -> 주차장 통로로", follow_distance_);
      start_to_lot(ref_ahead_m + lot_extra_m_);
      return;
    }

    if (!yx)
    {
      drive_follow(0.0);         // 잠깐 안 보이면 직진
      return;
    }

    // 목표 = 노란선에서 차선 폭 절반만큼 오른쪽 (= 길 가운데)
    const double target_x = *yx + lane_width_px_ / 2.0;
    const double dx = target_x - bev_center_x_ * yellow.cols;          // + 면 목표가 오른쪽
    const double ahead = (yellow.rows - 1 - ref_y) + robot_y_offset_px_;
    const double w = std::clamp(-steer_gain_ * std::atan2(dx, ahead), -max_angular_, max_angular_);
    drive_follow(w);
  }

  // 따라가기 중 바퀴 명령 (앞이 막혔으면 멈춤)
  void drive_follow(double w)
  {
    if (front_closer_than(front_stop_distance_))
    {
      stop_wheels();
      last_v_ = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }
    drive(follow_speed_, w);
    last_v_ = follow_speed_;
  }

  // 0.05초마다: 올라온 거리 누적, 영상이 끊기면 정지, 너무 오래 걸리면 포기
  void step_follow()
  {
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::PARKING, "follow");
    }

    follow_elapsed_ += dt_;
    if ((now() - last_image_time_).seconds() > image_timeout_)
    {
      stop_wheels();
      last_v_ = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "영상이 안 옴 -> 정지");
    }
    follow_distance_ += last_v_ * dt_;

    if (follow_timeout_ > 0.0 && follow_elapsed_ >= follow_timeout_)
    {
      abort("노란선 끝(주차장 입구)을 못 찾음 (시간 초과)");
    }
  }

  // ==================== 주차 동작 단계 만들기 ====================

  // 거리 [m] 를 속도 speed 로 가는 데 걸리는 시간 [s]
  static double time_for_distance(double meters, double speed)
  {
    return std::max(0.0, meters) / speed;
  }

  // 각도 [도] 를 angular_speed 로 도는 데 걸리는 시간 [s]
  double time_for_angle(double degrees) const
  {
    return (degrees * M_PI / 180.0) / angular_speed_;
  }

  // 단계 하나 추가하는 도우미들
  void add_forward(const std::string & name, double meters)
  {
    steps_.push_back({name, linear_speed_, 0.0, time_for_distance(meters, linear_speed_)});
  }

  void add_backward(const std::string & name, double meters)
  {
    steps_.push_back({name, -linear_speed_, 0.0, time_for_distance(meters, linear_speed_)});
  }

  void add_turn(const std::string & name, double direction, double degrees)
  {
    steps_.push_back({name, 0.0, direction * angular_speed_, time_for_angle(degrees)});
  }

  void add_stop(const std::string & name, double seconds)
  {
    steps_.push_back({name, 0.0, 0.0, seconds});
  }

  // ④⑤ : 통로 가운데까지 가서 멈추고 PSD 보기
  void start_to_lot(double meters)
  {
    to_lot_m_ = meters;
    steps_.clear();
    add_forward("to_lot", meters);               // ④
    add_stop("check", check_time_);              // ⑤

    left_samples_.clear();
    right_samples_.clear();
    begin_steps(Phase::TO_LOT);
  }

  // ⑥~⑫ : 빈 칸에 주차하고 나와서 도로로 돌아가기. side = 주차할 칸 쪽 (LEFT / RIGHT)
  void start_park_out(double side)
  {
    // 내려갈 거리: 정해 두지 않았으면(0) 올라온 거리(③ + ④) 그대로
    const double down_m = down_stem_m_ > 0.0 ? down_stem_m_ :
      follow_distance_ + to_lot_m_ + down_extra_m_;

    steps_.clear();
    add_turn("turn_in", side, 90.0);             // ⑥ 빈 칸 쪽으로
    add_forward("enter", spot_depth_m_);         // ⑦
    add_stop("wait", park_wait_time_);           // ⑧
    add_backward("back_out", spot_depth_m_);     // ⑨
    add_turn("turn_out", side, 90.0);            // ⑩ 같은 방향으로 90도 더 -> 도로 쪽을 봄
    add_forward("down_stem", down_m);            // ⑪
    add_turn("turn_road", LEFT, 90.0);           // ⑫ 원래 진행 방향으로
    begin_steps(Phase::PARK_OUT);
  }

  void begin_steps(Phase phase)
  {
    phase_ = phase;
    step_index_ = 0;
    step_elapsed_ = 0.0;
    if (state_ != State::MANEUVER && state_ != State::FOLLOW)
    {
      publish_active(true);      // 처음 직접 운전을 시작할 때 한 번
    }
    state_ = State::MANEUVER;
    RCLCPP_INFO(get_logger(), "주차 단계: %s (%.1fs)", steps_[0].name.c_str(), steps_[0].duration);
  }

  // 지금 PSD 를 모으는 단계(⑤ check)인지
  bool in_check_step() const
  {
    return state_ == State::MANEUVER && phase_ == Phase::TO_LOT &&
           step_index_ < steps_.size() && steps_[step_index_].name == "check";
  }

  // 단계 목록을 다 했을 때
  void on_steps_done()
  {
    if (phase_ == Phase::TO_STEM)
    {
      start_follow();
    }
    else if (phase_ == Phase::TO_LOT)
    {
      start_park_out(choose_free_side());
    }
    else
    {
      finish("주차하고 도로로 돌아옴");
    }
  }

  // ==================== 빈 칸 고르기 ====================

  // PSD 값이 정상(0 보다 큼)일 때만 모은다
  static void add_sample(std::vector<double> & samples, double value)
  {
    if (value > 0.0)
    {
      samples.push_back(value);
    }
  }

  // 평균. 모은 게 없으면 아주 큰 값 (= 아무것도 없음)
  static double average(const std::vector<double> & samples)
  {
    if (samples.empty())
    {
      return 99.0;
    }
    double sum = 0.0;
    for (double s : samples)
    {
      sum += s;
    }
    return sum / samples.size();
  }

  // 로봇이 서 있는 칸의 반대쪽을 고른다
  double choose_free_side()
  {
    const double left = average(left_samples_);
    const double right = average(right_samples_);
    const bool left_taken = left <= side_psd_threshold_;
    const bool right_taken = right <= side_psd_threshold_;

    RCLCPP_INFO(get_logger(), "옆 PSD 평균: 왼쪽 %.2fm, 오른쪽 %.2fm", left, right);

    if (left_taken && !right_taken)
    {
      RCLCPP_INFO(get_logger(), "왼쪽 칸에 로봇 있음 -> 오른쪽 칸에 주차");
      return RIGHT;
    }
    if (right_taken && !left_taken)
    {
      RCLCPP_INFO(get_logger(), "오른쪽 칸에 로봇 있음 -> 왼쪽 칸에 주차");
      return LEFT;
    }

    // 둘 다 비었거나 둘 다 막힘 -> 정해 둔 쪽
    RCLCPP_WARN(get_logger(), "어느 칸이 빈지 모르겠음 -> 기본값(%s) 칸에 주차",
      default_side_.c_str());
    return default_side_ == "left" ? LEFT : RIGHT;
  }

  // ==================== 끝내기 ====================

  // 주차 구간 끝
  void finish(const char * reason)
  {
    if (state_ == State::MANEUVER || state_ == State::FOLLOW)
    {
      stop_wheels();
      publish_active(false);
    }
    state_ = State::FINISHED;

    std_msgs::msg::Bool done;
    done.data = true;
    done_pub_->publish(done);
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::LANE, "done");
    }
    RCLCPP_INFO(get_logger(), "%s -> 평소 자율주행으로", reason);
  }

  // 포기
  void abort(const char * reason)
  {
    if (state_ == State::MANEUVER || state_ == State::FOLLOW)
    {
      stop_wheels();
      publish_active(false);
    }
    state_ = State::FINISHED;
    RCLCPP_WARN(get_logger(), "주차 중단: %s", reason);
  }

  // ==================== 0.05초마다 ====================

  void step()
  {
    if (state_ == State::APPROACH)
    {
      check_approach_timeout();  // 운전은 lane_follower 가 하므로 바퀴 명령을 안 보낸다
    }
    else if (state_ == State::FOLLOW)
    {
      step_follow();             // 바퀴 명령은 영상이 올 때마다 follow_yellow() 가 보낸다
    }
    else if (state_ == State::MANEUVER)
    {
      run_current_step();
    }
  }

  // 표지판을 본 뒤 너무 오래 가로선을 못 찾으면 포기 (lane_follower 는 계속 달린다)
  void check_approach_timeout()
  {
    approach_elapsed_ += dt_;
    if (approach_timeout_ > 0.0 && approach_elapsed_ >= approach_timeout_)
    {
      finish("가로 노란선을 못 찾음 (시간 초과)");
    }
  }

  void run_current_step()
  {
    const Step & s = steps_[step_index_];

    // lane_follower 가 멈춰 있도록 "주차 중" 을 계속 알린다 (task_planner 가 있으면 그쪽이 한다)
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::PARKING, s.name);
    }

    // 안전: 전진 중 앞이 너무 가까우면 멈추고 기다린다 (시간도 안 흐름)
    if (s.v > 0.0 && front_closer_than(front_stop_distance_))
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 대기");
      return;
    }

    // 시간이 다 됐으면 다음 단계로
    step_elapsed_ += dt_;
    if (step_elapsed_ >= s.duration)
    {
      go_next_step();
      return;
    }

    drive(s.v, s.w);
  }

  void go_next_step()
  {
    step_index_++;
    step_elapsed_ = 0.0;
    if (step_index_ >= steps_.size())
    {
      stop_wheels();
      on_steps_done();
      return;
    }
    RCLCPP_INFO(get_logger(), "주차 단계: %s (%.1fs)",
      steps_[step_index_].name.c_str(), steps_[step_index_].duration);
  }

  // ==================== 도우미 ====================

  // 정면 PSD 가 distance [m] 보다 가까우면 true
  bool front_closer_than(double distance) const
  {
    if (distance <= 0.0 || !psd_received_)
    {
      return false;
    }
    return psd_.front > 0.0 && psd_.front <= distance;
  }

  void drive(double v, double w)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
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

  void publish_active(bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    active_pub_->publish(msg);
  }

  // ==================== 변수 ====================

  // 파라미터
  bool use_task_planner_;
  double min_confidence_;
  double sign_area_ratio_threshold_;
  int64_t sign_hits_needed_;
  std::string image_topic_;
  int64_t yellow_l_min_;
  int64_t yellow_b_min_;
  int64_t yellow_a_max_;
  double bev_center_x_;
  double robot_y_offset_px_;
  double bev_m_per_px_;
  double lane_width_px_;
  int64_t hline_min_pixels_;
  int64_t hline_min_rows_;
  int64_t hline_hits_needed_;
  double approach_timeout_;
  double follow_speed_;
  double follow_ref_ratio_;
  int64_t line_band_px_;
  int64_t line_min_pixels_;
  double steer_gain_;
  double max_angular_;
  int64_t yellow_end_frames_;
  double follow_min_time_;
  double follow_timeout_;
  double image_timeout_;
  double linear_speed_;
  double angular_speed_;
  double align_extra_m_;
  double lot_extra_m_;
  double spot_depth_m_;
  double check_time_;
  double side_psd_threshold_;
  std::string default_side_;
  double park_wait_time_;
  double down_stem_m_;
  double down_extra_m_;
  double front_stop_distance_;
  double dt_;

  // 상태
  State state_ = State::WAITING;
  Phase phase_ = Phase::TO_STEM;
  int64_t sign_hits_ = 0;
  int64_t hline_hits_ = 0;
  double approach_elapsed_ = 0.0;
  std::vector<Step> steps_;
  size_t step_index_ = 0;
  double step_elapsed_ = 0.0;
  std::vector<double> left_samples_;
  std::vector<double> right_samples_;

  // 노란선 따라가기
  double follow_elapsed_ = 0.0;    // 따라가기 시작한 뒤 지난 시간 [s]
  double follow_distance_ = 0.0;   // 따라가며 올라온 거리 [m] (속도 x 시간)
  double to_lot_m_ = 0.0;          // ④ 에서 간 거리 [m]
  int64_t yellow_missing_ = 0;     // 왼쪽 노란선이 연속으로 안 보인 프레임 수
  double last_v_ = 0.0;            // 마지막으로 보낸 앞 속도 [m/s]
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}

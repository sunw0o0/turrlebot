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
//   아래 도로를 달리다 보면, 카메라(위에서 본 영상 BEV)의 왼쪽에
//   "가로로 누운 노란선 2개" (= 주차장 가는 길의 양쪽 선)가 보인다.
//   그 두 선 사이가 주차장으로 들어가는 길이다.
//
// [전체 흐름]
//
//   (1) WAITING   : 주차 표지판을 기다린다
//          │  주차 표지판이 보임
//          ▼
//   (2) APPROACH  : 운전은 lane_follower 가 계속 한다 (평소처럼 차선 따라가기)
//          │        이 노드는 카메라 영상에서 "왼쪽의 가로 노란선 2개" 를 찾는다
//          │  2개가 보임 -> 두 선의 가운데(= 길 가운데)까지 남은 거리를 계산
//          ▼
//   (3) MANEUVER  : 이 노드가 직접 바퀴를 움직인다 (lane_follower 는 멈춤)
//          │   ① align      : 길 가운데가 바퀴 축 옆에 올 때까지 직진
//          │   ② turn_stem  : 왼쪽으로 90도 (주차장 쪽을 봄)
//          │   ③ up_stem    : 주차장 가운데(통로)까지 직진
//          │   ④ check      : 멈춰서 왼쪽/오른쪽 PSD 로 어느 칸에 로봇이 있는지 본다
//          │                  왼쪽이 가까움 -> 오른쪽 칸에 주차 / 오른쪽이 가까움 -> 왼쪽 칸
//          │   ⑤ turn_in    : 빈 칸 쪽으로 90도
//          │   ⑥ enter      : 칸 안으로 직진
//          │   ⑦ wait       : 서 있기 (주차 완료)
//          │   ⑧ back_out   : 들어간 만큼 후진 (통로 가운데로)
//          │   ⑨ turn_out   : ⑤ 와 같은 방향으로 90도 더 -> 아래(도로 쪽)를 봄
//          │   ⑩ down_stem  : 도로까지 직진
//          │   ⑪ turn_road  : 왼쪽으로 90도 -> 원래 진행 방향(동쪽)
//          ▼
//   (4) FINISHED  : parking_done 을 보내고 끝. 다시 lane_follower 가 평소처럼 운전
//
// [거리 재는 법]
//   이 노드는 바퀴 엔코더를 아직 안 쓰고 "속도 x 시간" 으로 거리를 정한다.
//   예) 0.08 m/s 로 0.40 m 가려면 5초. 회전도 마찬가지 (90도 = (pi/2) / 회전속도 초)
//   그래서 parking.yaml 의 거리(stem_length_m 등)는 실제 맵을 줄자로 재서 넣어야 한다.
//
// [위에서 본 영상(BEV) 좌표]  400 x 400 px, 1px ≈ 1mm
//
//        x=0                    x=400
//    y=0  ┌──────────────────────┐   <- 멀리 (로봇 앞쪽)
//         │━━━━━┓                │   <- 가로 노란선 (먼 쪽)
//         │     ┃                │
//         │━━━━━┛                │   <- 가로 노란선 (가까운 쪽)
//         │   ┃ (차선 노란선)    │
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
//   - 표지판을 본 뒤 approach_timeout 초가 지나도 길을 못 찾으면 주차를 포기한다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
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
    APPROACH,    // 표지판 봤음. lane_follower 가 운전, 이 노드는 가로 노란선 2개를 찾는 중
    MANEUVER,    // 이 노드가 직접 주차 동작 중
    FINISHED,    // 주차 구간 끝 (또는 포기)
  };

  // 주차 동작은 두 번에 나눠 만든다.
  //   TO_LOT   : ①~④ 주차장 가운데까지 가서 멈추고 PSD 보기
  //   PARK_OUT : ⑤~⑪ (④ 에서 어느 칸이 빈지 알아야 방향을 정할 수 있으므로 그 뒤에 만든다)
  enum class Phase
  {
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

    // 2) 가로 노란선 찾기
    image_topic_ = declare_parameter("image_topic", std::string("image_bev"));
    yellow_l_min_ = declare_parameter("yellow_l_min", 80);
    yellow_b_min_ = declare_parameter("yellow_b_min", 150);
    yellow_a_max_ = declare_parameter("yellow_a_max", 135);
    bev_center_x_ = declare_parameter("bev_center_x", 0.5);
    robot_y_offset_px_ = declare_parameter("robot_y_offset_px", 335.0);
    bev_m_per_px_ = declare_parameter("bev_m_per_px", 0.001);
    hline_min_pixels_ = declare_parameter("hline_min_pixels", 35);
    hline_min_rows_ = declare_parameter("hline_min_rows", 3);
    stem_gap_min_px_ = declare_parameter("stem_gap_min_px", 120);
    stem_gap_max_px_ = declare_parameter("stem_gap_max_px", 380);
    stem_hits_needed_ = declare_parameter("stem_hits_needed", 2);
    approach_timeout_ = declare_parameter("approach_timeout", 40.0);

    // 3) 주차 동작 (거리는 실제 맵을 재서 넣기)
    linear_speed_ = declare_parameter("linear_speed", 0.08);
    angular_speed_ = declare_parameter("angular_speed", 0.6);
    align_extra_m_ = declare_parameter("align_extra_m", 0.0);
    stem_length_m_ = declare_parameter("stem_length_m", 0.80);
    spot_depth_m_ = declare_parameter("spot_depth_m", 0.25);
    check_time_ = declare_parameter("check_time", 1.0);
    side_psd_threshold_ = declare_parameter("side_psd_threshold", 0.30);
    default_side_ = declare_parameter("default_side", std::string("right"));
    park_wait_time_ = declare_parameter("park_wait_time", 2.0);

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
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 (신뢰도 %.2f) -> 주차장 가는 길 찾기 시작",
        sign.confidence);
      state_ = State::APPROACH;
      approach_elapsed_ = 0.0;
      stem_hits_ = 0;
      if (!use_task_planner_)
      {
        publish_mode(ControlMode::LANE, "approach");   // GUI 에 주차 중인 게 보이게 (운전은 lane_follower)
      }
    }
  }

  // 영상: APPROACH 중에만 가로 노란선 2개를 찾는다
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (state_ != State::APPROACH)
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

    double stem_center_y = 0.0;
    const bool found = find_stem(cv->image, stem_center_y);
    stem_hits_ = found ? stem_hits_ + 1 : 0;

    if (stem_hits_ >= stem_hits_needed_)
    {
      // 길 가운데까지 남은 거리 = (화면 맨 아래 - 길 가운데) + (바퀴 축 ~ 화면 맨 아래)
      const double bottom_y = cv->image.rows - 1;
      const double remain_px = (bottom_y - stem_center_y) + robot_y_offset_px_;
      const double remain_m = remain_px * bev_m_per_px_ + align_extra_m_;
      RCLCPP_INFO(get_logger(), "주차장 가는 길 발견 (화면 y=%.0f) -> %.2fm 더 가서 왼쪽으로",
        stem_center_y, remain_m);
      start_to_lot(remain_m);
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
    const bool working = state_ == State::APPROACH || state_ == State::MANEUVER;
    if (stop_request && working && msg.mission_state == "turtle_gui")
    {
      abort("GUI 에서 STOP/MANUAL");
    }
  }

  // ==================== 가로 노란선 2개 찾기 ====================

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

  // 로봇 왼쪽 절반에서, 줄(가로 한 줄)마다 노란 점이 몇 개인지 센다
  //   세로로 서 있는 차선은 한 줄에 15~20개 정도, 가로로 누운 선은 훨씬 많다
  std::vector<int> count_left_yellow_per_row(const cv::Mat & yellow) const
  {
    const int left_width = static_cast<int>(bev_center_x_ * yellow.cols);
    std::vector<int> counts(yellow.rows, 0);
    for (int y = 0; y < yellow.rows; y++)
    {
      const cv::Mat row = yellow.row(y).colRange(0, left_width);
      counts[y] = cv::countNonZero(row);
    }
    return counts;
  }

  // 노란 점이 많은 줄들이 연속으로 붙어 있는 덩어리 = 가로 노란선 하나.
  // 각 덩어리의 가운데 줄 번호(y)를 위에서부터 순서대로 돌려준다
  std::vector<double> find_horizontal_lines(const std::vector<int> & counts) const
  {
    std::vector<double> lines;
    const int rows = static_cast<int>(counts.size());
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
        const int thickness = y - start;     // 덩어리 두께 [줄]
        if (thickness >= hline_min_rows_)
        {
          lines.push_back((start + y - 1) / 2.0);
        }
        start = -1;              // 덩어리 끝
      }
    }
    return lines;
  }

  // 가로 노란선 2개가 알맞은 간격으로 보이면 true, 그 가운데 y 를 center_y 에 넣는다
  bool find_stem(const cv::Mat & bgr, double & center_y) const
  {
    const cv::Mat yellow = yellow_mask(bgr);
    const std::vector<double> lines = find_horizontal_lines(count_left_yellow_per_row(yellow));

    // 이웃한 두 선씩 비교해서 간격이 길 폭 정도인 쌍을 찾는다 (아래쪽 = 가까운 쌍부터)
    for (int i = static_cast<int>(lines.size()) - 1; i >= 1; i--)
    {
      const double gap = lines[i] - lines[i - 1];
      if (gap >= stem_gap_min_px_ && gap <= stem_gap_max_px_)
      {
        center_y = (lines[i] + lines[i - 1]) / 2.0;
        return true;
      }
    }
    return false;
  }

  // ==================== 주차 동작 만들기 ====================

  // 거리 [m] 를 linear_speed 로 가는 데 걸리는 시간 [s]
  double time_for_distance(double meters) const
  {
    return std::max(0.0, meters) / linear_speed_;
  }

  // 각도 [도] 를 angular_speed 로 도는 데 걸리는 시간 [s]
  double time_for_angle(double degrees) const
  {
    return (degrees * M_PI / 180.0) / angular_speed_;
  }

  // 단계 하나 추가하는 도우미들
  void add_forward(const std::string & name, double meters)
  {
    steps_.push_back({name, linear_speed_, 0.0, time_for_distance(meters)});
  }

  void add_backward(const std::string & name, double meters)
  {
    steps_.push_back({name, -linear_speed_, 0.0, time_for_distance(meters)});
  }

  void add_turn(const std::string & name, double direction, double degrees)
  {
    steps_.push_back({name, 0.0, direction * angular_speed_, time_for_angle(degrees)});
  }

  void add_stop(const std::string & name, double seconds)
  {
    steps_.push_back({name, 0.0, 0.0, seconds});
  }

  // ①~④ : 길 가운데까지 가서 왼쪽으로 돌고, 주차장 가운데까지 가서 멈추고 PSD 보기
  void start_to_lot(double remain_m)
  {
    steps_.clear();
    add_forward("align", remain_m);              // ①
    add_turn("turn_stem", LEFT, 90.0);           // ②
    add_forward("up_stem", stem_length_m_);      // ③
    add_stop("check", check_time_);              // ④

    left_samples_.clear();
    right_samples_.clear();
    phase_ = Phase::TO_LOT;
    begin_steps();
  }

  // ⑤~⑪ : 빈 칸에 주차하고 나와서 도로로 돌아가기. side = 주차할 칸 쪽 (LEFT / RIGHT)
  void start_park_out(double side)
  {
    steps_.clear();
    add_turn("turn_in", side, 90.0);             // ⑤ 빈 칸 쪽으로
    add_forward("enter", spot_depth_m_);         // ⑥
    add_stop("wait", park_wait_time_);           // ⑦
    add_backward("back_out", spot_depth_m_);     // ⑧
    add_turn("turn_out", side, 90.0);            // ⑨ 같은 방향으로 90도 더 -> 도로 쪽을 봄
    add_forward("down_stem", stem_length_m_);    // ⑩
    add_turn("turn_road", LEFT, 90.0);           // ⑪ 원래 진행 방향으로

    phase_ = Phase::PARK_OUT;
    begin_steps();
  }

  void begin_steps()
  {
    step_index_ = 0;
    step_elapsed_ = 0.0;
    state_ = State::MANEUVER;
    publish_active(true);
    RCLCPP_INFO(get_logger(), "주차 단계: %s (%.1fs)", steps_[0].name.c_str(), steps_[0].duration);
  }

  // 지금 PSD 를 모으는 단계(④ check)인지
  bool in_check_step() const
  {
    return state_ == State::MANEUVER && phase_ == Phase::TO_LOT &&
           step_index_ < steps_.size() && steps_[step_index_].name == "check";
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

  // 단계 목록을 다 했을 때
  void on_steps_done()
  {
    if (phase_ == Phase::TO_LOT)
    {
      start_park_out(choose_free_side());
      return;
    }
    finish("주차하고 도로로 돌아옴");
  }

  // 주차 구간 끝
  void finish(const char * reason)
  {
    if (state_ == State::MANEUVER)
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
    if (state_ == State::MANEUVER)
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
      check_approach_timeout();
      return;                    // APPROACH 에서는 lane_follower 가 운전하므로 바퀴 명령을 안 보낸다
    }
    if (state_ == State::MANEUVER)
    {
      run_current_step();
    }
  }

  // 표지판을 본 뒤 너무 오래 길을 못 찾으면 포기 (lane_follower 는 계속 달린다)
  void check_approach_timeout()
  {
    approach_elapsed_ += dt_;
    if (approach_timeout_ > 0.0 && approach_elapsed_ >= approach_timeout_)
    {
      finish("주차장 가는 길을 못 찾음 (시간 초과)");
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
  int64_t hline_min_pixels_;
  int64_t hline_min_rows_;
  int64_t stem_gap_min_px_;
  int64_t stem_gap_max_px_;
  int64_t stem_hits_needed_;
  double approach_timeout_;
  double linear_speed_;
  double angular_speed_;
  double align_extra_m_;
  double stem_length_m_;
  double spot_depth_m_;
  double check_time_;
  double side_psd_threshold_;
  std::string default_side_;
  double park_wait_time_;
  double front_stop_distance_;
  double dt_;

  // 상태
  State state_ = State::WAITING;
  Phase phase_ = Phase::TO_LOT;
  int64_t sign_hits_ = 0;
  int64_t stem_hits_ = 0;
  double approach_elapsed_ = 0.0;
  std::vector<Step> steps_;
  size_t step_index_ = 0;
  double step_elapsed_ = 0.0;
  std::vector<double> left_samples_;
  std::vector<double> right_samples_;

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

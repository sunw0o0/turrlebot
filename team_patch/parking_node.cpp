// ============================================================================
//  parking_node.cpp - 주차 모드: 흰선은 무시하고 노란선만 따라간다
// ============================================================================
//
// [전체 흐름]
//
//   (1) WAITING   : 주차 표지판을 기다린다. 운전은 lane_follower 가 한다 (평소 주행)
//          │  주차 표지판이 보임
//          ▼
//   (2) PARKING   : 이 노드가 직접 운전한다 (control_mode = PARKING, lane_follower 는 멈춤)
//          │        카메라 영상(위에서 본 BEV)에서 노란색만 찾는다. 흰선은 아예 안 본다.
//          │          - 노란선이 왼쪽, 오른쪽 둘 다 보임 -> 두 선의 가운데로
//          │          - 왼쪽에만 보임                 -> 그 선에서 차선 폭 절반만큼 오른쪽으로
//          │          - 오른쪽에만 보임               -> 그 선에서 차선 폭 절반만큼 왼쪽으로
//          │          - 하나도 안 보임                -> 마지막으로 노란선을 본 쪽으로 천천히 돌며 찾기
//          │                                             (처음에는 왼쪽에 있다고 본다)
//          │
//          │  GUI 에서 STOP / MANUAL 을 누르면 멈추고 주차 모드 끝
//          ▼
//   (3) FINISHED  : 끝 (다시 시작하지 않음)
//
//   ※ 주차 모드를 언제 끝낼지(주차 칸에 들어가기, 다시 평소 주행으로 돌아가기)는 아직 없다.
//
// [노란선 위치 재는 법]
//   화면 한 줄만 보면 선이 끊긴 곳이나 가로로 누운 곳에서 크게 흔들린다. 그래서
//     (1) 줄마다 노란 점들의 가운데 x 를 하나씩 모으고 (가로로 누운 선 줄은 뺀다)
//     (2) 그 점들을 곡선 x = a + b*y + c*y^2 로 잇는다
//   이렇게 하면 선이 잠깐 끊겨도 앞뒤를 이어서 어디 있는지 안다.
//
// [핸들 돌리는 법]  (카메라는 바퀴 축보다 약 33cm 앞을 본다)
//   - 위치 오차 : 차선 가운데 곡선을 바퀴 축까지 이어서, 로봇이 가운데에서 얼마나 벗어났는지
//   - 방향 오차 : 화면 맨 아래에서 차선이 로봇 정면보다 얼마나 틀어져 있는지
//   회전 속도 = -(heading_gain x 방향 오차 + offset_gain x 위치 오차 각도)
//   화면 먼 곳 한 점만 보고 가면 커브에서 일찍 꺾어 안쪽으로 파고든다. 이 방법은 그걸 줄인다.
//
// [위에서 본 영상(BEV) 좌표]  400 x 400 px, 1px ≈ 1mm
//
//        x=0                    x=400
//    y=0  ┌──────────────────────┐   <- 멀리 (로봇 앞쪽)
//         │   ┃           ┃      │
//         │   ┃  노란선   ┃      │
//         │   ┃           ┃      │
//  y=400  └──────────────────────┘   <- 로봇 바로 앞
//                    ▲ 로봇 (x = 200). 바퀴 축은 화면 맨 아래보다 약 335px 뒤에 있다
//
// [task_planner 와 함께 쓸 때] (use_task_planner: true)
//   control_mode 는 task_planner 가 보낸다. 이 노드는 parking_active (지금 직접 운전 중인지)만
//   알려주고, task_planner 가 그걸 보고 PARKING 모드로 바꾼다.
//
// [안전]
//   - 정면 PSD 가 front_stop_distance 보다 가까우면 멈추고 기다린다.
//   - 영상이 image_timeout 초 동안 안 오면 멈춘다.

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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

// 이 노드가 보내는 control_mode 의 mission_state 앞에 붙는 이름 (내가 보낸 걸 다시 받으면 무시하려고)
static const char * const MY_TAG = "parking_node";

// 화면 위의 선 하나: x = a + b*y + c*y^2  (y 는 화면 줄 번호, 아래로 갈수록 커짐)
struct Curve
{
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  // 줄 y 에서의 가로 위치 [px]
  double x_at(double y) const
  {
    return a + b * y + c * y * y;
  }

  // 줄 y 에서 기울기 (y 가 1 커질 때 x 가 얼마나 변하는지)
  double slope_at(double y) const
  {
    return b + 2.0 * c * y;
  }

  // 옆으로 dx [px] 만큼 옮긴 선
  Curve shifted(double dx) const
  {
    Curve moved = *this;
    moved.a += dx;
    return moved;
  }

  // 두 선의 가운데 선
  static Curve middle(const Curve & p, const Curve & q)
  {
    Curve m;
    m.a = (p.a + q.a) / 2.0;
    m.b = (p.b + q.b) / 2.0;
    m.c = (p.c + q.c) / 2.0;
    return m;
  }
};

class ParkingNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;

  enum class State
  {
    WAITING,     // 주차 표지판 기다리는 중
    PARKING,     // 주차 모드: 이 노드가 노란선만 보고 운전
    FINISHED,    // 끝
  };

  // 한 프레임에서 찾은 노란선. 없으면 비어 있음
  struct YellowLines
  {
    std::optional<Curve> left;       // 로봇보다 왼쪽에 있는 노란선
    std::optional<Curve> right;      // 로봇보다 오른쪽에 있는 노란선
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
    active_pub_ = create_publisher<std_msgs::msg::Bool>("parking_active", 10);

    // ---- 0.1초마다: 모드 알림, 영상 끊김 확인 ----
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]
      {
        watchdog();
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

    // 2) 노란색 찾기
    image_topic_ = declare_parameter("image_topic", std::string("image_bev"));
    yellow_l_min_ = declare_parameter("yellow_l_min", 80);
    yellow_b_min_ = declare_parameter("yellow_b_min", 150);
    yellow_a_max_ = declare_parameter("yellow_a_max", 135);
    bev_center_x_ = declare_parameter("bev_center_x", 0.5);
    robot_y_offset_px_ = declare_parameter("robot_y_offset_px", 335.0);
    lane_width_px_ = declare_parameter("lane_width_px", 267.0);
    hline_min_pixels_ = declare_parameter("hline_min_pixels", 35);
    line_min_rows_ = declare_parameter("line_min_rows", 20);
    min_pair_gap_px_ = declare_parameter("min_pair_gap_px", 150.0);
    last_side_ = declare_parameter("initial_side", std::string("left")) == "right" ? -1.0 : +1.0;

    // 3) 운전
    speed_ = declare_parameter("speed", 0.07);
    heading_gain_ = declare_parameter("heading_gain", 1.0);
    offset_gain_ = declare_parameter("offset_gain", 1.0);
    offset_soft_px_ = declare_parameter("offset_soft_px", 300.0);
    max_angular_ = declare_parameter("max_angular", 1.0);
    lost_turn_ = declare_parameter("lost_turn", 0.3);
    image_timeout_ = declare_parameter("image_timeout", 0.5);
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);
  }

  // ==================== 표지판 / 모드 ====================

  // 주차 표지판이 보이면 주차 모드 시작
  void on_sign(const Sign & sign)
  {
    const bool parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;

    sign_hits_ = parking_sign ? sign_hits_ + 1 : 0;

    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "주차 표지판 인식 (신뢰도 %.2f) -> 주차 모드: 노란선만 따라감",
        sign.confidence);
      state_ = State::PARKING;
      last_image_time_ = now();
      publish_active(true);
      if (!use_task_planner_)
      {
        publish_mode(ControlMode::PARKING, "yellow");
      }
    }
  }

  // GUI 에서 STOP / MANUAL 을 누르면 주차 모드 끝
  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state.rfind(MY_TAG, 0) == 0)
    {
      return;                    // 내가 보낸 것
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    if (state_ == State::PARKING && stop_request && msg.mission_state == "turtle_gui")
    {
      stop("GUI 에서 STOP/MANUAL");
    }
  }

  // ==================== 영상 한 장마다: 노란선 보고 운전 ====================

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (state_ != State::PARKING)
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

    const cv::Mat yellow = yellow_mask(cv->image);                     // 1) 노란색만 남기기
    const YellowLines lines = find_yellow_lines(yellow);               // 2) 노란선 찾기
    remember_side(lines);

    if (!lines.left && !lines.right)
    {
      drive(speed_, last_side_ * lost_turn_);                          // 노란선 없음 -> 본 쪽으로 돌며 찾기
      return;
    }
    const Curve center = lane_center(lines);                           // 3) 차선 가운데 선
    const double w = steer(center, yellow);                            // 4) 회전 속도
    drive(speed_, w);                                                  // 5) 움직이기
  }

  // 노란선이 어느 쪽에 있었는지 기억 (+1 왼쪽, -1 오른쪽). 둘 다 보이면 그대로 둔다
  void remember_side(const YellowLines & lines)
  {
    if (lines.left && !lines.right)
    {
      last_side_ = +1.0;
    }
    else if (lines.right && !lines.left)
    {
      last_side_ = -1.0;
    }
  }

  // 1) 영상에서 노란색인 곳만 255, 나머지 0 인 흑백 그림 (흰색은 여기서 이미 빠진다)
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

  // 2) 로봇 왼쪽 절반, 오른쪽 절반에서 각각 노란선을 찾는다
  YellowLines find_yellow_lines(const cv::Mat & yellow) const
  {
    const int mid = static_cast<int>(bev_center_x_ * yellow.cols);
    const double bottom_y = yellow.rows - 1;

    YellowLines lines;
    lines.left = fit_line(yellow, 0, mid);
    lines.right = fit_line(yellow, mid, yellow.cols);

    // 왼쪽/오른쪽 "선" 이 화면 맨 아래에서 너무 가까우면 로봇 가운데에 걸친 선 하나다.
    // 화면 전체에서 선 하나로 다시 찾고, 마지막으로 노란선을 본 쪽의 선으로 친다
    if (lines.left && lines.right &&
      lines.right->x_at(bottom_y) - lines.left->x_at(bottom_y) < min_pair_gap_px_)
    {
      const std::optional<Curve> one = fit_line(yellow, 0, yellow.cols);
      lines = YellowLines();
      if (last_side_ < 0.0)
      {
        lines.right = one;
      }
      else
      {
        lines.left = one;
      }
    }
    return lines;
  }

  // 가로 범위 [x0, x1) 안의 노란선을 곡선으로 잇는다
  //   (1) 줄마다 노란 점들의 가운데 x 를 하나씩 모은다
  //       - 노란 점이 너무 많은 줄(가로로 누운 선)은 뺀다
  //   (2) 모은 점들을 x = a + b*y + c*y^2 곡선으로 잇는다 (최소제곱법)
  //   줄이 line_min_rows 개보다 적으면 "선 없음"
  std::optional<Curve> fit_line(const cv::Mat & yellow, int x0, int x1) const
  {
    std::vector<double> ys;
    std::vector<double> xs;
    for (int y = 0; y < yellow.rows; y++)
    {
      const cv::Moments m = cv::moments(yellow.row(y).colRange(x0, x1), true);  // m00 = 점 개수
      if (m.m00 < 1.0 || m.m00 >= hline_min_pixels_)
      {
        continue;                // 노란 점 없음, 또는 가로로 누운 선
      }
      ys.push_back(y);
      xs.push_back(x0 + m.m10 / m.m00);                // m10 / m00 = 점들의 평균 x
    }
    if (static_cast<int64_t>(ys.size()) < line_min_rows_)
    {
      return std::nullopt;
    }
    return fit_curve(ys, xs);
  }

  // 점들 (y, x) 를 지나는 곡선 x = a + b*y + c*y^2 (점들과의 차이가 가장 작게)
  //   점이 위아래로 짧게 몰려 있으면 곡선이 이상하게 휘므로 직선(c = 0)으로 잇는다
  static Curve fit_curve(const std::vector<double> & ys, const std::vector<double> & xs)
  {
    const double span = *std::max_element(ys.begin(), ys.end()) -
      *std::min_element(ys.begin(), ys.end());
    const int terms = span >= 100.0 ? 3 : 2;           // 3 = 곡선, 2 = 직선
    const int n = static_cast<int>(ys.size());

    cv::Mat A(n, terms, CV_64F);                       // 각 줄: [1, y, y^2]
    cv::Mat B(n, 1, CV_64F);                           // 각 줄: x
    for (int i = 0; i < n; i++)
    {
      for (int k = 0; k < terms; k++)
      {
        A.at<double>(i, k) = std::pow(ys[i], k);
      }
      B.at<double>(i) = xs[i];
    }

    cv::Mat coef;
    cv::solve(A, B, coef, cv::DECOMP_SVD);             // 최소제곱으로 a, b, (c) 구하기

    Curve curve;
    curve.a = coef.at<double>(0);
    curve.b = coef.at<double>(1);
    curve.c = terms == 3 ? coef.at<double>(2) : 0.0;
    return curve;
  }

  // 3) 차선 가운데 선 (노란선이 하나 이상 보일 때만 부른다)
  Curve lane_center(const YellowLines & lines) const
  {
    const double half_lane = lane_width_px_ / 2.0;

    if (lines.left && lines.right)
    {
      return Curve::middle(*lines.left, *lines.right);  // 두 노란선 가운데
    }
    if (lines.left)
    {
      return lines.left->shifted(+half_lane);           // 왼쪽 선에서 반 차선 오른쪽
    }
    return lines.right->shifted(-half_lane);            // 오른쪽 선에서 반 차선 왼쪽
  }

  // 4) 회전 속도 [rad/s]. + 왼쪽, - 오른쪽
  double steer(const Curve & center, const cv::Mat & img) const
  {
    const double robot_x = bev_center_x_ * img.cols;
    const double bottom_y = img.rows - 1;
    const double slope = center.slope_at(bottom_y);

    // 방향 오차: 화면 맨 아래에서 차선이 오른쪽으로 틀어진 각도
    //   (위로 1px 갈 때 x 가 -slope 만큼 변하므로 오른쪽으로 atan(-slope))
    const double heading_right = std::atan(-slope);

    // 위치 오차: 차선 가운데를 화면 맨 아래에서 바퀴 축까지 직선으로 이어서,
    //   바퀴 축 높이에서 차선 가운데가 로봇보다 오른쪽에 얼마나 있는지 [px]
    const double center_at_axle = center.x_at(bottom_y) + slope * robot_y_offset_px_;
    const double offset_right = center_at_axle - robot_x;

    const double w = -(heading_gain_ * heading_right +
      offset_gain_ * std::atan2(offset_right, offset_soft_px_));
    return std::clamp(w, -max_angular_, max_angular_);
  }

  // ==================== 0.1초마다 ====================

  void watchdog()
  {
    if (state_ != State::PARKING)
    {
      return;
    }

    // lane_follower 가 멈춰 있도록 "주차 중" 을 계속 알린다 (task_planner 가 있으면 그쪽이 한다)
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::PARKING, "yellow");
    }

    // 영상이 끊기면 멈춤
    if ((now() - last_image_time_).seconds() > image_timeout_)
    {
      stop_wheels();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "영상이 안 옴 -> 정지");
    }
  }

  // ==================== 도우미 ====================

  // 5) 바퀴 명령. 앞이 막혔으면 멈춤
  void drive(double v, double w)
  {
    if (front_closer_than(front_stop_distance_))
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

  // 주차 모드 끝 (멈추고 다시 시작하지 않음)
  void stop(const char * reason)
  {
    stop_wheels();
    publish_active(false);
    state_ = State::FINISHED;
    RCLCPP_WARN(get_logger(), "주차 모드 끝: %s", reason);
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
  double lane_width_px_;
  int64_t hline_min_pixels_;
  int64_t line_min_rows_;
  double min_pair_gap_px_;
  double speed_;
  double heading_gain_;
  double offset_gain_;
  double offset_soft_px_;
  double max_angular_;
  double lost_turn_;
  double image_timeout_;
  double front_stop_distance_;

  // 상태
  State state_ = State::WAITING;
  int64_t sign_hits_ = 0;
  double last_side_ = +1.0;        // 노란선을 마지막으로 본 쪽 (+1 왼쪽, -1 오른쪽)
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
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingNode>());
  rclcpp::shutdown();
  return 0;
}

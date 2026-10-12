// ============================================================================
//  construction_node.cpp - 공사 구간: 공사 표지판을 보면 장애물(갈색 박스)을 피해 간다
// ============================================================================
//
// [전체 흐름]
//
//   (1) WAITING  : 공사 표지판(sign_construction)이 가까이 3번 연속 보일 때까지 기다린다
//          ▼
//   (2) AVOIDING : 이 노드가 직접 운전한다 (control_mode = CONSTRUCTION, lane_follower 는 멈춤)
//          │  위에서 본 영상(image_bev)에서 매 프레임:
//          │    - 노란선(왼쪽), 흰선(오른쪽), 갈색 박스(장애물)를 색으로 찾는다
//          │    - 장애물 + 차선이 보이면 : 장애물과 차선 사이 빈 공간 중 넓은 쪽으로 간다
//          │                              (= 장애물 옆에 붙은 차선에서 멀어지는 방향)
//          │                              단, 로봇 중심은 항상 두 차선 안쪽에 둔다
//          │    - 장애물만 있고 차선이 없으면 : 직진
//          │    - 장애물이 없으면       : 차선 가운데로 (차선도 없으면 직진)
//          │  주차 표지판이 보이면 공사 구간 끝 (주차 노드가 같은 순간 이어받음)
//          │  (exit_clear_time > 0 이면 장애물이 그 시간 동안 안 보여도 끝)
//          ▼
//   (3) FINISHED : control_mode 를 LANE 으로 돌려준다 -> 평소 자율주행
//
// [위에서 본 영상(BEV) 좌표]  400 x 400 px, 1px ≈ 1mm (bird_eye_view 보정값)
//
//        x=0                    x=400
//    y=0  ┌──────────────────────┐   <- 멀리
//         │   노란선    흰선      │
//         │     |   [박스]  |     │
//         │     |          |     │
//  y=400  └──────────────────────┘   <- 로봇 바로 앞
//                    ▲ 로봇 (x = 200)
//
// [task_planner 와 함께 쓸 때] (use_task_planner: true)
//   control_mode 는 task_planner 가 보낸다. 이 노드는 construction_active 만 알려준다.

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

static const char * const MY_TAG = "construction_node";

class ConstructionNode : public rclcpp::Node
{
  using ControlMode = interfaces::msg::ControlMode;
  using Sign = interfaces::msg::Sign;

  enum class State
  {
    WAITING,     // 공사 표지판 기다리는 중
    AVOIDING,    // 공사 구간: 이 노드가 운전
    FINISHED,    // 끝 (다시 시작하지 않음)
  };

  // 한 프레임에서 찾은 것들
  struct Scene
  {
    std::optional<double> left_x;      // 노란선 가로 위치 [px] (기준 줄에서)
    std::optional<double> right_x;     // 흰선 가로 위치 [px]
    bool obstacle = false;             // 장애물 있음
    int obs_x0 = 0;                    // 장애물 왼쪽 끝 [px]
    int obs_x1 = 0;                    // 장애물 오른쪽 끝 [px]
    int ref_y = 0;                     // 위 값들을 잰 가로줄 [px] (장애물 아래 끝, 없으면 화면 아래쪽)
  };

public:
  ConstructionNode()
  : Node("construction")
  {
    // ---- 파라미터 (config/construction.yaml) ----
    use_task_planner_ = declare_parameter("use_task_planner", false);
    min_confidence_ = declare_parameter("min_confidence", 0.5);
    sign_area_ratio_threshold_ = declare_parameter("sign_area_ratio_threshold", 0.02);
    sign_hits_needed_ = declare_parameter("sign_hits_needed", 3);
    // 끝: 주차 표지판 (parking.yaml 의 표지판 조건과 같게)
    parking_sign_area_ratio_threshold_ = declare_parameter("parking_sign_area_ratio_threshold", 0.0);
    parking_sign_hits_needed_ = declare_parameter("parking_sign_hits_needed", 1);
    const auto image_topic = declare_parameter("image_topic", std::string("image_bev"));
    // 색 (lane_detection / path_planner 와 같은 실측값, OpenCV LAB)
    yellow_l_min_ = declare_parameter("yellow_l_min", 80);
    yellow_b_min_ = declare_parameter("yellow_b_min", 150);
    yellow_a_max_ = declare_parameter("yellow_a_max", 135);
    white_l_min_ = declare_parameter("white_l_min", 190);
    white_ab_dev_ = declare_parameter("white_ab_dev", 15);
    box_l_min_ = declare_parameter("box_l_min", 10);
    box_l_max_ = declare_parameter("box_l_max", 110);
    box_a_min_ = declare_parameter("box_a_min", 137);
    box_a_max_ = declare_parameter("box_a_max", 165);
    box_b_min_ = declare_parameter("box_b_min", 128);
    box_b_max_ = declare_parameter("box_b_max", 152);
    box_min_b_minus_a_ = declare_parameter("box_min_b_minus_a", -15);
    box_min_area_ = declare_parameter("box_min_area", 300);
    // 크기 / 위치
    bev_center_x_ = declare_parameter("bev_center_x", 0.5);
    lane_width_px_ = declare_parameter("lane_width_px", 267.0);
    robot_y_offset_px_ = declare_parameter("robot_y_offset_px", 335.0);
    line_band_px_ = declare_parameter("line_band_px", 20);
    line_min_pixels_ = declare_parameter("line_min_pixels", 30);
    lane_ref_ratio_ = declare_parameter("lane_ref_ratio", 0.75);
    line_keep_px_ = declare_parameter("line_keep_px", 40.0);
    // 운전
    avoid_speed_ = declare_parameter("avoid_speed", 0.07);
    steer_gain_ = declare_parameter("steer_gain", 1.5);
    max_angular_ = declare_parameter("max_angular", 1.0);
    exit_clear_time_ = declare_parameter("exit_clear_time", 0.0);
    zone_timeout_ = declare_parameter("zone_timeout", 0.0);
    image_timeout_ = declare_parameter("image_timeout", 0.5);
    front_stop_distance_ = declare_parameter("front_stop_distance", 0.10);

    // ---- 받는 토픽 ----
    const auto qos = rclcpp::SensorDataQoS();
    sign_sub_ = create_subscription<Sign>(
      "sign", qos, [this](Sign::ConstSharedPtr msg)
      {
        on_sign(*msg);
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg)
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
    active_pub_ = create_publisher<std_msgs::msg::Bool>("construction_active", 10);

    // ---- 0.1초마다: 시간 초과 확인 (영상이 끊겨도 멈추게) ----
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]
      {
        watchdog();
      });

    RCLCPP_INFO(get_logger(), "공사 구간 노드 시작: 공사 표지판 기다림");
  }

private:
  // ==================== 표지판 / 모드 ====================

  void on_sign(const Sign & sign)
  {
    const bool close_sign =
      sign.type == Sign::CONSTRUCTION &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= sign_area_ratio_threshold_;
    sign_hits_ = close_sign ? sign_hits_ + 1 : 0;

    // 공사 구간 중에 주차 표지판이 가까이 연속으로 보이면 -> 끝 (주차 노드가 이어받는다)
    const bool close_parking_sign =
      sign.type == Sign::PARKING &&
      sign.confidence >= min_confidence_ &&
      sign.area_ratio >= parking_sign_area_ratio_threshold_;
    parking_hits_ = close_parking_sign ? parking_hits_ + 1 : 0;
    if (state_ == State::AVOIDING && parking_hits_ >= parking_sign_hits_needed_)
    {
      finish("주차 표지판 인식", true);
      return;
    }

    if (state_ == State::WAITING && sign_hits_ >= sign_hits_needed_)
    {
      RCLCPP_INFO(get_logger(), "공사 표지판 인식 -> 장애물 피하기 모드");
      state_ = State::AVOIDING;
      zone_start_ = now();
      last_obstacle_ = now();
      obstacle_seen_ = false;
      last_image_ = now();
      publish_active(true);
    }
  }

  void on_mode(const ControlMode & msg)
  {
    if (msg.mission_state.rfind(MY_TAG, 0) == 0)
    {
      return;
    }
    const bool stop_request = msg.mode == ControlMode::STOP || msg.mode == ControlMode::MANUAL;
    if (state_ == State::AVOIDING && stop_request && msg.mission_state == "turtle_gui")
    {
      finish("GUI 에서 STOP/MANUAL", false);
    }
  }

  // ==================== 영상 한 장마다: 보고, 정하고, 움직이기 ====================

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (state_ != State::AVOIDING)
    {
      return;
    }
    last_image_ = now();

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

    const Scene scene = look(cv->image);           // 1) 보기
    if (scene.obstacle)
    {
      obstacle_seen_ = true;
      last_obstacle_ = now();
    }

    // (선택) 장애물이 한동안 안 보이면 공사 구간 끝. exit_clear_time = 0 이면 안 씀
    if (exit_clear_time_ > 0.0 && obstacle_seen_ &&
      (now() - last_obstacle_).seconds() >= exit_clear_time_)
    {
      finish("장애물이 더 이상 안 보임", true);
      return;
    }

    const double w = decide(scene, cv->image.cols, cv->image.rows);   // 2) 정하기
    drive(avoid_speed_, w);                                            // 3) 움직이기
  }

  // ---- 1) 보기: 색으로 노란선, 흰선, 박스를 찾는다 ----
  Scene look(const cv::Mat & bgr) const
  {
    const int w = bgr.cols;
    const int h = bgr.rows;
    const double robot_x = bev_center_x_ * w;

    cv::Mat lab;
    cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

    // 색 마스크 (해당 색이면 255, 아니면 0)
    cv::Mat yellow, white, box;
    cv::inRange(lab, cv::Scalar(yellow_l_min_, 0, yellow_b_min_),
      cv::Scalar(255, yellow_a_max_, 255), yellow);
    cv::inRange(lab, cv::Scalar(white_l_min_, 128 - white_ab_dev_, 128 - white_ab_dev_),
      cv::Scalar(255, 128 + white_ab_dev_, 128 + white_ab_dev_), white);
    cv::inRange(lab, cv::Scalar(box_l_min_, box_a_min_, box_b_min_),
      cv::Scalar(box_l_max_, box_a_max_, box_b_max_), box);

    // 빨간 정지선은 박스에서 뺀다 (b - a 가 너무 작으면 빨강)
    std::vector<cv::Mat> ch;
    cv::split(lab, ch);
    cv::Mat b_minus_a;
    cv::subtract(ch[2], ch[1], b_minus_a, cv::noArray(), CV_32S);
    cv::Mat red = b_minus_a < static_cast<double>(box_min_b_minus_a_);
    box.setTo(0, red);

    // 잡음 지우기
    const cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(yellow, yellow, cv::MORPH_OPEN, k3);
    cv::morphologyEx(white, white, cv::MORPH_OPEN, k3);
    cv::morphologyEx(box, box, cv::MORPH_OPEN, k3);

    Scene s;
    s.ref_y = static_cast<int>(lane_ref_ratio_ * (h - 1));

    // 장애물: 충분히 큰 갈색 덩어리 중 로봇에 가장 가까운 것 (아래 끝이 가장 아래인 것)
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(box, labels, stats, centroids, 8, CV_32S);
    int best_bottom = -1;
    for (int i = 1; i < n; i++)
    {
      if (stats.at<int>(i, cv::CC_STAT_AREA) < box_min_area_)
      {
        continue;
      }
      const int x = stats.at<int>(i, cv::CC_STAT_LEFT);
      const int bw = stats.at<int>(i, cv::CC_STAT_WIDTH);
      const int bottom = stats.at<int>(i, cv::CC_STAT_TOP) + stats.at<int>(i, cv::CC_STAT_HEIGHT) - 1;
      if (bottom > best_bottom)
      {
        best_bottom = bottom;
        s.obstacle = true;
        s.obs_x0 = x;
        s.obs_x1 = x + bw - 1;
      }
    }
    if (s.obstacle)
    {
      s.ref_y = best_bottom;      // 장애물 바로 옆 높이에서 차선 위치를 잰다
    }

    // 차선: 기준 줄 위아래 line_band_px 안의 픽셀 가로 위치 중간값
    //   노란선은 로봇보다 왼쪽(+반 차선)까지만, 흰선은 로봇보다 오른쪽(-반 차선)부터 찾는다
    const double half_lane = lane_width_px_ / 2.0;
    s.left_x = line_x(yellow, s.ref_y, 0, static_cast<int>(robot_x + half_lane));
    s.right_x = line_x(white, s.ref_y, static_cast<int>(robot_x - half_lane), w);
    return s;
  }

  // 마스크에서 y0 근처 가로줄 띠 안, x 범위 [x0, x1) 의 픽셀 가로 위치 중간값. 너무 적으면 없음
  std::optional<double> line_x(const cv::Mat & mask, int y0, int x0, int x1) const
  {
    x0 = std::clamp(x0, 0, mask.cols);
    x1 = std::clamp(x1, 0, mask.cols);
    const int top = std::clamp(y0 - static_cast<int>(line_band_px_), 0, mask.rows);
    const int bottom = std::clamp(y0 + static_cast<int>(line_band_px_), 0, mask.rows);
    if (x1 <= x0 || bottom <= top)
    {
      return std::nullopt;
    }
    std::vector<cv::Point> pts;
    cv::findNonZero(mask(cv::Rect(x0, top, x1 - x0, bottom - top)), pts);
    if (static_cast<int64_t>(pts.size()) < line_min_pixels_)
    {
      return std::nullopt;
    }
    std::vector<int> xs;
    xs.reserve(pts.size());
    for (const auto & p : pts)
    {
      xs.push_back(p.x + x0);
    }
    std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
    return static_cast<double>(xs[xs.size() / 2]);
  }

  // ---- 2) 정하기: 어디로 갈지 -> 회전 속도 w ----
  double decide(const Scene & s, int w, int h)
  {
    const double robot_x = bev_center_x_ * w;

    // 차선이 하나도 안 보이면 직진
    if (!s.left_x && !s.right_x)
    {
      log_throttled("차선 없음 -> 직진", s, robot_x);
      return 0.0;
    }

    // 한쪽만 보이면 다른 쪽은 차선 폭만큼 떨어져 있다고 본다
    const double left = s.left_x ? *s.left_x : *s.right_x - lane_width_px_;
    const double right = s.right_x ? *s.right_x : *s.left_x + lane_width_px_;

    double target;
    if (s.obstacle)
    {
      // 장애물 양옆 빈 공간 중 넓은 쪽 가운데로 (= 장애물에 붙은 차선에서 멀어지는 쪽)
      const double gap_left = s.obs_x0 - left;      // 왼쪽 차선 ~ 장애물 왼쪽 끝
      const double gap_right = right - s.obs_x1;    // 장애물 오른쪽 끝 ~ 오른쪽 차선
      if (gap_left >= gap_right)
      {
        target = (left + s.obs_x0) / 2.0;
      }
      else
      {
        target = (s.obs_x1 + right) / 2.0;
      }
    }
    else
    {
      target = (left + right) / 2.0;                // 장애물이 없으면 차선 가운데
    }

    // 로봇 중심은 항상 두 차선 안쪽 (각 차선에서 line_keep_px 이상 떨어지게)
    const double lo = left + line_keep_px_;
    const double hi = right - line_keep_px_;
    if (lo <= hi)
    {
      target = std::clamp(target, lo, hi);
    }
    else
    {
      target = (left + right) / 2.0;
    }

    // 목표점까지 각도 -> 회전 속도. 목표가 오른쪽(+)이면 오른쪽으로 돈다(w < 0)
    const double dx = target - robot_x;
    const double ahead = (h - 1 - s.ref_y) + robot_y_offset_px_;   // 바퀴 축에서 목표 줄까지 [px]
    const double angle = std::atan2(dx, ahead);
    const double w_cmd = std::clamp(-steer_gain_ * angle, -max_angular_, max_angular_);

    log_throttled(s.obstacle ? "장애물 회피" : "차선 가운데", s, target);
    return w_cmd;
  }

  // ---- 3) 움직이기 ----
  void drive(double v, double w)
  {
    if (!use_task_planner_)
    {
      publish_mode(ControlMode::CONSTRUCTION, "avoid");
    }
    // 안전: 앞이 너무 가까우면 멈춤
    if (front_stop_distance_ > 0.0 && psd_received_ &&
      psd_.front > 0.0 && psd_.front < front_stop_distance_)
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "앞이 막힘 -> 정지");
      return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  // ==================== 시간 초과 / 끝 ====================

  void watchdog()
  {
    if (state_ != State::AVOIDING)
    {
      return;
    }
    if (zone_timeout_ > 0.0 && (now() - zone_start_).seconds() >= zone_timeout_)
    {
      finish("공사 구간 시간 초과", true);
      return;
    }
    if ((now() - last_image_).seconds() >= image_timeout_)
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist());   // 영상이 끊기면 멈춤
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "영상(%s) 없음 -> 정지",
        "image_bev");
    }
  }

  // ok = true 면 정상 종료 (라인 주행으로 돌려줌), false 면 포기 (모드는 GUI 가 정한 대로)
  void finish(const char * reason, bool ok)
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    state_ = State::FINISHED;
    publish_active(false);
    if (ok && !use_task_planner_)
    {
      publish_mode(ControlMode::LANE, "done");
    }
    RCLCPP_INFO(get_logger(), "공사 구간 끝: %s", reason);
  }

  // ==================== 도우미 ====================

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

  void log_throttled(const char * what, const Scene & s, double target)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "%s: 노란선 %s 흰선 %s 장애물 %s -> 목표 x=%.0f",
      what,
      s.left_x ? std::to_string(static_cast<int>(*s.left_x)).c_str() : "-",
      s.right_x ? std::to_string(static_cast<int>(*s.right_x)).c_str() : "-",
      s.obstacle ? (std::to_string(s.obs_x0) + "~" + std::to_string(s.obs_x1)).c_str() : "-",
      target);
  }

  // ==================== 변수 ====================

  bool use_task_planner_;
  double min_confidence_;
  double sign_area_ratio_threshold_;
  int64_t sign_hits_needed_;
  double parking_sign_area_ratio_threshold_;
  int64_t parking_sign_hits_needed_;
  int64_t yellow_l_min_, yellow_b_min_, yellow_a_max_;
  int64_t white_l_min_, white_ab_dev_;
  int64_t box_l_min_, box_l_max_, box_a_min_, box_a_max_, box_b_min_, box_b_max_;
  int64_t box_min_b_minus_a_, box_min_area_;
  double bev_center_x_;
  double lane_width_px_;
  double robot_y_offset_px_;
  int64_t line_band_px_;
  int64_t line_min_pixels_;
  double lane_ref_ratio_;
  double line_keep_px_;
  double avoid_speed_;
  double steer_gain_;
  double max_angular_;
  double exit_clear_time_;
  double zone_timeout_;
  double image_timeout_;
  double front_stop_distance_;

  State state_ = State::WAITING;
  int64_t sign_hits_ = 0;
  int64_t parking_hits_ = 0;
  bool obstacle_seen_ = false;
  rclcpp::Time zone_start_;
  rclcpp::Time last_obstacle_;
  rclcpp::Time last_image_;
  interfaces::msg::PsdArray psd_;
  bool psd_received_ = false;

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
  rclcpp::spin(std::make_shared<ConstructionNode>());
  rclcpp::shutdown();
  return 0;
}

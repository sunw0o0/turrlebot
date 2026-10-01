#!/usr/bin/env python3
"""
ROS 2 노드: 차선 정보 (/lane_info) -> 속도 명령 (/cmd_vel)

  팀 저장소의 lane_detection 노드가 보내는 interfaces/LaneInfo 를 받아서
  차선 가운데를 따라가도록 /cmd_vel 을 만든다.
  /cmd_vel 은 stm_bridge (팀 C++ 또는 stm32_bridge.py) 가 받아서
  왼쪽/오른쪽 바퀴 속도로 바꿔 STM32 에 보낸다.

  카메라 -> bird_eye_view -> lane_detection -> [lane_follower] -> stm_bridge -> STM32 -> 바퀴

LaneInfo:
  offset : 차선 중심이 로봇보다 오른쪽이면 + (반 차선 = 1.0)
  angle  : 차선이 오른쪽으로 휘면 + (rad)
cmd_vel:
  angular.z 는 왼쪽 회전이 + 이므로, 차선이 오른쪽에 있으면 - (오른쪽으로 돈다)

제어 (PD):
  w = -(kp_offset * offset + kd_offset * offset 변화율 + kp_angle * angle)
  v = base_speed, 많이 꺾을수록 느리게

실행 (팀 워크스페이스를 source 해야 interfaces 메시지를 쓸 수 있다):
  source /opt/ros/jazzy/setup.bash
  source ~/Robit_intelligence_turtlebot_team/colcon_ws/install/setup.bash
  python3 lane_follower.py
  python3 lane_follower.py --ros-args -p base_speed:=0.05 -p kp_offset:=0.6
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from interfaces.msg import LaneInfo
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def clamp(value, limit):
    return max(-limit, min(limit, value))


class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")
        p = self.declare_parameter
        self.base_speed = p("base_speed", 0.08).value       # 직선에서 속도 (m/s)
        self.min_speed = p("min_speed", 0.03).value         # 크게 꺾을 때 최소 속도 (m/s)
        self.kp_offset = p("kp_offset", 0.8).value          # 옆으로 벗어난 만큼 돌기 (rad/s / offset)
        self.kd_offset = p("kd_offset", 0.05).value         # 벗어나는 속도만큼 미리 돌기 (흔들림 줄임)
        self.kp_angle = p("kp_angle", 1.0).value            # 차선이 휜 만큼 돌기 (rad/s / rad)
        self.max_angular = p("max_angular", 1.2).value      # 회전 최대 (rad/s)
        self.min_confidence = p("min_confidence", 0.25).value
        self.lost_timeout = p("lost_timeout", 0.5).value    # 차선을 이만큼 못 보면 정지 (s)
        self.rate = p("rate", 20.0).value                   # cmd_vel 보내는 주기 (Hz)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaneInfo, "lane_info", self.on_lane, qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.on_timer)

        self.offset = 0.0
        self.angle = 0.0
        self.prev_offset = None
        self.prev_time = None
        self.d_offset = 0.0
        self.last_seen = 0.0           # 마지막으로 차선을 제대로 본 시각
        self.was_driving = False
        self.get_logger().info(
            f"lane_info 기다리는 중 (base_speed={self.base_speed} m/s, "
            f"kp_offset={self.kp_offset}, kp_angle={self.kp_angle})")

    def on_lane(self, msg):
        if not msg.detected or msg.confidence < self.min_confidence:
            return                     # 못 봤으면 무시 -> 시간이 지나면 on_timer 가 정지
        now = time.monotonic()
        if self.prev_offset is not None and now - self.prev_time > 1e-3:
            raw = (msg.offset - self.prev_offset) / (now - self.prev_time)
            self.d_offset = 0.7 * self.d_offset + 0.3 * raw   # 잡음 줄이기
        self.prev_offset = msg.offset
        self.prev_time = now
        self.offset = msg.offset
        self.angle = msg.angle
        self.last_seen = now

    def on_timer(self):
        cmd = Twist()
        driving = time.monotonic() - self.last_seen < self.lost_timeout
        if driving:
            w = -(self.kp_offset * self.offset
                  + self.kd_offset * self.d_offset
                  + self.kp_angle * self.angle)
            w = clamp(w, self.max_angular)
            # 많이 꺾을수록 천천히 (커브에서 차선 놓치지 않게)
            slow = 1.0 - abs(w) / self.max_angular
            v = self.min_speed + (self.base_speed - self.min_speed) * slow
            cmd.linear.x = v
            cmd.angular.z = w
        else:
            self.prev_offset = None    # 다시 찾으면 미분을 새로 시작
            self.d_offset = 0.0
        self.cmd_pub.publish(cmd)

        if driving != self.was_driving:
            self.was_driving = driving
            if driving:
                self.get_logger().info("차선 찾음 -> 주행")
            else:
                self.get_logger().warn("차선 놓침 -> 정지")

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

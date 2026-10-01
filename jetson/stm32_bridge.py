#!/usr/bin/env python3
"""
ROS 2 노드: /cmd_vel (geometry_msgs/Twist) -> STM32

  - /cmd_vel 의 linear.x (m/s), angular.z (rad/s) 를 왼쪽/오른쪽 바퀴 속도로 바꿔
    20Hz 로 STM32 에 보낸다. (팀 C++ stm 패키지와 같은 일. 빌드 없이 쓰는 파이썬 버전)
  - /cmd_vel 이 0.5초 넘게 안 오면 0 을 보낸다 (STM32 도 0.3초 끊기면 스스로 정지).
  - STM32 상태 패킷은 상태가 바뀔 때 로그로 출력한다.

실행:
  source /opt/ros/jazzy/setup.bash
  python3 stm32_bridge.py                      (기본 /dev/ttyUSB0)
  python3 stm32_bridge.py --ros-args -p port:=/dev/ttyUSB1

테스트 (다른 터미널):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from stm32_link import Stm32Link


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__("stm32_bridge")
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        self.timeout = self.declare_parameter("cmd_timeout", 0.5).value
        separation = self.declare_parameter("wheel_separation", 0.160).value

        self.link = Stm32Link(port, separation=separation)
        self.v = 0.0
        self.w = 0.0
        self.last_cmd = 0.0
        self.last_state = None
        self.was_connected = True

        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(0.05, self.on_timer)      # 20Hz
        self.get_logger().info(f"{port} 열림. /cmd_vel 기다리는 중")

    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = time.monotonic()

    def on_timer(self):
        if time.monotonic() - self.last_cmd > self.timeout:
            self.v = 0.0
            self.w = 0.0
        self.link.send_vel(self.v, self.w)

        if self.link.connected != self.was_connected:
            self.was_connected = self.link.connected
            if self.link.connected:
                self.get_logger().info(f"USB 다시 연결됨 (재연결 {self.link.reconnects}번)")
            else:
                self.get_logger().warn("USB 끊김 -> 다시 연결 기다리는 중")

        for st in self.link.read_status():
            key = (st["ready"], st["state"], st["sw"], st["hw_err"])
            if key != self.last_state:
                self.last_state = key
                self.get_logger().info(
                    f"STM32: {st['state_name']}, 준비={st['ready']}, "
                    f"스위치={st['sw']:04b}, 에러={st['hw_err']}, 전압={st['volt']:.1f}V")

    def destroy_node(self):
        self.link.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
오도메트리(엔코더로 잰 이동 거리 / 회전 각도) 확인 도구

stm_bridge 가 보내는 /odom 을 보고 로봇을 움직인다.
  - show            : 움직이지 않고, 지금까지 간 거리와 돈 각도를 계속 보여준다 (GUI 로 움직이면서 보기)
  - forward 1.0     : 엔코더 기준으로 1.0 m 직진하고 멈춘다 -> 줄자로 실제 거리를 잰다
  - turn 360        : 엔코더 기준으로 왼쪽으로 360도 돌고 멈춘다 (-360 = 오른쪽) -> 처음 방향과 비교

실행 (젯슨, stm_bridge 가 켜져 있어야 함):
  python3 ~/turrlebot/team_patch/odom_test.py show
  python3 ~/turrlebot/team_patch/odom_test.py forward 1.0
  python3 ~/turrlebot/team_patch/odom_test.py turn 360

※ 이 스크립트가 움직이는 동안에는 다른 노드(lane_follower 등)가 cmd_vel 을 보내면 안 된다.
   stm_bridge 만 켜고 시험하기. Ctrl+C 를 누르면 바로 멈춘다.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

LINEAR_SPEED = 0.08     # [m/s] 직진 속도
ANGULAR_SPEED = 0.5     # [rad/s] 회전 속도
SLOW_DOWN = 0.05        # 목표의 마지막 5% 는 천천히 (지나치지 않게)


def yaw_from_quaternion(q):
    """쿼터니언(odom 의 방향 표현) -> 방향 각도 [rad]. z 축(바닥에서 도는 방향)만 본다"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    """a - b 를 -pi ~ pi 사이로 (예: 179도 - (-179도) = -2도)"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class OdomTest(Node):
    def __init__(self, mode, target):
        super().__init__('odom_test')
        self.mode = mode            # 'show', 'forward', 'turn'
        self.target = target        # forward: [m], turn: [도]

        self.start = None           # 시작할 때의 (x, y, 방향)
        self.distance = 0.0         # 시작부터 간 거리 [m] (앞 +, 뒤 -)
        self.turned = 0.0           # 시작부터 돈 각도 [rad] (왼쪽 +). 360도 넘게 돌아도 계속 더한다
        self.last_xy = None
        self.last_yaw = None
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.05, self.step)     # 0.05초마다 바퀴 명령
        self.create_timer(0.5, self.print_now)  # 0.5초마다 화면 출력
        self.get_logger().info('odom 기다리는 중... (stm_bridge 가 켜져 있고 S1 이 켜져 있어야 함)')

    # odom 이 올 때마다 거리 / 각도 누적
    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.start is None:
            self.start = (p.x, p.y, yaw)
            self.get_logger().info('odom 받음 -> 시작')
        else:
            # 앞뒤 방향을 고려한 이동 거리: 이동량을 로봇 방향으로 투영
            dx = p.x - self.last_xy[0]
            dy = p.y - self.last_xy[1]
            self.distance += dx * math.cos(self.last_yaw) + dy * math.sin(self.last_yaw)
            self.turned += angle_diff(yaw, self.last_yaw)
        self.last_xy = (p.x, p.y)
        self.last_yaw = yaw

    # 목표까지 남은 비율에 따라 속도 정하기 (끝에서 천천히)
    def speed_for(self, done_amount, goal, full_speed):
        remain = abs(goal) - abs(done_amount)
        if remain <= 0.0:
            return 0.0
        slow = full_speed * 0.3 if remain < abs(goal) * SLOW_DOWN else full_speed
        return math.copysign(slow, goal)

    def step(self):
        if self.start is None or self.done or self.mode == 'show':
            return
        cmd = Twist()
        if self.mode == 'forward':
            cmd.linear.x = self.speed_for(self.distance, self.target, LINEAR_SPEED)
            finished = cmd.linear.x == 0.0
        else:
            goal = math.radians(self.target)
            cmd.angular.z = self.speed_for(self.turned, goal, ANGULAR_SPEED)
            finished = cmd.angular.z == 0.0
        self.cmd_pub.publish(cmd)
        if finished:
            self.done = True
            self.print_now()
            self.print_result()

    def print_now(self):
        if self.start is None:
            return
        print('  간 거리 %+.3f m   돈 각도 %+7.1f 도' % (self.distance, math.degrees(self.turned)))

    def print_result(self):
        print()
        if self.mode == 'forward':
            print('엔코더 기준 %.2f m 갔어요. 줄자로 실제 거리를 재 보세요.' % self.target)
            print('  실제가 더 길면  -> wheel_radius 를 (실제 / %.2f) 배로 키우기' % self.target)
            print('  실제가 더 짧으면 -> wheel_radius 를 (실제 / %.2f) 배로 줄이기' % self.target)
            print('  예) 1.00 m 명령에 실제 1.05 m -> wheel_radius x 1.05')
        else:
            print('엔코더 기준 %.0f 도 돌았어요. 처음 방향과 비교해 보세요.' % self.target)
            print('  덜 돌았으면 (실제 < 명령) -> wheel_separation 을 (명령 / 실제) 배로 키우기')
            print('  더 돌았으면 (실제 > 명령) -> wheel_separation 을 (명령 / 실제) 배로 줄이기')
            print('  예) 360 도 명령에 실제 340 도 -> wheel_separation x 360/340')
        print('값은 colcon_ws/src/stm/config/stm_bridge.yaml 에서 바꾸고 stm 다시 빌드')
        print('끝내려면 Ctrl+C')

    def stop(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass               # 이미 종료 중이면 무시 (stm_bridge 가 0.5초 뒤 스스로 멈춘다)


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('show', 'forward', 'turn'):
        print(__doc__)
        return
    mode = sys.argv[1]
    target = float(sys.argv[2]) if len(sys.argv) > 2 else (1.0 if mode == 'forward' else 360.0)

    rclpy.init()
    node = OdomTest(mode, target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()            # Ctrl+C 를 눌러도 꼭 멈추기
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

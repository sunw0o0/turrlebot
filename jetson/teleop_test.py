#!/usr/bin/env python3
"""
ROS 없이 키보드로 STM32 에 속도 명령 보내기 (통신 테스트용)

  w / s : 앞 / 뒤 (선속도 +-0.05 m/s 씩)
  a / d : 왼쪽 / 오른쪽 회전 (각속도 +-0.2 rad/s 씩)
  스페이스 : 정지
  q : 종료

STM32 스위치: S1 켜기, S2 끄기 (그래야 ROS 명령대로 움직인다)

사용법:  python3 teleop_test.py               (기본 /dev/ttyUSB0)
         python3 teleop_test.py /dev/ttyUSB1
"""
import select
import sys
import termios
import time
import tty

import serial

from stm32_link import Stm32Link

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
link = Stm32Link(port)

v = 0.0
w = 0.0
status = None
old = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
print(__doc__)
try:
    last_print = 0.0
    while True:
        # 키 입력 (없으면 기다리지 않고 넘어감)
        if select.select([sys.stdin], [], [], 0.05)[0]:
            key = sys.stdin.read(1)
            if key == "w":
                v += 0.05
            elif key == "s":
                v -= 0.05
            elif key == "a":
                w += 0.2
            elif key == "d":
                w -= 0.2
            elif key == " ":
                v, w = 0.0, 0.0
            elif key == "q":
                break
            v = max(-0.3, min(0.3, round(v, 2)))
            w = max(-1.5, min(1.5, round(w, 1)))

        # 멈춰 있어도 계속 보낸다 (0.3초 끊기면 STM32 가 정지)
        try:
            link.send_vel(v, w)
        except serial.SerialException:
            print("USB-TTL 연결이 끊겼습니다 (ls /dev/ttyUSB* 로 확인)")
            break

        for st in link.read_status():
            status = st
        if time.time() - last_print > 0.5:
            last_print = time.time()
            if status:
                print(f"보냄 v={v:+.2f} m/s w={w:+.1f} rad/s | "
                      f"STM32: {status['state_name']}, 준비={status['ready']}, "
                      f"스위치={status['sw']:04b}, 에러={status['hw_err']}, "
                      f"전압={status['volt']:.1f}V")
            else:
                print(f"보냄 v={v:+.2f} w={w:+.1f} | STM32 상태 패킷 아직 없음 (배선/포트 확인)")
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    link.close()
    print("정지 후 종료")

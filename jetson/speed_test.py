#!/usr/bin/env python3
"""
바퀴 속도 시험: 목표 속도를 단계별로 올리면서 두 모터의 실제 속도(rpm)를 잰다.

  - 바퀴를 바닥에서 띄우고 실행할 것! (로봇이 달려나감)
  - STM32 스위치: S1 켜기, S2 끄기
  - ROS 브리지 / teleop_test 는 끄고 실행 (포트는 한 프로그램만)

각 단계에서 2.5초 동안 같은 속도를 보내고, 마지막 1초의 실제 속도를 평균 낸다.
펌웨어 최대는 바퀴 0.30 m/s (MX-64 값 250 = 약 57 rpm). 그 위로 보내도 250 으로 잘린다.

사용법:  python3 speed_test.py               (기본 /dev/ttyUSB0)
         python3 speed_test.py /dev/ttyUSB1
"""
import sys
import time

from stm32_link import Stm32Link

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
link = Stm32Link(port)

STEPS = [0.05, 0.10, 0.15, 0.20, 0.25, 0.26, 0.28, 0.30]   # 바퀴 선속도 [m/s]
HOLD = 2.5      # 단계마다 유지 시간 [s]
AVERAGE = 1.0   # 마지막 몇 초를 평균낼지 [s]

print(__doc__)
input("바퀴를 띄웠으면 Enter (중단: Ctrl+C) ")
print(f"{'바퀴 m/s':>8} | {'목표 rpm':>8} | {'왼쪽 실제':>9} {'(%)':>6} | {'오른쪽 실제':>10} {'(%)':>6} | 에러")

try:
    for speed in STEPS:
        start = time.time()
        samples = []
        last = None
        while time.time() - start < HOLD:
            link.send_wheels(speed, speed)             # 0.05초마다 계속 보내야 멈추지 않음
            for st in link.read_status():
                last = st
                if time.time() - start > HOLD - AVERAGE and st.get("vel_rpm"):
                    samples.append(st["vel_rpm"])
            time.sleep(0.05)

        if not last or not last.get("vel_rpm"):
            print("상태 패킷에 속도가 없음 -> 펌웨어가 최신인지 확인")
            break
        if not last["ready"]:
            print(f"{speed:8.2f} | STM32 준비 안 됨 ({last['state_name']}, 모터수={last['count']})")
            continue
        goal = last["goal_rpm"][0]
        left = sum(s[0] for s in samples) / max(len(samples), 1)
        right = sum(s[1] for s in samples) / max(len(samples), 1)

        def pct(v):
            return f"{100 * v / goal:5.0f}%" if goal else "   -  "

        print(f"{speed:8.2f} | {goal:8.1f} | {left:9.1f} {pct(left)} | {right:10.1f} {pct(right)} | "
              f"{last['hw_err']}  ({last['state_name']})")
except KeyboardInterrupt:
    pass
finally:
    link.close()                                       # 정지 명령 보내고 닫기
    print("정지")

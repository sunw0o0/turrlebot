#!/usr/bin/env python3
"""
PSD 거리 센서 3개 확인 (ROS 없이)

STM32 가 0.05초마다 보내는 PSD 패킷(0x10)을 받아 막대로 보여 준다.
손이나 종이를 센서 앞 10 ~ 80 cm 에서 움직여 보면서 값이 바뀌는지 확인한다.

사용법:  python3 psd_test.py               (기본 /dev/ttyUSB0)
         python3 psd_test.py /dev/ttyUSB1
"""
import sys
import time

from stm32_link import Stm32Link

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
link = Stm32Link(port)
print(f"{port} 열림. Ctrl+C 로 종료 (가까울수록 막대가 짧음, 0.80m = 아무것도 없음)")


def bar(meters):
    return "#" * int(meters * 40)      # 0.80m -> 32칸


try:
    status_count = 0
    last_print = 0.0
    while True:
        if link.read_status():         # 상태 + PSD 패킷 받기 (PSD 는 link.psd 에 저장)
            status_count += 1
        if link.psd and time.time() - last_print > 0.2:
            last_print = time.time()
            p = link.psd
            print(f"왼쪽 {p['left']:.2f}m {bar(p['left']):<32} | "
                  f"앞 {p['front']:.2f}m {bar(p['front']):<32} | "
                  f"오른쪽 {p['right']:.2f}m {bar(p['right'])}")
        elif not link.psd and time.time() - last_print > 1.0:
            last_print = time.time()
            if status_count:
                print(f"상태 패킷은 옴 ({status_count}개) -> STM32 는 동작 중. "
                      "PSD 패킷만 없음 = 보드에 PSD 펌웨어(ed25f17)가 아직 안 올라감")
            else:
                print("아무 패킷도 없음 -> 포트를 다른 프로그램(ROS 브리지 등)이 쓰는지, "
                      "STM32 가 멈췄는지(리셋 버튼), 배선 확인")
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    link.ser.close()

#!/usr/bin/env python3
"""
젯슨 <-> STM32 (USART6) 에코 테스트

STM32 는 받은 바이트를 그대로 돌려보낸다.
보낸 문자열이 그대로 돌아오면 배선 / 포트 / 통신 속도가 맞는 것이다.

사용법:  python3 echo_test.py            (기본 /dev/ttyTHS1)
         python3 echo_test.py /dev/ttyTHS2
"""
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
ser = serial.Serial(port, 115200, timeout=0.5)
ser.reset_input_buffer()
print(f"{port} 열림. Ctrl+C 로 종료")

ok = 0
fail = 0
try:
    n = 0
    while True:
        msg = f"hello {n}\n".encode()
        ser.write(msg)
        back = ser.read(len(msg))
        if back == msg:
            ok += 1
            print(f"OK   {back!r}")
        else:
            fail += 1
            print(f"FAIL 보냄 {msg!r} / 받음 {back!r}")
        n += 1
        time.sleep(0.5)
except KeyboardInterrupt:
    print(f"\n성공 {ok}, 실패 {fail}")
finally:
    ser.close()

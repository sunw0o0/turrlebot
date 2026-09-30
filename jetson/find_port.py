#!/usr/bin/env python3
"""
젯슨 <-> STM32 (USART6) 연결 포트 찾기

1) 15초 동안 ttyTHS1, ttyTHS2 를 동시에 듣는다.
   이때 STM32 리셋 버튼을 누르면 STM32 가 "STM32 ready" 를 보낸다.
   -> 이 글자가 들어온 포트가 STM32 -> 젯슨 방향으로 연결된 포트
2) 각 포트로 "ping" 을 보내고 STM32 가 그대로 돌려보내는지(에코) 본다.
   -> 돌아오면 양방향 통신 성공

사용법: python3 find_port.py
"""
import time
import serial

PORTS = ["/dev/ttyTHS1", "/dev/ttyTHS2"]
BAUD = 115200

opened = {}
for p in PORTS:
    try:
        opened[p] = serial.Serial(p, BAUD, timeout=0)
        opened[p].reset_input_buffer()
    except Exception as e:
        print(f"{p}: 열기 실패 ({e})")

if not opened:
    raise SystemExit("열 수 있는 포트가 없습니다 (권한: dialout 그룹 확인)")

print("=== 1단계: 15초 동안 듣는 중. 지금 STM32 리셋 버튼을 누르세요 ===")
got = {p: b"" for p in opened}
end = time.time() + 15
while time.time() < end:
    for p, s in opened.items():
        data = s.read(256)
        if data:
            got[p] += data
            print(f"  {p} 에서 받음: {data!r}")
    time.sleep(0.01)

for p in opened:
    if got[p]:
        print(f"[1단계] {p}: 받음 {got[p]!r}  -> STM32 -> 젯슨 방향 OK")
    else:
        print(f"[1단계] {p}: 아무것도 못 받음")

print("\n=== 2단계: 각 포트로 ping 을 보내고 에코 확인 ===")
for p, s in opened.items():
    s.reset_input_buffer()
    msg = b"ping\n"
    s.write(msg)
    back = b""
    end = time.time() + 1.0
    while time.time() < end and len(back) < len(msg):
        back += s.read(len(msg) - len(back))
        time.sleep(0.01)
    if back == msg:
        print(f"[2단계] {p}: 에코 성공 {back!r}  -> 양방향 통신 OK. 이 포트를 쓰세요")
    else:
        print(f"[2단계] {p}: 에코 실패 (받음 {back!r})")

for s in opened.values():
    s.close()

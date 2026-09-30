"""
stm32_link.py - 젯슨 <-> STM32 패킷 주고받기 (App/proto.h 와 같은 규칙)

  [0xAA] [0x55] [CMD] [LEN] [DATA ...] [CHK]
  CHK = (CMD + LEN + DATA) 합의 아래 8비트
"""
import struct
import serial

HEAD1 = 0xAA
HEAD2 = 0x55
CMD_VEL = 0x01
CMD_STATUS = 0x81

STATE_NAMES = {
    0: "STOP_SW (S1 꺼짐)",
    1: "ROS 주행",
    2: "ESTOP (S2 켜짐)",
    3: "NO_CMD (명령 끊김)",
    4: "NOT_READY (모터 준비 안 됨)",
    5: "MANUAL (스위치 테스트)",
}


def build_packet(cmd, data):
    chk = (cmd + len(data) + sum(data)) & 0xFF
    return bytes([HEAD1, HEAD2, cmd, len(data)]) + bytes(data) + bytes([chk])


def vel_packet(linear_mps, angular_rps):
    """선속도 (m/s), 각속도 (rad/s) -> 속도 명령 패킷"""
    lin = max(-32768, min(32767, int(round(linear_mps * 1000))))
    ang = max(-32768, min(32767, int(round(angular_rps * 1000))))
    return build_packet(CMD_VEL, struct.pack("<hh", lin, ang))


class Stm32Link:
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0)
        self.buf = bytearray()

    def send_vel(self, linear_mps, angular_rps):
        self.ser.write(vel_packet(linear_mps, angular_rps))

    def read_status(self):
        """받은 상태 패킷들을 dict 리스트로 돌려준다 (없으면 빈 리스트)"""
        self.buf += self.ser.read(self.ser.in_waiting or 1)
        out = []
        while True:
            i = self.buf.find(bytes([HEAD1, HEAD2]))
            if i < 0:
                del self.buf[:-1]          # 마지막 0xAA 는 남겨 둔다
                return out
            del self.buf[:i]
            if len(self.buf) < 4:
                return out
            length = self.buf[3]
            if len(self.buf) < 5 + length:
                return out
            cmd = self.buf[2]
            data = bytes(self.buf[4:4 + length])
            chk = self.buf[4 + length]
            if (cmd + length + sum(data)) & 0xFF != chk:
                del self.buf[:1]           # 헤더가 가짜였음. 한 바이트 넘기고 다시
                continue
            del self.buf[:5 + length]
            if cmd == CMD_STATUS and length == 7:
                ready, state, sw, hw0, hw1, volt = struct.unpack("<BBBBBH", data)
                out.append({
                    "ready": ready,
                    "state": state,
                    "state_name": STATE_NAMES.get(state, str(state)),
                    "sw": sw,
                    "hw_err": (hw0, hw1),
                    "volt": volt / 10.0,
                })

    def close(self):
        self.send_vel(0.0, 0.0)
        self.ser.close()

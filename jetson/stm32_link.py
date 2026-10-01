"""
stm32_link.py - 젯슨 <-> STM32 패킷 주고받기 (App/proto.h, 팀 stm/protocol.hpp 와 같은 규칙)

  [0xAA] [0x55] [CMD] [LEN] [DATA ...] [CHK]
  CHK = (CMD + LEN + DATA) 합의 아래 8비트

  젯슨 -> STM32 : CMD 0x01, 왼쪽 바퀴 mm/s (int16), 오른쪽 바퀴 mm/s (int16)
"""
import struct
import serial

HEAD1 = 0xAA
HEAD2 = 0x55
CMD_WHEEL = 0x01

# 좌우 바퀴 사이 거리 (m). 팀 stm_bridge.yaml 의 wheel_separation 과 같게. 실측 후 수정
WHEEL_SEPARATION = 0.160
CMD_STATUS = 0x81
CMD_PSD = 0x10

STATE_NAMES = {
    0: "STOP_SW (S1 꺼짐)",
    1: "주행 (젯슨 명령)",
    2: "ESTOP (S2 켜짐)",
    3: "NO_CMD (명령 끊김)",
    4: "NOT_READY (모터 준비 안 됨)",
    5: "MANUAL (스위치 테스트)",
}


def build_packet(cmd, data):
    chk = (cmd + len(data) + sum(data)) & 0xFF
    return bytes([HEAD1, HEAD2, cmd, len(data)]) + bytes(data) + bytes([chk])


def wheel_packet(left_mps, right_mps):
    """왼쪽/오른쪽 바퀴 선속도 (m/s) -> 바퀴 명령 패킷"""
    left = max(-32768, min(32767, int(round(left_mps * 1000))))
    right = max(-32768, min(32767, int(round(right_mps * 1000))))
    return build_packet(CMD_WHEEL, struct.pack("<hh", left, right))


def twist_to_wheels(linear_mps, angular_rps, separation=WHEEL_SEPARATION):
    """로봇 선속도 v, 각속도 w -> 왼쪽/오른쪽 바퀴 선속도 (차동 구동)"""
    left = linear_mps - angular_rps * separation / 2.0
    right = linear_mps + angular_rps * separation / 2.0
    return left, right


class Stm32Link:
    """
    USB 가 순간적으로 끊겨도 (잡음으로 USB 허브가 리셋되는 경우)
    프로그램이 죽지 않고 다시 연결될 때까지 조용히 재시도한다.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, separation=WHEEL_SEPARATION):
        self.separation = separation
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0)
        self.buf = bytearray()
        self.reconnects = 0          # 다시 연결한 횟수
        self.psd = None              # 마지막 PSD 거리 {"left", "front", "right"} [m]
        self.connected = True

    def _lost(self):
        """연결이 끊겼을 때: 포트를 닫고 다음에 다시 연다"""
        if self.connected:
            self.connected = False
            try:
                self.ser.close()
            except Exception:
                pass

    def _ensure_open(self):
        """끊겨 있으면 다시 열어 본다. 열려 있으면 True"""
        if self.connected:
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
        except (serial.SerialException, OSError):
            return False             # 아직 USB 가 안 돌아옴
        self.buf = bytearray()
        self.connected = True
        self.reconnects += 1
        return True

    def send_vel(self, linear_mps, angular_rps):
        """로봇 선속도 (m/s), 각속도 (rad/s) 로 명령 (바퀴 속도로 바꿔서 보냄)"""
        left, right = twist_to_wheels(linear_mps, angular_rps, self.separation)
        self.send_wheels(left, right)

    def send_wheels(self, left_mps, right_mps):
        """왼쪽/오른쪽 바퀴 선속도 (m/s) 로 명령"""
        if not self._ensure_open():
            return
        try:
            self.ser.write(wheel_packet(left_mps, right_mps))
        except (serial.SerialException, OSError):
            self._lost()

    def read_status(self):
        """받은 상태 패킷들을 dict 리스트로 돌려준다 (없으면 빈 리스트)"""
        if not self._ensure_open():
            return []
        try:
            self.buf += self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            self._lost()
            return []
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
            if cmd == CMD_PSD and length == 6:
                left, front, right = struct.unpack("<HHH", data)
                self.psd = {"left": left / 1000.0, "front": front / 1000.0,
                            "right": right / 1000.0}
                continue
            if cmd == CMD_STATUS and length >= 7:
                ready, state, sw, hw0, hw1, volt = struct.unpack("<BBBBBH", data[:7])
                count, id_l, id_r, torque = (data[7], data[8], data[9], data[10]) \
                    if length >= 11 else (None, None, None, None)
                out.append({
                    "count": count,
                    "ids": (id_l, id_r),
                    "torque": None if torque is None else (torque & 1, (torque >> 1) & 1),
                    "ready": ready,
                    "state": state,
                    "state_name": STATE_NAMES.get(state, str(state)),
                    "sw": sw,
                    "hw_err": (hw0, hw1),
                    "volt": volt / 10.0,
                })

    def close(self):
        self.send_vel(0.0, 0.0)         # 마지막으로 정지 명령
        try:
            self.ser.close()
        except Exception:
            pass

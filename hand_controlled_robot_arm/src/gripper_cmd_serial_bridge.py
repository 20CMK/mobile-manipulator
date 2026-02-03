#!/usr/bin/env python3
# gripper_cmd_serial_bridge.py
"""
/gripper_cmd(Int8) → USB-Serial 문자 송신
  · 1 ⇒ 'a\\n'  (OPEN)   · 0 ⇒ 'b\\n'  (CLOSE)
"""

import rospy, serial
from std_msgs.msg import Int8
from serial.tools import list_ports   # 자동 포트 탐색용(선택)

# ── 1. 시리얼 기본값(수정하면 끝) ────────────────────
PORT = "/dev/ttyACM1"    # 아두이노 포트, 바꿔쓰세요
BAUD = 115200

# ① 자동 탐색이 편하면 주석 해제
# cand = [p.device for p in list_ports.comports() if "USB" in p.device or "ACM" in p.device]
# if cand: PORT = cand[0]

class Bridge:
    def __init__(self):
        self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
        self.prev_cmd = None
        rospy.Subscriber("/gripper_cmd", Int8, self.cb)

    def cb(self, msg: Int8):
        cmd = b'a\n' if msg.data == 1 else b'b\n'
        if cmd == self.prev_cmd:
            return                        # 중복 송신 방지
        try:
            self.ser.write(cmd)
            self.prev_cmd = cmd
            rospy.loginfo(f"📤 {cmd.strip().decode()} sent")
        except serial.SerialException as e:
            rospy.logerr(f"Serial error: {e}")

    def spin(self):
        rospy.spin()
        self.ser.close()

if __name__ == "__main__":
    rospy.init_node("gripper_serial_bridge", anonymous=True)
    Bridge().spin()

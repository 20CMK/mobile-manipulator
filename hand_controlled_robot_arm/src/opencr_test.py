#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import sys, select, termios, tty

def get_key(timeout=0.0):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

def main():
    rospy.init_node("keyboard_to_opencr")

    port    = rospy.get_param("~port", "/dev/ttyACM0")
    baud    = rospy.get_param("~baud", 115200)
    newline = rospy.get_param("~newline", True)   # '\n' 붙일지 여부

    ser = serial.Serial(port, baud, timeout=0.01)

    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print("=== Keyboard → OpenCR ===")
    print("Press 'a' or 'b' to send. 'q' to quit.")
    print("port: {}, baud: {}".format(port, baud))

    rate = rospy.Rate(100)
    try:
        while not rospy.is_shutdown():
            k = get_key(0.0)
            if k is None:
                rate.sleep()
                continue

            if k in ('a', 'b'):
                ser.write(k.encode('ascii'))
                if newline:
                    ser.write(b'\n')
                rospy.loginfo("Sent '{}'".format(k))
            elif k == 'q':
                break
            # 그 외 키는 무시
            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        ser.close()
        print("Bye.")

if __name__ == "__main__":
    main()

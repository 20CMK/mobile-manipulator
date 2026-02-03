#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import math 

# 퍼블리셔
pub = rospy.Publisher('joy_cmd', Float32MultiArray, queue_size=10)

# 현재 스무스된 값과 목표 값 저장
filtered_axes = [0.0, 0.0, 0.0]
target_axes = [0.0, 0.0, 0.0]

# 스무딩 속도 (작을수록 느리게 따라감)
alpha = 0.05

def joy_cb(msg):
    global target_axes
    # 새 목표값 설정
    target_axes = [(-1) * msg.axes[3], (-1) * msg.axes[1], msg.buttons[1]]

def timer_cb(event):
    global filtered_axes, target_axes
    # 목표값 쪽으로 filtered 값을 천천히 이동
    for i in range(3):
        filtered_axes[i] += alpha * (target_axes[i] - filtered_axes[i])
    filtered_axes[-1] = target_axes[-1]
    filtered_axes[0] = round(filtered_axes[0], 3)
    # print(filtered_axes[0])
    filtered_axes[1] = round(filtered_axes[1], 3)
    
    # 퍼블리시
    out = Float32MultiArray()
    out.data = filtered_axes
    pub.publish(out)

rospy.init_node("joy_to_mcu")

rospy.Subscriber("/joy", Joy, joy_cb)
rospy.Timer(rospy.Duration(0.02), timer_cb)  # 50Hz 주기로 업데이트
rospy.spin()
#!/usr/bin/env python3
"""
Test용 임의 토픽 발행 코드. poseStamped형식의 /fusion_pose 발행. record 데이터가 없을 시 사용
"""
import rospy
import math
import random
from geometry_msgs.msg import PoseStamped

def clamp(val, vmin, vmax):
    return max(vmin, min(val, vmax))

def main():
    rospy.init_node("test_fusion_pose_publisher")
    pub = rospy.Publisher("/fusion_pose", PoseStamped, queue_size=10)
    rate = rospy.Rate(20)  # 10Hz (0.1초 간격)

    # 초기 pose
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    # 위치 범위
    x, y, z = 0.4, 0.0, 0.46
    x_min, x_max = 0.2, 0.6
    y_min, y_max = -0.4, 0.4
    z_min, z_max = 0.2, 0.6

    # 랜덤 속도 초기화 (부드러운 움직임을 위해)
    vx = random.uniform(-0.01, 0.01)
    vy = random.uniform(-0.01, 0.01)
    vz = random.uniform(-0.005, 0.005)

    rospy.loginfo("🎯 Random continuous motion publisher started.")
    while not rospy.is_shutdown():
        # 위치 갱신
        x += vx
        y += vy
        z += vz

        # 범위 제한 및 방향 반전
        if x < x_min or x > x_max:
            vx *= -1
        if y < y_min or y > y_max:
            vy *= -1
        if z < z_min or z > z_max:
            vz *= -1

        # 약간씩 랜덤하게 속도 변동 (더 자연스럽게)
        vx += random.uniform(-0.002, 0.002)
        vy += random.uniform(-0.002, 0.002)
        vz += random.uniform(-0.001, 0.001)
        vx = clamp(vx, -0.02, 0.02)
        vy = clamp(vy, -0.02, 0.02)
        vz = clamp(vz, -0.01, 0.01)

        # 메시지 구성 및 발행
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pub.publish(pose)

        rospy.loginfo_throttle(1.0, f"Publishing (x={x:.3f}, y={y:.3f}, z={z:.3f})")
        rate.sleep()

if __name__ == "__main__":
    main()

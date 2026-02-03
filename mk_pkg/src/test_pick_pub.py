#!/usr/bin/env python3
"""
/flag/pick_flag = True 수신 시
/fusion_pose 에 고정 Pose (0.4, 0, 0.4, 0, 0, 0, 1) 발행
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class FixedPosePublisher:
    def __init__(self):
        rospy.init_node('fixed_fusion_pose_publisher', anonymous=True)

        self.flag = False

        self.pose_pub = rospy.Publisher(
            '/fusion_pose', PoseStamped, queue_size=10
        )

        rospy.Subscriber(
            '/flag/pick_flag', Bool, self.flag_callback
        )

        rospy.loginfo("Fixed fusion_pose publisher ready")
        rospy.spin()

    def flag_callback(self, msg):
        if msg.data:
            rospy.loginfo("pick_flag received → publish fixed fusion_pose")
            self.publish_fixed_pose()

    def publish_fixed_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.position.x = 0.4
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.4

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)
        rospy.loginfo("fusion_pose published: [0.4, 0, 0.4]")


if __name__ == '__main__':
    try:
        FixedPosePublisher()
    except rospy.ROSInterruptException:
        pass

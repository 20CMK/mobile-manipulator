#!/usr/bin/env python3
"""
pick & place 작업을 위해 물체의 위치를 발행하는 코드
base와 camera 위치만큼 offset을 주어 물체 위치에 대한 /fusion_pose 발행
방향은 0으로 고정
/flag/pick_flag 신호를 받아 실행
"""
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Bool
import time


global flag 
flag = False

def bool_callback(msg):
    global flag
    flag = msg.data
    rospy.loginfo("Received bool: %s", str(flag))

pose_msg = PoseStamped()



class RedObjectDetector:
    def __init__(self):
        rospy.init_node('red_object_detector', anonymous=True)

        self.bridge = CvBridge()
        self.intrinsics = None
        
        # Pose publisher
        self.pose_pub = rospy.Publisher('/fusion_pose', PoseStamped, queue_size=10)
        
        
        # 2번 core에서 돌릴 경우, 넘겨받는 mobile 카메라 토픽명으로 변경
        # Subscribers
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.color_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/flag/pick_flag', Bool, bool_callback)
        
        self.depth_image = None
        self.color_image = None

        rospy.spin()

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            # rospy.loginfo("Camera intrinsics received.")

    def color_callback(self, msg):
        # 압축 이미지 -> OpenCV 이미지
        self.color_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()



    def depth_callback(self, msg):
        # Depth 이미지 -> numpy 배열
        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image = np.array(depth_raw, dtype=np.float32)

    def process_images(self):
        # if self.color_image is None or self.depth_image is None or self.intrinsics is None:
        #     return

        # HSV 변환
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

        # # 빨간색 범위
        # lower_red1 = np.array([0, 120, 70])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([170, 120, 70])
        # upper_red2 = np.array([180, 255, 255])

        # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # mask = mask1 | mask2
        # ── 주황색 범위(권장 기본값) ─────────────────
        lower_orange = np.array([8, 120, 80], dtype=np.uint8)   # H≈8°
        upper_orange = np.array([24, 255, 255], dtype=np.uint8) # H≈24°
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # (선택) 노이즈 제거
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 가장 큰 윤곽선
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            # rospy.loginfo("빨간색 물체 없음.")
            return

        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] == 0:
            # rospy.logwarn("영역이 0입니다.")
            return

        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])

        # Depth 값 (meter 단위)
        z_raw = self.depth_image[v, u]
        if np.isnan(z_raw) or z_raw <= 0:
            rospy.logwarn("잘못된 Depth 값.")
            return

        # 단위 변환
        Z = z_raw  # meter
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        

        # rospy.loginfo("빨간색 물체 중심 좌표 (cm): X=%.1f, Y=%.1f, Z=%.1f", X, Y, Z)

        # 시각화
        result_img = self.color_image.copy()
        
        text = "X={:.3f}m Y={:.3f}m Z={:.3f}m".format(X, Y, Z)
        cv2.putText(
            result_img,
            text,
            (u + 10, v - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )
        
        real_x = (Z/1000) + 0.2
        if real_x < 0.0 :
            real_x = 0.0
        real_y = (-X/1000) + 0.05
        real_z = (-Y/1000) + 0.15
        real_x = round(real_x, 2)
        real_y = round(real_y, 2)
        real_z = round(real_z, 2)

        
        # PoseStamped 메시지 생성
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"   # 필요에 따라 수정

        pose_msg.pose.position.x = real_x
        pose_msg.pose.position.y = real_y
        pose_msg.pose.position.z = real_z

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        global flag
        # 토픽 발행
        if flag == True :
            self.pose_pub.publish(pose_msg)
            rospy.loginfo("fusion_pose 발행")
            time.sleep(10)
            
            flag = False
            
        cv2.circle(result_img, (u, v), 5, (0, 255, 0), -1)
        cv2.imshow("Red Object Detection", result_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        RedObjectDetector()
    except rospy.ROSInterruptException:
        pass





# import rospy
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import CompressedImage

# class CompressedImageViewer:
#     def __init__(self):
#         rospy.init_node('compressed_image_viewer', anonymous=True)
#         self.bridge = CvBridge()

#         # /camera/color/image_raw/compressed 구독
#         rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.callback)

#         rospy.loginfo("압축 이미지 구독 시작")
#         rospy.spin()

#     def callback(self, msg):
#         # 압축 이미지를 OpenCV 이미지로 변환
#         cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # 화면에 표시
#         cv2.imshow("Compressed Image", cv_image)
#         cv2.waitKey(1)

# if __name__ == '__main__':
#     try:
#         CompressedImageViewer()
#     except rospy.ROSInterruptException:
#         pass

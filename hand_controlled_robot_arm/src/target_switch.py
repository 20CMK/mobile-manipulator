#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, serial, threading, time
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int8
import message_filters

# MoveIt
from moveit_msgs.msg import (
    MoveGroupActionGoal, Constraints, JointConstraint, PlanningOptions
)
from actionlib_msgs.msg import GoalID

# ====================== 하드코딩 파트 ======================
GROUP_NAME = "arm"

HOME_JOINTS = {
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 0.0,
}

NUM_PLANNING_ATTEMPTS   = 10
ALLOWED_PLANNING_TIME   = 5.0
VEL_SCALE               = 0.1
ACC_SCALE               = 0.1
PIPELINE_ID             = "ompl"
PLANNER_ID              = ""    # 필요 없으면 빈 문자열

# workspace(world 좌표계, [-1,1])
WORKSPACE_FRAME         = "world"
WORKSPACE_MIN           = (-1.0, -1.0, -1.0)
WORKSPACE_MAX           = ( 1.0,  1.0,  1.0)

# joint tol = 0.0001
JOINT_TOL               = 1e-4

# ==========================================================

# ★ 홈 Pose (원하면 여기 숫자도 바꿔도 됨)
HOME_POS = (0.0, 0.0, 0.0)
HOME_ORI = (0.0, 0.0, 0.0, 0.0)

class TargetPosePublisher:
    def __init__(self):
        # ── 시리얼 ──────────────────────────────
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.ser_lock = threading.Lock()

        self.prev_b = False
        self.lock_timeout = rospy.get_param("~lock_timeout", 0.1)
        self.last_l_time = 0.0

        # ── 퍼블리셔 ────────────────────────────
        self.pub_pose      = rospy.Publisher("/target_pose", Pose, queue_size=10)
        self.pub_lock      = rospy.Publisher("/lock_switch", Bool, queue_size=10)
        self.pub_move_goal = rospy.Publisher("/move_group/goal",
                                             MoveGroupActionGoal, queue_size=1)
        # ▶ vision_pp_flag: 마지막 값 유지(latch)로 신규 구독자도 상태 인지 가능
        self.pub_vision_pp = rospy.Publisher("/vision_pp_flag", Bool, queue_size=1, latch=True)

        # ── 동기화 구독자 ───────────────────────
        sub_pos = message_filters.Subscriber("/robot_arm/target_position", Point)
        sub_imu = message_filters.Subscriber("/imu/data", Imu)
        rospy.Subscriber("/gripper_cmd", Int8, self.gripper_cb)
        ats = message_filters.ApproximateTimeSynchronizer(
            [sub_pos, sub_imu], queue_size=10, slop=0.03, allow_headerless=True)
        ats.registerCallback(self.sync_cb)

        # ── Home Pose 메시지 ────────────────────
        self.home_pose = Pose()
        self.home_pose.position.x, self.home_pose.position.y, self.home_pose.position.z = HOME_POS
        (self.home_pose.orientation.x,
         self.home_pose.orientation.y,
         self.home_pose.orientation.z,
         self.home_pose.orientation.w) = HOME_ORI

        # ── vision_pp_flag 상태 변수 ─────────────
        self.vision_flag_active  = False
        self.vision_timer        = None
        self.vision_pp_duration  = rospy.get_param("~vision_pp_duration", 3.0)
        self.pub_vision_pp.publish(Bool(False))  # 초기값 False

    # ① Point + Imu → Pose 재발행
    def sync_cb(self, pt_msg: Point, imu_msg: Imu):
        pose = Pose()
        pose.position    = pt_msg
        pose.orientation = imu_msg.orientation
        self.pub_pose.publish(pose)
        
    def gripper_cb(self, msg: Int8):
        cmd = b"a\n" if msg.data else b"b\n"     # 1=OPEN, 0=CLOSE
        with self.ser_lock:
            self.ser.write(cmd)

    # ② 시리얼 폴링 (‘b’, ‘l’, ‘y’ 처리)
    def poll_serial(self):
        with self.ser_lock:
            while self.ser.in_waiting:
                c = self.ser.read().decode('ascii', errors='ignore')

                if c == 'b':
                    if not self.prev_b:
                        self.publish_home_pose_and_movegoal()
                    self.prev_b = True
                else:
                    self.prev_b = False

                if c == 'l':
                    self.last_l_time = time.time()

                # ▶ 'y' 수신: True 1회 발행 → 3초 유지 → False 복귀
                if c == 'y' and not self.vision_flag_active:
                    self.trigger_vision_flag()

        # LOCK 스위치 상태 브로드캐스트
        pressed = (time.time() - self.last_l_time) < self.lock_timeout
        self.pub_lock.publish(Bool(pressed))

    # ── vision_pp_flag 트리거 ───────────────────
    def trigger_vision_flag(self):
        self.vision_flag_active = True
        self.pub_vision_pp.publish(Bool(True))
        # 기존 타이머가 있다면 종료 후 새로 시작(안전)
        if self.vision_timer is not None:
            try:
                self.vision_timer.shutdown()
            except Exception:
                pass
        self.vision_timer = rospy.Timer(
            rospy.Duration(self.vision_pp_duration),
            self._vision_flag_timeout,
            oneshot=True
        )
        rospy.loginfo("🟢 /vision_pp_flag: True ({}s 유지)".format(self.vision_pp_duration))

    def _vision_flag_timeout(self, _event):
        self.pub_vision_pp.publish(Bool(False))
        self.vision_flag_active = False
        self.vision_timer = None
        rospy.loginfo("⚪ /vision_pp_flag: False (자동 복귀)")

    # ── Home Pose & MoveGroupActionGoal 동시 발행 ──────────────────────────
    def publish_home_pose_and_movegoal(self):
        # 1) /target_pose
        self.pub_pose.publish(self.home_pose)
        rospy.loginfo("🏠 Home pose published")

        # 2) /move_group/goal
        if not HOME_JOINTS:
            rospy.logwarn("HOME_JOINTS 가 비어있습니다. /move_group/goal 은 보내지 않습니다.")
            return

        goal_msg = self.make_move_group_home_goal()
        self.pub_move_goal.publish(goal_msg)
        rospy.loginfo("🤖 MoveGroupActionGoal(home) published to /move_group/goal")

    # ── MoveGroupActionGoal 생성 ───────────────────────────────────────────
    def make_move_group_home_goal(self) -> MoveGroupActionGoal:
        goal = MoveGroupActionGoal()

        now = rospy.Time.now()
        goal.header.stamp = now
        goal.goal_id = GoalID()
        goal.goal_id.stamp = now
        goal.goal_id.id = "home_goal_%d" % int(now.to_sec() * 1000)

        req = goal.goal.request
        req.group_name = GROUP_NAME
        req.pipeline_id = PIPELINE_ID
        req.planner_id  = PLANNER_ID
        req.num_planning_attempts = NUM_PLANNING_ATTEMPTS
        req.allowed_planning_time = ALLOWED_PLANNING_TIME
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE

        # ── workspace_parameters ──────────────────────
        req.workspace_parameters.header.stamp = now
        req.workspace_parameters.header.frame_id = WORKSPACE_FRAME
        req.workspace_parameters.min_corner.x = WORKSPACE_MIN[0]
        req.workspace_parameters.min_corner.y = WORKSPACE_MIN[1]
        req.workspace_parameters.min_corner.z = WORKSPACE_MIN[2]
        req.workspace_parameters.max_corner.x = WORKSPACE_MAX[0]
        req.workspace_parameters.max_corner.y = WORKSPACE_MAX[1]
        req.workspace_parameters.max_corner.z = WORKSPACE_MAX[2]

        # ── start_state (is_diff=True 로만 세팅) ─────
        req.start_state.is_diff = True

        # ── JointConstraints (tol = 1e-4) ────────────
        jc_list = []
        for name, pos in HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = JOINT_TOL
            jc.tolerance_below = JOINT_TOL
            jc.weight = 1.0
            jc_list.append(jc)

        cstr = Constraints()
        cstr.joint_constraints = jc_list
        req.goal_constraints = [cstr]

        # ── PlanningOptions ──────────────────────────
        goal.goal.planning_options = PlanningOptions()
        goal.goal.planning_options.plan_only = False
        goal.goal.planning_options.look_around = False
        goal.goal.planning_options.replan = False

        return goal

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.poll_serial()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("target_switch_publisher")
    try:
        TargetPosePublisher().spin()
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")

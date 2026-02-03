#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
키보드 입력으로 'b','l','y' 이벤트를 받도록 수정한 버전 (아두이노/시리얼 제거)

[키 매핑]
  b : Home Pose 발행 + /move_group/goal 홈 조인트 목표 발행 (연속 입력 방지 로직 동일)
  l : /lock_switch True로 짧게 트리거 (lock_timeout 동안만 True)
  y : /vision_pp_flag True로 트리거 후 ~vision_pp_duration 초 뒤 False 자동 복귀

터미널(tty)에서 실행하세요. (VSCode 터미널 OK, GUI IDE 콘솔 일부는 미지원)
"""

import sys, termios, tty, select, atexit, time
import rospy
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

# ★ 홈 Pose (원하면 여기 숫자도 바꿔도 됨)
HOME_POS = (0.0, 0.0, 0.0)
HOME_ORI = (0.0, 0.0, 0.0, 0.0)

# ====================== 키보드 유틸 ======================
class KeyboardReader:
    """
    비차단 non-blocking getch() 구현.
    stdin을 raw 모드로 전환하고, select로 폴링하여 1문자씩 읽어온다.
    """
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_attrs = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        atexit.register(self.restore)

    def restore(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_attrs)
        except Exception:
            pass

    def getch_nowait(self):
        """입력이 있으면 1문자 반환, 없으면 None"""
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            ch = sys.stdin.read(1)
            return ch
        return None

# ====================== 메인 노드 ======================
class TargetPosePublisher:
    def __init__(self):
        # ── 파라미터 ──────────────────────────────
        self.lock_timeout       = rospy.get_param("~lock_timeout", 0.1)
        self.vision_pp_duration = rospy.get_param("~vision_pp_duration", 3.0)

        # ── 퍼블리셔 ───────────────────────────────
        self.pub_pose      = rospy.Publisher("/target_pose", Pose, queue_size=10)
        self.pub_lock      = rospy.Publisher("/lock_switch", Bool, queue_size=10)
        self.pub_move_goal = rospy.Publisher("/move_group/goal",
                                             MoveGroupActionGoal, queue_size=1)
        self.pub_vision_pp = rospy.Publisher("/vision_pp_flag", Bool, queue_size=1, latch=True)

        # ── 동기화 구독자 ─────────────────────────
        sub_pos = message_filters.Subscriber("/robot_arm/target_position", Point)
        sub_imu = message_filters.Subscriber("/imu/data", Imu)
        ats = message_filters.ApproximateTimeSynchronizer(
            [sub_pos, sub_imu], queue_size=10, slop=0.03, allow_headerless=True)
        ats.registerCallback(self.sync_cb)

        # (참고) /gripper_cmd는 더 이상 시리얼 송신을 하지 않지만, 필요한 경우를 위해 남겨둠
        rospy.Subscriber("/gripper_cmd", Int8, self.gripper_cb)

        # ── Home Pose 메시지 ──────────────────────
        self.home_pose = Pose()
        self.home_pose.position.x, self.home_pose.position.y, self.home_pose.position.z = HOME_POS
        (self.home_pose.orientation.x,
         self.home_pose.orientation.y,
         self.home_pose.orientation.z,
         self.home_pose.orientation.w) = HOME_ORI

        # ── 내부 상태 ─────────────────────────────
        self.prev_b             = False
        self.vision_flag_active = False
        self.vision_timer       = None
        self.pub_vision_pp.publish(Bool(False))  # 초기값 False

        # ★ 토글형 lock 상태
        self.lock_state       = False
        self.last_l_toggle    = 0.0
        self.toggle_debounce  = rospy.get_param("~toggle_debounce", 0.25)  # 초

        # ── 키보드 리더 ───────────────────────────
        self.kb = KeyboardReader()
        rospy.loginfo("Keyboard: [b]=Home, [l]=Lock TOGGLE, [y]=Vision flag")

    # ① Point + Imu → Pose 재발행
    def sync_cb(self, pt_msg: Point, imu_msg: Imu):
        pose = Pose()
        pose.position    = pt_msg
        pose.orientation = imu_msg.orientation
        self.pub_pose.publish(pose)

    # (옵션) 그리퍼 명령 들어오면 현재는 로그만 남김
    def gripper_cb(self, msg: Int8):
        # 1=OPEN, 0=CLOSE
        rospy.loginfo_throttle(1.0, f"/gripper_cmd={msg.data} (no serial forwarding)")

    # ───────── poll_keyboard() 전면 교체 ─────────
    def poll_keyboard(self):
        ch = self.kb.getch_nowait()

        if ch is not None:
            # 개행/특수키 무시
            if ch in ['\n', '\r', '\x03', '\x1b']:
                pass
            else:
                # 'b' : Home 1회성 처리(연타 보호 동일)
                if ch == 'b':
                    if not self.prev_b:
                        self.publish_home_pose_and_movegoal()
                    self.prev_b = True
                else:
                    self.prev_b = False

                # 'l' : 토글 + 디바운스
                if ch == 'l':
                    now = time.time()
                    if (now - self.last_l_toggle) > self.toggle_debounce:
                        self.lock_state = not self.lock_state
                        self.last_l_toggle = now
                        rospy.loginfo(f"/lock_switch toggled -> {self.lock_state}")

                # 'y' : vision flag 트리거(기존 동일)
                if ch == 'y' and not self.vision_flag_active:
                    self.trigger_vision_flag()
        else:
            # 키 입력이 없을 때에도 b 연타 보호 상태 해제는 poll 주기로 이뤄짐
            self.prev_b = False

        # 현재 lock 상태를 계속 브로드캐스트(상태 유지 목적)
        self.pub_lock.publish(Bool(self.lock_state))


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
            self.poll_keyboard()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("target_switch_publisher")
    TargetPosePublisher().spin()

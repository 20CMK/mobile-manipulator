#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import mediapipe as mp
from collections import deque
from geometry_msgs.msg import Point 
import serial
import time
from scipy.spatial.transform import Rotation as R
import math     
from std_msgs.msg import Bool, Int8

# 파일 상단 import 근처
from collections import deque
import time
cv2.setUseOptimized(True)
# [추가] 왼손(또는 원하는 손) 인덱스 찾기
TARGET_HAND_LABEL = "Left"  # ← 오른손만 쓰고 싶으면 "Right" 로 바꾸세요.
# --- 성능 파라미터 ---
PROC_W, PROC_H = 320, 240    # MediaPipe 전용 해상도 (270~360 권장)
MAX_HANDS = 1                # 왼손만 쓸 거면 1이 가장 빠름(정확도 필요시 2)
DRAW_EVERY_N = 2             # 랜드마크/텍스트 시각화 간격 프레임
CURL_EVERY_N = 2             # 컬 계산/그리퍼 판단 간격 프레임
DEPTH_WIN = 1                # 손목 주변 depth 샘플 반경(1→3x3, 2→5x5)

def get_hand_index(results, target_label=TARGET_HAND_LABEL):
    """
    results.multi_handedness와 results.multi_hand_landmarks의 순서는 동일합니다.
    target_label ("Left" / "Right")에 해당하는 손의 인덱스를 반환, 없으면 None.
    """
    if not results.multi_handedness:
        return None
    for i, handed in enumerate(results.multi_handedness):
        if handed.classification and handed.classification[0].label == target_label:
            return i
    return None


# ── 추가: 수학 유틸 ─────────────────────────────
def angle_atan2(u, v):
    u = norm(u); v = norm(v)
    c = np.cross(u, v)
    return np.degrees(np.arctan2(np.linalg.norm(c), np.dot(u, v)))

def proj_to_plane(v, n):  # v를 n(정규화된 법선)인 평면에 사영
    return v - np.dot(v, n) * n

def signed_angle_on_plane(u, v, n):  # 평면상 부호 있는 각
    u_p = norm(proj_to_plane(u, n)); v_p = norm(proj_to_plane(v, n))
    s = np.dot(np.cross(u_p, v_p), n)
    c = np.dot(u_p, v_p)
    return np.degrees(np.arctan2(s, c))

# MediaPipe 인덱스
WRIST = 0
MCP   = [2, 5, 9, 13, 17]
PIP   = [3, 6, 10, 14, 18]
DIP   = [3, 7, 11, 15, 19]   # 엄지는 3이 IP, DIP와 동일 취급
TIPS  = [4, 8, 12, 16, 20]

def get_xyz_from_depth(idx, hlm, depth_frame, intr_depth, R_inv, T_inv):
    p = get_landmark_3d(idx, hlm, depth_frame, intr_depth)
    if p is None:
        return None
    # 카메라→마커 좌표
    return (R_inv @ p.reshape(3,1) + T_inv).ravel()

def get_xyz_world(idx, wlm):
    # MediaPipe world_landmarks 사용 (권장: 각도 계산 안정)
    lm = wlm[idx]
    return np.array([lm.x, lm.y, lm.z], dtype=np.float32)

def compute_palm_axes(p_w, p_idx_mcp, p_mid_mcp, p_pky_mcp):
    v_x = norm(p_mid_mcp - p_w)
    v_z = norm(np.cross(p_idx_mcp - p_pky_mcp, v_x))
    v_y = np.cross(v_z, v_x)
    R_palm = np.stack([v_x, v_y, v_z], axis=1)  # columns
    return R_palm, v_x, v_y, v_z  # v_z = palm normal

# def compute_finger_curls(results, depth_frame, intr_depth, R_inv, T_inv, use_world=True):
#     """
#     반환: dict {'Thumb': curl_deg, 'Index': ..., 'Middle': ..., 'Ring': ..., 'Pinky': ...}
#     world_landmarks가 있으면 use_world=True로 각도 계산 시 사용 (권장)
#     """
#     if use_world and results.multi_hand_world_landmarks:
#         src = results.multi_hand_world_landmarks[0].landmark
#         getp = lambda i: get_xyz_world(i, src)
#     else:
#         hlm = results.multi_hand_landmarks[0].landmark
#         getp = lambda i: get_xyz_from_depth(i, hlm, depth_frame, intr_depth, R_inv, T_inv)
# [변경]
def compute_finger_curls(results, depth_frame, intr_depth, R_inv, T_inv, use_world=True, hand_index=0):
    if use_world and results.multi_hand_world_landmarks:
        src = results.multi_hand_world_landmarks[hand_index].landmark
        getp = lambda i: get_xyz_world(i, src)
    else:
        hlm = results.multi_hand_landmarks[hand_index].landmark
        getp = lambda i: get_xyz_from_depth(i, hlm, depth_frame, intr_depth, R_inv, T_inv)
    # (나머지 기존 로직 동일)

    # 손바닥 축
    pts = [getp(WRIST), getp(5), getp(9), getp(17)]
    if any(p is None for p in pts):  # depth 누락 시 실패
        return None
    p_w, p_idx_mcp, p_mid_mcp, p_pky_mcp = pts
    R_palm, v_x, v_y, v_z = compute_palm_axes(p_w, p_idx_mcp, p_mid_mcp, p_pky_mcp)

    names = ["Thumb","Index","Middle","Ring","Pinky"]
    curls = {}

    for fi, name in enumerate(names):
        p_mcp = getp(MCP[fi]); p_pip = getp(PIP[fi]); p_dip = getp(DIP[fi]); p_tip = getp(TIPS[fi])
        if any(p is None for p in [p_mcp, p_pip, p_dip, p_tip]):
            curls[name] = None
            continue

        u_prox = p_pip - p_mcp    # MCP→PIP
        u_mid  = p_dip - p_pip    # PIP→DIP
        u_dist = p_tip - p_dip    # DIP→TIP

        # 관절별 굽힘
        pip_deg = angle_atan2(u_prox, u_mid)
        dip_deg = angle_atan2(u_mid,  u_dist)

        # MCP flexion (선택): 손바닥 x축(v_x) 대비
        # mcp_flex = angle_atan2(v_x, proj_to_plane(u_prox, v_z))

        # Curl(한 숫자) = 가중합 (PIP 위주)
        if fi == 0:
            # 엄지는 IP가 PIP/DIP 역할을 함께 하므로 tip-독립 보정
            curl = pip_deg
        else:
            curl = 0.6 * pip_deg + 0.4 * dip_deg

        # 물리적 범위로 클램프
        curls[name] = float(np.clip(curl, 0.0, 160.0))
        order = ["Thumb","Index","Middle","Ring","Pinky"]

    return [float(curls[k]) if (k in curls and curls[k] is not None) else 0.0 for k in order]


class StableGripper:
    """
    thr_open: 이보다 작으면 '열림' 의도
    thr_close: 이보다 크면 '닫힘' 의도
    n_open/n_close: 해당 의도가 N프레임 연속일 때만 전이
    min_hold: 전이 후 최소 유지시간(초)
    refractory: 전이 직후 반대 전이를 무시할 시간(초)
    """
    def __init__(self, thr_open=20.0, thr_close=105.0, n_open=4, n_close=4,
                 min_hold=0.35, refractory=0.50):
        self.thr_open   = thr_open
        self.thr_close  = thr_close
        self.n_open     = n_open
        self.n_close    = n_close
        self.min_hold   = min_hold
        self.refractory = refractory

        self.state = 0            # 0=CLOSE, 1=OPEN
        self.last_switch_t = 0.0
        self.buf = deque(maxlen=max(n_open, n_close))

    def _vote_from_angles(self, angles):
        """
        angles: [Thumb, Index, Middle, Ring, Pinky] 순서의 굽힘각 리스트
        검지·중지·약지에서 다수결
        """
        use = [angles[1]]#, angles[2],angles[4]]  # Index, Middle, Ring
        open_cnt  = sum(a < self.thr_open  for a in use)
        close_cnt = sum(a > self.thr_close for a in use)

        if open_cnt >= 1:
            return 1  # OPEN 의도
        if close_cnt >= 1:
            return 0  # CLOSE 의도
        return self.state  # 히스테리시스 구간에서는 현 상태 유지

    def update(self, angles):
        now = time.time()

        # 불응기: 전이 직후 잠깐은 반대 전이를 무시
        if (now - self.last_switch_t) < self.refractory:
            return self.state, False

        vote = self._vote_from_angles(angles)
        self.buf.append(vote)

        changed = False
        if self.state == 0:
            # 닫힘 상태일 때 '열림' 의도가 n_open 프레임 연속
            if len(self.buf) >= self.n_open and all(b == 1 for b in list(self.buf)[-self.n_open:]):
                if (now - self.last_switch_t) >= self.min_hold:
                    self.state = 1
                    self.last_switch_t = now
                    changed = True
        else:
            # 열림 상태일 때 '닫힘' 의도가 n_close 프레임 연속
            if len(self.buf) >= self.n_close and all(b == 0 for b in list(self.buf)[-self.n_close:]):
                if (now - self.last_switch_t) >= self.min_hold:
                    self.state = 0
                    self.last_switch_t = now
                    changed = True

        return self.state, changed



def to_palm_global(pt_glob, R_palm, T_inv):
    """
    pt_glob : (3,)  마커 기준 좌표
    R_palm  : (3,3) 손바닥 회전 행렬
    T_inv   : (3,1) 마커 원점 → 카메라 원점 오프셋
    반환    : (3,)  Palm 로컬 좌표
    """
    vec = pt_glob        # shape (3,)
    return (R_palm.T @ vec).ravel()

# ── 벡터 정규화 ─────────────────────────────
def norm(v): return v / (np.linalg.norm(v) + 1e-8)

TH_OPEN, TH_CLOSE = 10,  5   # φ >15° → OPEN, φ <5° → CLOSE


# ──────────────────────────────────────────
# 0. 상수·전역 설정
# ──────────────────────────────────────────
# WRIST = 0
# TIPS  = [4, 8, 12, 16, 20]      # Thumb, Index, Middle, Ring, Pinky
# MCP   = [2, 5, 9, 13, 17]
# PIP   = [3, 6,10, 14, 18]

OPEN_US, CLOSE_US = 1650, 1100
THUMB_POS_OPEN, THUMB_POS_CLOSE = 1650, 1100

# ──────────────────────────────────────────
# 1. 보조 함수
# ──────────────────────────────────────────
# ――― 2) get_landmark_3d() 함수 ―――
def get_landmark_3d(lm_idx, hand_lm, depth_frame, intr, win=2):
    u = int(hand_lm[lm_idx].x * intr.width)
    v = int(hand_lm[lm_idx].y * intr.height)
    vals = []

    for dx in range(-win, win + 1):
        for dy in range(-win, win + 1):
            x = u + dx
            y = v + dy
            if 0 <= x < intr.width and 0 <= y < intr.height:
                d = depth_frame.get_distance(x, y)
                if d > 0.1:
                    vals.append(d)

    if not vals:
        return None
    depth = np.mean(vals)
    return np.array(rs.rs2_deproject_pixel_to_point(intr, [u, v], depth))


# def finger_angle(p_mcp, p_pip, p_tip):
#     """세 점으로 굽힘 각 계산 (0°: 펴짐, 90°↑: 완전 굽힘)"""
#     v1, v2 = p_pip - p_mcp, p_tip - p_pip
#     cos_t = np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2) + 1e-6)
#     return np.degrees(np.arccos(np.clip(cos_t, -1, 1)))
def thumb_flex_cmc(p_cmc, p_mcp, p_tip):
    """
    엄지 굽힘 각 (CMC 축 기준)
    · p_cmc : Thumb_CMC (id=1)
    · p_mcp : Thumb_MCP (id=2)
    · p_tip : Thumb_TIP (id=4)
    반환 : 0°(펴짐) ~ 90°+ (굽힘)
    """
    v0 = p_mcp - p_cmc          # CMC → MCP  (엄지 뼈대 첫 구간)
    v1 = p_tip - p_mcp          # MCP → TIP  (끝 구간)
    cos_t = np.dot(v0, v1) / (np.linalg.norm(v0)*np.linalg.norm(v1) + 1e-6)
    return np.degrees(np.arccos(np.clip(cos_t, -1, 1)))

def angle_to_us(angle, min_deg=15, max_deg=120):
    """각도 → 서보 PWM(µs) 선형 매핑"""
    ang = np.clip(angle, min_deg, max_deg)
    return int(OPEN_US - (ang - min_deg) / (max_deg - min_deg) * (OPEN_US - CLOSE_US))


# ── 2-단계 매핑 ──────────────────────────────
OPEN_PWM  = OPEN_US     # 1650 µs
CLOSE_PWM = CLOSE_US    # 1100 µs

def angle_to_openclose(angle, thr=95):
    """
    angle < thr  → 열림(OPEN_PWM)
    angle ≥ thr  → 닫힘(CLOSE_PWM)
    thr: 손가락 굽힘 임계각 [deg]
    """
    return OPEN_PWM if angle < thr else CLOSE_PWM

THUMB_THR = 40          # 엄지 굽힘 임계각(예시). 50° 이상이면 닫힘
def thumb_bend_to_pwm(angle):
    return OPEN_PWM if angle < THUMB_THR else CLOSE_PWM

# ───────────── LOCK 상태 콜백 ──────────────
lock_pressed = False
def lock_cb(msg: Bool):
    global lock_pressed
    lock_pressed = msg.data

def main():
    rospy.init_node("hand_controlled_robot_arm")
    pub = rospy.Publisher("/robot_arm/target_position", Point, queue_size=10)
    pub_gripper = rospy.Publisher("/gripper_cmd", Int8, queue_size=10)
    rospy.Subscriber("/lock_switch", Bool, lock_cb)           
    # --- 내부 파라미터 ---
    camera_matrix = np.array([
        [358.9792,    0.0,     338.2827],
        [0.0,       481.1011, 216.7561],
        [0.0,         0.0,       1.0]
    ])

    dist_coeffs = np.array([
        0.024017,  0.096752, -0.023020,  0.011180,  0.0
    ])

    grip = StableGripper(thr_open=20.0, thr_close=40.0, n_open=4, n_close=4,
                     min_hold=0.35, refractory=0.50)
    prev_state = grip.state  # 초기값 동기

    # --- 칼만 필터 설정 ---
    kf = cv2.KalmanFilter(6, 3)
    kf.measurementMatrix = np.zeros((3, 6), dtype=np.float32)
    kf.measurementMatrix[0, 0] = 1
    kf.measurementMatrix[1, 1] = 1
    kf.measurementMatrix[2, 2] = 1
    kf.transitionMatrix = np.array([
        [1, 0, 0, 1, 0, 0],
        [0, 1, 0, 0, 1, 0],
        [0, 0, 1, 0, 0, 1],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ], dtype=np.float32)
    kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-4
    kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-2
    kalman_initialized = False

    # --- ArUco 설정 ---
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    marker_length = 0.1  # 10cm

    # --- MediaPipe 설정 ---
    mp_hands = mp.solutions.hands
    # [변경]
    hands = mp_hands.Hands(
    max_num_hands=MAX_HANDS, model_complexity=0,
    min_detection_confidence=0.7, min_tracking_confidence=0.5
    )


    # --- RealSense 초기화 ---
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)

    # ── (여기에 추가) 오래된 프레임이 쌓이지 않도록 큐 길이 1로 설정 ──
    dev = profile.get_device()
    # 일부 환경에서는 dev.sensors 대신 dev.query_sensors()를 사용해야 합니다.
    try:
        sensors = dev.sensors
    except AttributeError:
        sensors = dev.query_sensors()

    for s in sensors:
        if s.supports(rs.option.frames_queue_size):
            s.set_option(rs.option.frames_queue_size, 1)


    align = rs.align(rs.stream.color)
    # ★ depth 스트림 intrinsics 한 번 캐싱
    intr_depth = profile.get_stream(rs.stream.depth) \
                    .as_video_stream_profile().get_intrinsics()
    
    # [추가] 컬러 intrinsics 캐시
    intr_color = profile.get_stream(rs.stream.color) \
    .as_video_stream_profile().get_intrinsics()

    # --- 좌표계 고정 변수 ---
    R_inv = None
    T_inv = None
    rvec_fixed = None
    tvec_fixed = None

    # 송신 주기 및 임계값 설정
    #rate = rospy.Rate(10)  # 초당 10번 송신 제한
    prev_msg = None
    threshold = 0.01  # 좌표 변화량 임계값 (0.1cm)
    prev_state = -1      # 0=CLOSE, 1=OPEN (초기값 없음)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            # [추가] FPS 카운터 업데이트 (루프 상단)
            if not hasattr(main, "_fps_t"):
                main._fps_t = time.time()
                main._fps_c = 0
                main._fps_str = "FPS: --"
            main._fps_c += 1
            if time.time() - main._fps_t > 1.0:
                fps = main._fps_c / (time.time() - main._fps_t)
                main._fps_t = time.time()
                main._fps_c = 0
                main._fps_str = f"FPS: {fps:.1f}"
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if ids is not None and 1 in ids:
                idx = np.where(ids == 1)[0][0]
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([
                    corners[idx]], marker_length, camera_matrix, dist_coeffs
                )
                rvec_fixed = rvec[0]
                tvec_fixed = tvec[0]
                cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec_fixed, tvec_fixed, 0.05)

                if R_inv is None and T_inv is None:
                    R, _ = cv2.Rodrigues(rvec_fixed)
                    T = tvec_fixed.reshape((3, 1))
                    R_inv = R.T
                    T_inv = -R_inv @ T
                    print("✅ 마커 1번 기준 좌표계 고정 완료")
            # 기존: 매 프레임 detectMarkers
            # 변경: R_inv/T_inv가 없을 때만 탐지
            # if R_inv is None or T_inv is None:
            #     gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            #     corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            #     if ids is not None and 1 in ids:
            #         idx = np.where(ids == 1)[0][0]
            #         rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            #             [corners[idx]], marker_length, camera_matrix, dist_coeffs
            #         )
            #         rvec_fixed = rvec[0]
            #         tvec_fixed = tvec[0]
            #         cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec_fixed, tvec_fixed, 0.05)

            #         R, _ = cv2.Rodrigues(rvec_fixed)
            #         T = tvec_fixed.reshape((3, 1))
            #         R_inv = R.T
            #         T_inv = -R_inv @ T
            #         print("✅ 마커 1번 기준 좌표계 고정 완료")


            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            rgb_proc = cv2.resize(rgb_image, (PROC_W, PROC_H), interpolation=cv2.INTER_LINEAR)
            results = hands.process(rgb_proc)
            #results = hands.process(rgb_image)
            if results.multi_hand_landmarks and R_inv is not None:
                # for hand_landmarks in results.multi_hand_landmarks:
                left_idx = get_hand_index(results, TARGET_HAND_LABEL)
                if left_idx is None:
                    # 왼손이 없으면 이 프레임은 건너뜁니다.
                    cv2.putText(color_image, "Waiting for RIGHT hand...", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.imshow("Hand Tracker", color_image)
                    if cv2.waitKey(1) & 0xFF == 27: break
                    continue

                hand_landmarks = results.multi_hand_landmarks[left_idx]

                # MediaPipe 시각화 유틸 가져오기
                mp_drawing = mp.solutions.drawing_utils

                # 반복문 안에서 hand_landmarks 그리기
                mp_drawing.draw_landmarks(
                    color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                wrist = hand_landmarks.landmark[0]
                u = int(wrist.x * 640)
                v = int(wrist.y * 480)

                cv2.circle(color_image, (u, v), 5, (0, 255, 255), -1)

                # ――― 1) 깊이 샘플링 루프 (main 내부) ―――
                depth_vals = []
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        x = u + dx
                        y = v + dy
                        if 0 <= x < intr_depth.width and 0 <= y < intr_depth.height:
                            d = depth_frame.get_distance(x, y)
                            if d > 0.1:
                                depth_vals.append(d)


                if depth_vals:
                    depth = np.mean(depth_vals)
                    #intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                    point_3d = rs.rs2_deproject_pixel_to_point(intr_color, [u, v], depth)
                    X_cam = np.array(point_3d).reshape((3, 1))

                    text1 = f"(u,v): ({u},{v})"
                    text2 = f"depth: {depth:.3f} m"
                    text3 = f"3D: ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f}) m"
                    cv2.putText(color_image, text1, (u + 10, v - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                    cv2.putText(color_image, text2, (u + 10, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                    cv2.putText(color_image, text3, (u + 10, v + 5),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

                    X_relative = R_inv @ X_cam + T_inv

                    measurement = X_relative.astype(np.float32)
                    if not kalman_initialized:
                        kf.statePre[:3, 0] = measurement.ravel()
                        kf.statePre[3:, 0] = 0
                        kalman_initialized = True
                    kf.predict()    
                    kf.correct(measurement)
                    X_filtered = kf.statePost[:3]

                    #x_m = X_filtered[1, 0]     
                    #y_m = X_filtered[0, 0] 
                    #z_m = X_filtered[2, 0]
                    #print(f"로봇팔 타겟 좌표: X={x_m:.1f} cm, Y={y_m:.1f} cm, Z={z_m:.1f} cm")

                    # --- ROS 토픽 송신 ---
                    msg = Point()
                    msg.x = np.clip(X_filtered[1, 0], 0.2, 0.65)
                    msg.y = -(np.clip(X_filtered[0, 0], -0.4, 0.4))
                    msg.z = np.clip(0.95 - point_3d[2], 0.2, 0.65)
                    # 변화량이 0.01m 이상일 때만 송신
                    if prev_msg is None or \
                    abs(msg.x - prev_msg.x) > threshold or \
                    abs(msg.y - prev_msg.y) > threshold or \
                    abs(msg.z - prev_msg.z) > threshold:
                        if lock_pressed:
                            pub.publish(msg)
                            prev_msg = msg  # 현재 값을 저장
                            

                    target_x = np.clip(X_filtered[1, 0], 0.0, 0.6)
                    target_y = np.clip(X_filtered[0, 0], -0.4, 0.4)
                    target_z = np.clip(0.95 - point_3d[2], 0.0, 0.55)

                    text4 = f"MARKER 3D: ({target_x :.2f}, {target_y :.2f}, {target_z:.2f}) m"
                    cv2.putText(color_image, text4, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 1)

                    if rvec_fixed is not None and tvec_fixed is not None:
                        origin_pt = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
                        wrist_pt = X_filtered.T.astype(np.float32)
                        img_origin, _ = cv2.projectPoints(origin_pt, rvec_fixed, tvec_fixed, camera_matrix, dist_coeffs)
                        img_wrist, _ = cv2.projectPoints(wrist_pt, rvec_fixed, tvec_fixed, camera_matrix, dist_coeffs)

                        p0 = tuple(img_origin[0].ravel().astype(int))
                        p1 = tuple(img_wrist[0].ravel().astype(int))
                        cv2.circle(color_image, p0, 6, (0, 255, 0), -1)
                        #cv2.circle(color_image, p1, 6, (0, 0, 255), -1)
                        cv2.putText(color_image, "origin", (p0[0]+5, p0[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                        #cv2.putText(color_image, "wrist", (p1[0]+5, p1[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                hlm = results.multi_hand_landmarks[left_idx].landmark
                
                # ❶ 손바닥 축 계산용 포인트 (깊이 필수)
                pts_idx = [0, 5, 9, 17]               # Wrist, Index_MCP, Middle_MCP, Pinky_MCP
                p_glob = []
                for i in pts_idx:
                    p = get_landmark_3d(i, hlm, depth_frame, intr_depth)
                    if p is None: break
                    p_glob.append( (R_inv @ p.reshape(3,1) + T_inv).ravel() )
                if len(p_glob) < 4:
                    continue          # depth 누락 시 건너뜀

                p_w, p_idx, p_mid, p_pky = p_glob
                v_x = norm(p_mid - p_w)
                v_z = norm(np.cross(p_idx - p_pky, v_x))
                v_y = np.cross(v_z, v_x)
                R_palm = np.stack([v_x, v_y, v_z], axis=1)   # 3×3
                to_palm = lambda p: (R_palm.T @ p).ravel()   # ❷ 로컬 변환 함수

                # 2-A. 각 손가락 관절 3D 좌표
                # ang_list = []
                # miss_depth = False

                # for ti, mi, pi in zip(TIPS, MCP, PIP):
                #     # --- ① 깊이 추출 (카메라 → 마커 좌표) ----------------
                #     p_mcp_g = get_landmark_3d(mi, hlm, depth_frame, intr_depth)
                #     p_pip_g = get_landmark_3d(pi, hlm, depth_frame, intr_depth)
                #     p_tip_g = get_landmark_3d(ti, hlm, depth_frame, intr_depth)
                #     if any(x is None for x in (p_mcp_g, p_pip_g, p_tip_g)):
                #         miss_depth = True
                #         break
                #     p_mcp_g = (R_inv @ p_mcp_g.reshape(3,1) + T_inv).ravel()
                #     p_pip_g = (R_inv @ p_pip_g.reshape(3,1) + T_inv).ravel()
                #     p_tip_g = (R_inv @ p_tip_g.reshape(3,1) + T_inv).ravel()

                #     # --- ② Palm 변환 ------------------------------------
                #     p_mcp = to_palm_global(p_mcp_g, R_palm, T_inv)
                #     p_pip = to_palm_global(p_pip_g, R_palm, T_inv)
                #     p_tip = to_palm_global(p_tip_g, R_palm, T_inv)


                #     # --- ③ 각도 계산 ------------------------------------
                #     ang_list.append(finger_angle(p_mcp, p_pip, p_tip))

                # if miss_depth:
                #     continue  # depth 누락 시 건너뜀

                # # ── Thumb CMC-기준 굽힘각 계산 ─────────────────────────
                # p_cmc_g = get_landmark_3d(1, hlm, depth_frame, intr_depth)   # Thumb_CMC
                # p_mcp_g = get_landmark_3d(2, hlm, depth_frame, intr_depth)   # Thumb_MCP
                # p_tip_g = get_landmark_3d(4, hlm, depth_frame, intr_depth)   # Thumb_TIP
                # if any(x is None for x in (p_cmc_g, p_mcp_g, p_tip_g)):
                #     continue      # 깊이 누락 시 프레임 스킵                        # 깊이 누락 시 프레임 스킵

                # # 카메라 → 마커
                # p_cmc_g = (R_inv @ p_cmc_g.reshape(3,1) + T_inv).ravel()
                # p_mcp_g = (R_inv @ p_mcp_g.reshape(3,1) + T_inv).ravel()
                # p_tip_g = (R_inv @ p_tip_g.reshape(3,1) + T_inv).ravel()

                # # Palm 로컬
                # p_cmc = to_palm_global(p_cmc_g, R_palm, T_inv)
                # p_mcp = to_palm_global(p_mcp_g, R_palm, T_inv)
                # p_tip = to_palm_global(p_tip_g, R_palm, T_inv)

                # thumb_deg = thumb_flex_cmc(p_cmc, p_mcp, p_tip)   # ← 새 각도

                # finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
                # for idx, deg in enumerate(ang_list):
                # #     # ① 터미널 로그
                #     print(f"{finger_names[idx]:>6}: {deg:5.1f}°", end="  ")
                # print()
                # (추가) 새 컬 기반 각도 계산
                # curls = compute_finger_curls(results, depth_frame, intr_depth, R_inv, T_inv, use_world=True)
                # [변경] 왼손 인덱스를 넘겨줌
                curls = compute_finger_curls(results, depth_frame, intr_depth, R_inv, T_inv,
                                            use_world=True, hand_index=left_idx)

                if curls is None:
                    continue

                # StableGripper가 기대하는 순서: [Thumb, Index, Middle, Ring, Pinky]
                ang_list = curls
                print(f"Idx:{ang_list[1]:5.1f}  Mid:{ang_list[2]:5.1f}  Ring:{ang_list[3]:5.1f}  Pky:{ang_list[4]:5.1f}")



                #print(f"Thumb (CMC) : {thumb_deg:5.1f}°")
                #print("pinky:",{ang_list[4]})

                # Palm 좌표계 TIP·CMC
                # p_tip_loc = to_palm( (R_inv @ get_landmark_3d(4, hlm, depth_frame, intr_depth).reshape(3,1)+T_inv).ravel() )
                # p_cmc_loc = to_palm( (R_inv @ get_landmark_3d(1, hlm, depth_frame, intr_depth).reshape(3,1)+T_inv).ravel() )
                # phi = math.degrees(math.atan2(p_tip_loc[1]-p_cmc_loc[1], p_tip_loc[0]-p_cmc_loc[0]))
                # #print(f"[Thumb-spread φ] {phi:5.1f}°")   # 터미널 로그
                # # 히스테리시스 상태머신
                # if not hasattr(main, "thumb_state"):
                #     main.thumb_state = "CLOSE"
                # if   abs(phi) > TH_OPEN:   main.thumb_state = "OPEN"
                # elif abs(phi) < TH_CLOSE:  main.thumb_state = "CLOSE"

                # us_pos  = THUMB_POS_OPEN  if main.thumb_state=="OPEN" else THUMB_POS_CLOSE   # 서보0
                                                      # 서보1


            #     pulses_raw = [
            #     us_pos,                               # Servo 0 : 엄지 벌림
            #     angle_to_openclose(ang_list[1]),#thumb_bend_to_pwm(thumb_deg),      # Servo 1 : 엄지 굽힘
            #     angle_to_openclose(ang_list[1]),      # Servo 2 : 검지
            #     angle_to_openclose(ang_list[1]),      # Servo 3 : 중지
            #     angle_to_openclose(ang_list[1]),      # Servo 4 : 약지
            #     angle_to_openclose(ang_list[1])       # Servo 5 : 새끼
            # ]
                # ang_list 순서: [Thumb, Index, Middle, Ring, Pinky]
                state, changed = grip.update(ang_list)

                if lock_pressed and changed:
                    pub_gripper.publish(Int8(1 if state == 1 else 0))
                    prev_state = state

                # state = 1 if angle_to_openclose(ang_list[1]) == 1650 else 0
                # if lock_pressed and state != prev_state:
                #     pub_gripper.publish(Int8(state))   # ⬅️ 토픽 발행
                #     prev_state = state
            cv2.putText(color_image, main._fps_str, (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Hand Tracker", color_image)
            if cv2.waitKey(1) & 0xFF == 27:
                break
            
            # 송신 주기 조정
            #rate.sleep() 

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
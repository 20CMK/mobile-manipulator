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
WRIST = 0
TIPS  = [4, 8, 12, 16, 20]      # Thumb, Index, Middle, Ring, Pinky
MCP   = [2, 5, 9, 13, 17]
PIP   = [3, 6,10, 14, 18]

OPEN_US, CLOSE_US = 1650, 1100
THUMB_POS_OPEN, THUMB_POS_CLOSE = 1650, 1100

# ──────────────────────────────────────────
# 1. 보조 함수
# ──────────────────────────────────────────
def get_landmark_3d(lm_idx, hand_lm, depth_frame, intr, win=2):
    """Landmark index → RealSense 카메라 기준 3D [m]; depth 미검출 시 None"""
    u = int(hand_lm[lm_idx].x * intr.width)
    v = int(hand_lm[lm_idx].y * intr.height)
    vals = [depth_frame.get_distance(u+dx, v+dy)
            for dx in range(-win, win+1) for dy in range(-win, win+1)
            if depth_frame.get_distance(u+dx, v+dy) > 0.1]
    if not vals:
        return None
    depth = np.mean(vals)
    return np.array(rs.rs2_deproject_pixel_to_point(intr, [u, v], depth))  # (x,y,z)

def finger_angle(p_mcp, p_pip, p_tip):
    """세 점으로 굽힘 각 계산 (0°: 펴짐, 90°↑: 완전 굽힘)"""
    v1, v2 = p_pip - p_mcp, p_tip - p_pip
    cos_t = np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2) + 1e-6)
    return np.degrees(np.arccos(np.clip(cos_t, -1, 1)))
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



def main():
    rospy.init_node("hand_controlled_robot_arm")
    pub = rospy.Publisher("/robot_arm/target_position", Point, queue_size=10)
    arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=0)
    last_lock_time = 0.0
    LOCK_TIMEOUT = 0.1  
    # --- 내부 파라미터 ---
    camera_matrix = np.array([
        [358.9792,    0.0,     338.2827],
        [0.0,       481.1011, 216.7561],
        [0.0,         0.0,       1.0]
    ])

    dist_coeffs = np.array([
        0.024017,  0.096752, -0.023020,  0.011180,  0.0
    ])

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
    hands = mp_hands.Hands(max_num_hands=1, model_complexity=0,min_detection_confidence=0.7)

    # --- RealSense 초기화 ---
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)

    align = rs.align(rs.stream.color)
    # ★ depth 스트림 intrinsics 한 번 캐싱
    intr_depth = profile.get_stream(rs.stream.depth) \
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

    try:
        while True:
            # ── ① LOCK 스위치 문자 읽기 ─────────────────────
            while arduino.in_waiting:
                ch = arduino.read().decode('ascii', errors='ignore')
                if ch == 'l':                       # ‘l’ = LOCK 눌림
                    last_lock_time = time.time()

            lock_pressed = (time.time() - last_lock_time) < LOCK_TIMEOUT

            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
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

            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb_image)
            if results.multi_hand_landmarks and R_inv is not None:
                for hand_landmarks in results.multi_hand_landmarks:
                    # MediaPipe 시각화 유틸 가져오기
                    mp_drawing = mp.solutions.drawing_utils

                    # 반복문 안에서 hand_landmarks 그리기
                    mp_drawing.draw_landmarks(
                        color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    
                    wrist = hand_landmarks.landmark[0]
                    u = int(wrist.x * 640)
                    v = int(wrist.y * 480)

                    cv2.circle(color_image, (u, v), 5, (0, 255, 255), -1)

                    depth_vals = []
                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            d = depth_frame.get_distance(u + dx, v + dy)
                            if d > 0.1:
                                depth_vals.append(d)

                    if depth_vals:
                        depth = np.mean(depth_vals)
                        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                        point_3d = rs.rs2_deproject_pixel_to_point(intr, [u, v], depth)
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
                        msg.x = np.clip(X_filtered[1, 0], 0.0, 0.5)
                        msg.y = -(np.clip(X_filtered[0, 0], -0.4, 0.4))
                        msg.z = np.clip(0.95 - point_3d[2], 0.0, 0.55)
                        # 변화량이 0.01m 이상일 때만 송신
                        if prev_msg is None or \
                        abs(msg.x - prev_msg.x) > threshold or \
                        abs(msg.y - prev_msg.y) > threshold or \
                        abs(msg.z - prev_msg.z) > threshold:
                            if lock_pressed:
                                pub.publish(msg)
                                prev_msg = msg  # 현재 값을 저장
                                

                        target_x = np.clip(X_filtered[1, 0], 0.0, 0.5)
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
                hlm = results.multi_hand_landmarks[0].landmark
                
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
                ang_list = []
                miss_depth = False

                for ti, mi, pi in zip(TIPS, MCP, PIP):
                    # --- ① 깊이 추출 (카메라 → 마커 좌표) ----------------
                    p_mcp_g = get_landmark_3d(mi, hlm, depth_frame, intr_depth)
                    p_pip_g = get_landmark_3d(pi, hlm, depth_frame, intr_depth)
                    p_tip_g = get_landmark_3d(ti, hlm, depth_frame, intr_depth)
                    if any(x is None for x in (p_mcp_g, p_pip_g, p_tip_g)):
                        miss_depth = True
                        break
                    p_mcp_g = (R_inv @ p_mcp_g.reshape(3,1) + T_inv).ravel()
                    p_pip_g = (R_inv @ p_pip_g.reshape(3,1) + T_inv).ravel()
                    p_tip_g = (R_inv @ p_tip_g.reshape(3,1) + T_inv).ravel()

                    # --- ② Palm 변환 ------------------------------------
                    p_mcp = to_palm_global(p_mcp_g, R_palm, T_inv)
                    p_pip = to_palm_global(p_pip_g, R_palm, T_inv)
                    p_tip = to_palm_global(p_tip_g, R_palm, T_inv)


                    # --- ③ 각도 계산 ------------------------------------
                    ang_list.append(finger_angle(p_mcp, p_pip, p_tip))

                if miss_depth:
                    continue  # depth 누락 시 건너뜀

                # ── Thumb CMC-기준 굽힘각 계산 ─────────────────────────
                p_cmc_g = get_landmark_3d(1, hlm, depth_frame, intr_depth)   # Thumb_CMC
                p_mcp_g = get_landmark_3d(2, hlm, depth_frame, intr_depth)   # Thumb_MCP
                p_tip_g = get_landmark_3d(4, hlm, depth_frame, intr_depth)   # Thumb_TIP
                if any(x is None for x in (p_cmc_g, p_mcp_g, p_tip_g)):
                    continue      # 깊이 누락 시 프레임 스킵                        # 깊이 누락 시 프레임 스킵

                # 카메라 → 마커
                p_cmc_g = (R_inv @ p_cmc_g.reshape(3,1) + T_inv).ravel()
                p_mcp_g = (R_inv @ p_mcp_g.reshape(3,1) + T_inv).ravel()
                p_tip_g = (R_inv @ p_tip_g.reshape(3,1) + T_inv).ravel()

                # Palm 로컬
                p_cmc = to_palm_global(p_cmc_g, R_palm, T_inv)
                p_mcp = to_palm_global(p_mcp_g, R_palm, T_inv)
                p_tip = to_palm_global(p_tip_g, R_palm, T_inv)

                thumb_deg = thumb_flex_cmc(p_cmc, p_mcp, p_tip)   # ← 새 각도

                finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
                for idx, deg in enumerate(ang_list):
                #     # ① 터미널 로그
                    print(f"{finger_names[idx]:>6}: {deg:5.1f}°", end="  ")
                print()

                #print(f"Thumb (CMC) : {thumb_deg:5.1f}°")
                #print("pinky:",{ang_list[4]})

                # Palm 좌표계 TIP·CMC
                p_tip_loc = to_palm( (R_inv @ get_landmark_3d(4, hlm, depth_frame, intr_depth).reshape(3,1)+T_inv).ravel() )
                p_cmc_loc = to_palm( (R_inv @ get_landmark_3d(1, hlm, depth_frame, intr_depth).reshape(3,1)+T_inv).ravel() )
                phi = math.degrees(math.atan2(p_tip_loc[1]-p_cmc_loc[1], p_tip_loc[0]-p_cmc_loc[0]))
                #print(f"[Thumb-spread φ] {phi:5.1f}°")   # 터미널 로그
                # 히스테리시스 상태머신
                if not hasattr(main, "thumb_state"):
                    main.thumb_state = "CLOSE"
                if   abs(phi) > TH_OPEN:   main.thumb_state = "OPEN"
                elif abs(phi) < TH_CLOSE:  main.thumb_state = "CLOSE"

                us_pos  = THUMB_POS_OPEN  if main.thumb_state=="OPEN" else THUMB_POS_CLOSE   # 서보0
                                                      # 서보1


                pulses_raw = [
                us_pos,                               # Servo 0 : 엄지 벌림
                angle_to_openclose(ang_list[1]),#thumb_bend_to_pwm(thumb_deg),      # Servo 1 : 엄지 굽힘
                angle_to_openclose(ang_list[1]),      # Servo 2 : 검지
                angle_to_openclose(ang_list[1]),      # Servo 3 : 중지
                angle_to_openclose(ang_list[1]),      # Servo 4 : 약지
                angle_to_openclose(ang_list[1])       # Servo 5 : 새끼
            ]
                prev_state = None            # None / 0 / 1

                state = 1 if angle_to_openclose(ang_list[1]) == 1650 else 0
                if lock_pressed and state != prev_state:
                    arduino.write(b"a\n" if state else b"b\n")
                    prev_state = state
                    #arduino.write( (",".join(map(str, pulses_raw)) + "\n").encode() )                    
                #print("[Servo µs] ", pulses_raw)

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
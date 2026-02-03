-- map_builder.lua와 trajectory_builder.lua 설정 파일을 포함
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                    -- 맵 빌더 설정 (2D 또는 3D 선택 가능)
  trajectory_builder = TRAJECTORY_BUILDER,      -- 궤적 빌더 설정 (주행 경로 생성 방식)

  map_frame = "map",                            -- SLAM이 생성한 전역 맵 프레임 이름
  -- tracking_frame = "imu_link",               -- (사용 안 함) IMU 기준 좌표 프레임
  tracking_frame = "camera_gyro_frame",                 -- 로봇의 위치 추적에 사용하는 프레임 (IMU 없을 경우 base_link 사용)
  -- tracking_frame = "camera_gyro_link222",
  published_frame = "base_footprint",                -- pose가 퍼블리시될 기준 프레임, 원래는 base_link
  odom_frame = "odom",                          -- 로봇의 로컬 오도메트리 프레임
  provide_odom_frame = true,                    -- odom -> tracking_frame(tf 연결)을 cartographer가 제공할지 여부

  use_odometry = false,                         -- 오도메트리 데이터를 사용할지 여부
  use_nav_sat = false,                          -- GPS(nav_sat) 데이터를 사용할지 여부
  use_landmarks = false,                        -- 랜드마크 기반 위치 추정 사용 여부

  publish_frame_projected_to_2d = true,         -- 3D 위치 데이터를 2D로 투영하여 퍼블리시할지 여부

  num_laser_scans = 1,                          -- 사용되는 2D 라이다 수
  num_multi_echo_laser_scans = 0,               -- multi-echo 라이다 수 (사용 안함)
  num_subdivisions_per_laser_scan = 1,          -- 라이다 스캔 1회당 subdivide 횟수 (값이 클수록 더 부드러우나 계산량 증가)
  num_point_clouds = 0,                         -- 포인트 클라우드 센서 수 (D435 등 depth sensor용)

  lookup_transform_timeout_sec = 0.2,           -- TF 조회 타임아웃 시간 (초)
  submap_publish_period_sec = 0.3,              -- 서브맵 퍼블리시 주기 (초)
  pose_publish_period_sec = 5e-3,               -- pose 퍼블리시 주기 (초) = 5ms
  trajectory_publish_period_sec = 30e-3,        -- 궤적 퍼블리시 주기 (초) = 30ms

  -- 샘플링 비율 설정: 낮추면 일부 데이터만 사용해 계산량 줄임
  rangefinder_sampling_ratio = 1.0,             -- 라이다 샘플링 비율 (1.0 = 모든 데이터 사용)
  odometry_sampling_ratio = 1.0,                -- 오도메트리 샘플링 비율
  imu_sampling_ratio = 1.0,                     -- IMU 샘플링 비율
  fixed_frame_pose_sampling_ratio = 1.0,        -- 외부 위치 기준 샘플링 비율 (GPS 등)
  landmarks_sampling_ratio = 1.0,               -- 랜드마크 샘플링 비율
}

MAP_BUILDER.use_trajectory_builder_2d = true     -- 2D SLAM을 사용할 것인지 여부 설정
TRAJECTORY_BUILDER_2D.use_imu_data = true       -- 2D 궤적 빌더에서 IMU 데이터 사용 여부

-- 선택사항: 실시간 상관관계 스캔 매칭을 사용 (정확도 ↑, 연산량 ↑)
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

return options

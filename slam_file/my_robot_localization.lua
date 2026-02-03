include "my_robot.lua"

-- localization 모드 설정
MAP_BUILDER.use_trajectory_builder_2d = true

-- SLAM이 아닌 localization을 위한 설정
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

-- SLAM 전용 scan-matching 비활성화 (선택)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

return options
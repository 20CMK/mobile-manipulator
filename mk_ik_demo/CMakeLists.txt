cmake_minimum_required(VERSION 3.0.2)
project(mk_ik_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  eigen_conversions
  serial
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mk_test src/test_original.cpp)
target_link_libraries(mk_test
  ${catkin_LIBRARIES}
)

add_executable(mk_pub src/pub.cpp)
target_link_libraries(mk_pub
  ${catkin_LIBRARIES} 
)

add_executable(mk_cartesian src/test_cartesian.cpp)
target_link_libraries(mk_cartesian
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(mk_cartesian_mobile src/test_cartesian_mobile.cpp)
target_link_libraries(mk_cartesian_mobile
  ${catkin_LIBRARIES}
)

add_executable(mk src/mk.cpp)
target_link_libraries(mk
  ${catkin_LIBRARIES}
)
add_dependencies(mk ${catkin_EXPORTED_TARGETS})


add_executable(mk_low_mobile src/test_low_control_mobile.cpp)
target_link_libraries(mk_low_mobile
  ${catkin_LIBRARIES}
)


add_executable(mk_low src/test_low_control.cpp)
target_link_libraries(mk_low
  ${catkin_LIBRARIES}
)

add_executable(test_wdls src/test_wdls.cpp)
target_link_libraries(test_wdls
  ${catkin_LIBRARIES}
)

add_executable(test_wdls_v2 src/test_wdls_v2.cpp)
target_link_libraries(test_wdls_v2
  ${catkin_LIBRARIES}
)


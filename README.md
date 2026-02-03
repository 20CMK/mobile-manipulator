# 원격 조종 기반 다목적 이동 로봇
## 시스템 개요
개요
- 비정형, 고위험 환경의 작업자 안전 확보
- 기존 Leader/Follower 방식과 차별화된 Physical AI 플랫폼 개발

목표 : 사람 손의 동작을 실시간으로 모방하는 시스템 개발

시스템구성
- H/W 구성 : 6DoF Robotic Hand + 6DoF Robot Arm + Mobile robot
- S/W 구조
  - (기능1) Pick & Place : 물체 인식 > 좌표 변환 > Motion Planning
  - (기능2) Vision Teleoperation : 손목 Pose 인식 (위치: RGB-D, 방향: IMU) > IK 연산 > Sim 구동 > 실제 로봇 구동
- 환경 : ROS noetic (Ubuntu 20.04)
- System Architecture : Main PC(주 연산) - SBC(무선 중계) - MCU(센서, 구동기 처리)
  - openCR : 로봇팔 및 eep 구동용
  - stm32 : 주행부 구동용

 (St)기능2를 위한 손목 인식에 사용 / (2)자율주행 및 기능1에 사용

## 실행방법

외부 설치 git 패키지

- realsense-ros
- rplidar-ros
- multimaster_fkie
- cartographer
  - 해당 Package는 별도 ws에 빌드
  - 설치 후 slam_file의 file 추가

```
# terminal 1
roslaunch manipulator_moveit_config test.launch

# terminal 2
roslaunch realsense-ros rs_camera.launch
```
```
# terminal 3 - Test용 Topic 발행
rosrun mk_pkg test_publish_target_pose.py
```



## 실제 로봇 구동
```
# terminal 1
rosrun rosserial_python serial_node.py /dev/ttyACM* __name:=rosserial_opencr

# terminal 2
rosrun rosserial_python serial_node.py /dev/ttyACM* __name:=rosserial_stm32
```
udev-rule 설정으로 포트 이름 지정 가능



## 자율주행
```
#SLAM
roslaunch cartographer-ros my_robot.launch

#Navigation
roslaunch manipulator_mobile_description display.launch
roslaunch cartographer-ros my_robot_localization.launch
roslaunch manipulator_mobile_navigation move_base.launch
```
주의사항

- Navigation
  - cartographer-ros 패키지와 다른 패키지의 ws가 다름을 유의하여 빌드 및 setup 후 launch 진행




[cartographer]([https://google-cartographer.readthedocs.io/en/latest/](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html))
[cartographer-git](https://github.com/cartographer-project/cartographer_ros)

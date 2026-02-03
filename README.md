# 원격 조종 기반 다목적 이동 로봇

개요 :

목표 : 

시스템구성
- H/W 구성 : 6DoF Robotic Hand + 6DoF Robot Arm + Mobile robot
  - Robot Arm : Dynamicel MX64-T
  - 
- 통신 및 연산 Architecture : 



## 실행방법
```
# terminal 1
roslaunch manipulator_moveit_config test.launch

# terminal 2
roslaunch realsense-ros rs_camera.launch
```


```
rosrun rosserial_python serial_node.py /dev/ttyACM1 __name:=custom_node_name

#또는 udev-rule 설정 이름 지정
rosrun rosserial_python serial_node.py /dev/opencr __name:=custom_node_name
```


필요 git 패키지
realsense-ros
rplidar-ros
multimaster_fkie

cartographer

[cartographer]([https://google-cartographer.readthedocs.io/en/latest/](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html))
[cartographer-git](https://github.com/cartographer-project/cartographer_ros)

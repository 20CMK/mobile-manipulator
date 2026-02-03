#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
ros::NodeHandle nh;

void angleCallback(const std_msgs::Int16MultiArray& msg) {
  // msg.data[0] ~ msg.data[5] 입력
  for (int i = 0; i < 6; i++) {
    int pulse = msg.data[i];        // 이미 변환된 펄스폭(130~510 등)
    pwm.setPWM(i, 0, pulse);        // PCA9685 채널 i 로 쓰기
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("angle_sensor6", angleCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  Wire.begin();
  Wire.setClock(400000);  // ★ 400kHz I2C 속도 (강력 추천)

  pwm.begin();
  pwm.setPWMFreq(50);     // servo: 50Hz
}

void loop() {
  nh.spinOnce();
}

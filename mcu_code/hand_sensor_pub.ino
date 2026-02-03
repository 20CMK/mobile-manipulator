#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

std_msgs::Int16MultiArray angle_msg;
ros::Publisher pub_angle("angle_sensor_raw", &angle_msg);

void setup() {
  nh.initNode();
  nh.advertise(pub_angle);

  // 배열 크기 6개 미리 지정
  angle_msg.data_length = 6;
  angle_msg.data = (int16_t*)malloc(sizeof(int16_t) * 6);
}

void loop() {
  // A0~A5 읽기(0~1023)
  angle_msg.data[0] = analogRead(A0);
  angle_msg.data[1] = analogRead(A1);
  angle_msg.data[2] = analogRead(A2);
  angle_msg.data[3] = analogRead(A3);
  angle_msg.data[4] = analogRead(A4);
  angle_msg.data[5] = analogRead(A5);

  pub_angle.publish(&angle_msg);

  nh.spinOnce();
  delay(20);  // 50Hz 발행
}

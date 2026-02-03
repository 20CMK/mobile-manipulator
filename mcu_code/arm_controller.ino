#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <Dynamixel2Arduino.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define PI 3.14159265
using namespace ControlTableItem;

#define DEBUG_SERIAL Serial
#define DXL_SERIAL Serial3
#define DXL_DIR_PIN 84

// ========== Dynamixel, ROS 통신 설정 ==========
#define ADDR_MX_HOMING_OFFSET 20

const uint8_t DXL_ID[] = {1, 2, 3, 4, 5, 6};
const float DXL_PROTOCOL_VERSION = 2.0;
float target_position[6] = {0};     // deg 기준
int target_velocity[6] = {75, 75, 75, 20, 20, 20};

// 수신된 ROS 토픽 각도 저장
float received_position_rad[6] = {0.0};
float received_position_deg[6] = {0.0};

// PID 값
const uint16_t POSITION_P_GAIN_VALUE = 100;
const uint16_t POSITION_I_GAIN_VALUE = 200;
const uint16_t POSITION_D_GAIN_VALUE = 800;

// 객체 선언
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
HardwareSerial &rosSerial = Serial1;
ros::NodeHandle nh;

// ========== 변환 함수 ==========
int angleToDxlPosition(float angle, float gear_reduction) {
  float step_per_degree = 4096.0 / 360.0;
  return static_cast<int>((angle * step_per_degree) * gear_reduction);
}

// ========== Homing Offset 관련 ==========
int32_t readHomingOffset(uint8_t id) {
  int32_t offset;
  uint8_t buf[4];
  if (dxl.read(id, ADDR_MX_HOMING_OFFSET, sizeof(buf), buf, sizeof(buf), 100)) {
    offset = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
    return offset;
  }
  DEBUG_SERIAL.print("Failed to read homing offset for ID ");
  DEBUG_SERIAL.println(id);
  return 0;
}

int32_t readCurrentPositionRaw(uint8_t id) {
  int32_t pos;
  uint8_t buf[4];
  if (dxl.read(id, 132, sizeof(buf), buf, sizeof(buf), 100)) {
    pos = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
    return pos;
  }
  DEBUG_SERIAL.println("Failed to read position.");
  return 0;
}

void setHomingOffset(uint8_t id, int32_t offset) {
  uint8_t buf[4];
  buf[0] = (offset >> 0) & 0xFF;
  buf[1] = (offset >> 8) & 0xFF;
  buf[2] = (offset >> 16) & 0xFF;
  buf[3] = (offset >> 24) & 0xFF;
  if (!dxl.write(id, ADDR_MX_HOMING_OFFSET, buf, 4)) {
    DEBUG_SERIAL.print("Failed to set offset for ID ");
    DEBUG_SERIAL.println(id);
  } else {
    DEBUG_SERIAL.print("Set homing offset for ID ");
    DEBUG_SERIAL.println(id);
  }
}

// ========== ROS 콜백 ==========
void jointStateCallback(const sensor_msgs::JointState &msg) {
  for (size_t i = 0; i < 6; i++) {
    received_position_rad[i] = msg.position[i];
    received_position_deg[i] = msg.position[i] * (180.0 / PI);
    target_position[i] = received_position_deg[i];

    float vel_map = msg.velocity[i] * 100;
    // target_velocity[i] = map(abs(vel_map), 0, 100, 10, 300);
    target_velocity[i] = map(abs(vel_map), 0, 100, 10, (i < 3 ? 300 : 50)); //10이 최적
  }
}

ros::Subscriber<sensor_msgs::JointState> jointStateSub("joint_states", &jointStateCallback);

std_msgs::Float32MultiArray pos_msg;
ros::Publisher pos_pub("motor_position", &pos_msg);

// ========== SETUP ==========
void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

  rosSerial.begin(57600);
  nh.initNode();
  nh.subscribe(jointStateSub);
  nh.advertise(pos_pub);

  pos_msg.data_length = 6;
  pos_msg.data = (float*)malloc(sizeof(float) * 6);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  delay(1000);

  for (int i = 0; i < 6; i++) {
    if (dxl.ping(DXL_ID[i])) {
      dxl.torqueOff(DXL_ID[i]);
      dxl.setOperatingMode(DXL_ID[i], OP_EXTENDED_POSITION);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], 100);
      dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID[i], POSITION_P_GAIN_VALUE);
      dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID[i], POSITION_I_GAIN_VALUE);
      dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID[i], POSITION_D_GAIN_VALUE);
      dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID[i], 1023);
      dxl.writeControlTableItem(TORQUE_LIMIT, DXL_ID[i], 1000);

      int32_t prev = readHomingOffset(DXL_ID[i]);
      int32_t curr = readCurrentPositionRaw(DXL_ID[i]);
      int32_t new_offset = -(curr - prev);
      setHomingOffset(DXL_ID[i], new_offset);

      dxl.torqueOn(DXL_ID[i]);
    } else {
      DEBUG_SERIAL.print("Ping failed for ID ");
      DEBUG_SERIAL.println(DXL_ID[i]);
    }
  }
}

// ========== LOOP ==========
void loop() {
  nh.spinOnce();

  for (int i = 0; i < 6; i++) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], target_velocity[i]);
    int pos_tick = angleToDxlPosition(target_position[i], i < 3 ? 6.0 : 1.0);
    dxl.setGoalPosition(DXL_ID[i], pos_tick);

    int32_t raw_pos = readCurrentPositionRaw(DXL_ID[i]);
    float gear_reduction = (i < 3) ? 6.0 : 1.0;
    float deg = (raw_pos / gear_reduction) * (360.0 / 4096.0);
    float rad = deg * PI / 180.0;

    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(DXL_ID[i]);

    DEBUG_SERIAL.print(" | Target: ");
    DEBUG_SERIAL.print(received_position_rad[i], 4);  // rad
    DEBUG_SERIAL.print(" rad / ");
    DEBUG_SERIAL.print(received_position_deg[i], 2);  // deg

    DEBUG_SERIAL.print(" -> Current: ");
    DEBUG_SERIAL.print(deg, 2);
    DEBUG_SERIAL.println(" deg");

    pos_msg.data[i] = target_velocity[i];
  }


  pos_pub.publish(&pos_msg);
}

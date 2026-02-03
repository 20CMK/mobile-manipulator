// #include <ros/ros.h>
// #include <sensor_msgs/JointState.h>

// ros::Publisher throttled_pub;
// sensor_msgs::JointState latest_msg;
// bool new_msg_available = false;

// // Callback function to receive messages from /joint_states
// void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//     latest_msg = *msg;  // Store the latest message
//     new_msg_available = true;
// }

// int main(int argc, char** argv) {
//     // Initialize the ROS node
//     ros::init(argc, argv, "joint_states_throttle");
//     ros::NodeHandle nh;

//     // Subscriber to /joint_states
//     ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);

//     // Publisher to /joint_states_throttled
//     throttled_pub = nh.advertise<sensor_msgs::JointState>("/joint_states_throttled", 10);

//     // Set the loop rate to 10Hz
//     ros::Rate loop_rate(10);

//     while (ros::ok()) {
//         // If a new message is available, publish it
//         if (new_msg_available) {
//             throttled_pub.publish(latest_msg);
//             new_msg_available = false;  // Reset the flag
//         }

//         // Process callbacks and sleep to maintain the loop rate
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }

//////////////////////////////////////////

// #include <ros/ros.h>
// #include <sensor_msgs/JointState.h>
// #include <cmath> // for std::round

// ros::Publisher throttled_pub;
// sensor_msgs::JointState latest_msg;
// bool new_msg_available = false;

// // 함수: 소수점 둘째 자리로 반올림
// double roundToTwoDecimalPlaces(double value) {
//     return std::round(value * 100.0) / 100.0;
// }

// // Callback function to receive messages from /joint_states
// void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//     latest_msg = *msg;  // Store the latest message

//     // velocity 값을 소수 둘째 자리로 반올림
//     for (size_t i = 0; i < latest_msg.velocity.size(); ++i) {
//         latest_msg.velocity[i] = roundToTwoDecimalPlaces(latest_msg.velocity[i]);
//     }

//     new_msg_available = true;
// }

// int main(int argc, char** argv) {
//     // Initialize the ROS node
//     ros::init(argc, argv, "joint_states_throttle");
//     ros::NodeHandle nh;

//     // Subscriber to /joint_states
//     ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);

//     // Publisher to /joint_states_throttled
//     throttled_pub = nh.advertise<sensor_msgs::JointState>("/joint_states_throttled", 10);

//     // Set the loop rate to 10Hz
//     ros::Rate loop_rate(10);

//     while (ros::ok()) {
//         // If a new message is available, publish it
//         if (new_msg_available) {
//             throttled_pub.publish(latest_msg);
//             new_msg_available = false;  // Reset the flag
//         }

//         // Process callbacks and sleep to maintain the loop rate
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }





#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cmath> // for std::round

ros::Publisher throttled_pub;
sensor_msgs::JointState latest_msg;
bool new_msg_available = false;

// 함수: 소수점 둘째 자리로 반올림
double roundToTwoDecimalPlaces(double value) {
    return std::round(value * 100.0) / 100.0;
}

// Callback function to receive messages from /joint_states
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    latest_msg = *msg;  // Store the latest message

    // position 값을 소수 둘째 자리로 반올림
    for (size_t i = 0; i < latest_msg.position.size(); ++i) {
        latest_msg.position[i] = roundToTwoDecimalPlaces(latest_msg.position[i]);
    }

    // velocity 값을 소수 둘째 자리로 반올림
    for (size_t i = 0; i < latest_msg.velocity.size(); ++i) {
        latest_msg.velocity[i] = roundToTwoDecimalPlaces(latest_msg.velocity[i]);
    }

    new_msg_available = true;
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "joint_states_throttle");
    ros::NodeHandle nh;

    // Subscriber to /joint_states
    ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);

    // Publisher to /joint_states_throttled
    throttled_pub = nh.advertise<sensor_msgs::JointState>("/joint_states_throttled", 10);

    // Set the loop rate to 10Hz
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // If a new message is available, publish it
        if (new_msg_available) {
            throttled_pub.publish(latest_msg);
            new_msg_available = false;  // Reset the flag
        }

        // Process callbacks and sleep to maintain the loop rate
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

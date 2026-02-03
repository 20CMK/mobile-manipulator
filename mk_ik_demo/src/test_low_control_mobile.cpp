// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/robot_state/robot_state.h>
// #include <Eigen/LU>  // For rank calculation
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <cmath>  // M_PI 사용 가능하게
// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <ros/topic.h>  // 요거 꼭 추가!


// geometry_msgs::Pose target_pose;
// // Pose 콜백 함수
// // class PoseSubscriber
// // {
// // private:
// // public:
// //     void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
// //     {

//     //     // waypoints[1]에 Pose의 xyz 값 저장
//     //     target_pose.position.x = msg->position.x;
//     //     target_pose.position.y = msg->position.y;
//     //     target_pose.position.z = msg->position.z;
//     //     target_pose.orientation.x = msg->orientation.x;
//     //     target_pose.orientation.y = msg->orientation.y;
//     //     target_pose.orientation.z = msg->orientation.z;
//     //     target_pose.orientation.w = msg->orientation.w;

//     //     // Waypoints의 갱신된 값을 출력 (디버깅용)
//     //     ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
//     //         target_pose.position.x,
//     //         target_pose.position.y,
//     //         target_pose.position.z,
//     //         target_pose.orientation.x,
//     //         target_pose.orientation.y,
//     //         target_pose.orientation.z,
//     //         target_pose.orientation.w);
   

//         // std::string base_frame = "base_link";
//         // moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//         // visual_tools.deleteAllMarkers();
//         // // visual_tools.loadRemotecontrol();
//         // visual_tools.publishAxisLabeled(target_pose, "target_pose");
//         // visual_tools.trigger();
// //     }
// // };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "mk_low");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(2);
//     spinner.start();

//     // PoseSubscriber pose_subscriber;
//     // ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);
//     ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);

//     const std::string PLANNING_GROUP = "arm";  // 사용 중인 planning group 이름

//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();
//     const robot_state::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);
//     const std::string eef_link = move_group.getEndEffectorLink();





//     std::string base_frame = "base_link";
//     moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//     visual_tools.deleteAllMarkers();
//     // visual_tools.loadRemotecontrol();
//     visual_tools.publishAxisLabeled(target_pose, "target_pose");
//     visual_tools.trigger();





//     const double TIMEOUT = 0.1;
//     bool found_ik = false;

//     while(ros::ok()){


//         geometry_msgs::PoseConstPtr msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/target_pose", nh, ros::Duration());


//         if (!msg)
//         {
//             ROS_WARN("No target_pose received within timeout.");
//             break;
//         }

//         target_pose.position.x = msg->position.x;
//         target_pose.position.y = msg->position.y;
//         target_pose.position.z = msg->position.z;
//         target_pose.orientation.x = msg->orientation.x;
//         target_pose.orientation.y = msg->orientation.y;
//         target_pose.orientation.z = msg->orientation.z;
//         target_pose.orientation.w = msg->orientation.w;

//         std::string base_frame = "base_link";
//         moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//         visual_tools.deleteAllMarkers();
//         // visual_tools.loadRemotecontrol();
//         visual_tools.publishAxisLabeled(target_pose, "target_pose");
//         visual_tools.trigger();

//         // Waypoints의 갱신된 값을 출력 (디버깅용)
//         ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
//             target_pose.position.x,
//             target_pose.position.y,
//             target_pose.position.z,
//             target_pose.orientation.x,
//             target_pose.orientation.y,
//             target_pose.orientation.z,
//             target_pose.orientation.w);



//         found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, TIMEOUT);
//         if (found_ik)
//         {
//             Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
//             Eigen::MatrixXd jacobian;

//             kinematic_state->getJacobian(
//                 joint_model_group,
//                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
//                 reference_point,
//                 jacobian);

//             int rank = Eigen::FullPivLU<Eigen::MatrixXd>(jacobian).rank();
//             int dof = joint_model_group->getVariableCount();

//             ROS_INFO_STREAM("Jacobian rank: " << rank << " / DOF: " << dof);

//             if (rank == dof)
//             {
//                 std::vector<double> joint_values;
//                 kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

//                 ROS_INFO("good_good_good_good_good_good_good");
//                 for (size_t i = 0; i < joint_values.size(); ++i)
//                 {
//                     double deg = joint_values[i] * (180.0 / M_PI);  // 라디안 → 도
//                     ROS_INFO("Joint %lu: %f deg", i, deg);
//                 }
//                 // 관절 trajectory 메시지 생성
//                 trajectory_msgs::JointTrajectory traj;
//                 traj.joint_names = joint_model_group->getVariableNames();  // 조인트 이름들 자동으로 가져오기

//                 trajectory_msgs::JointTrajectoryPoint point;
//                 point.positions = joint_values;
//                 point.time_from_start = ros::Duration(0.8);  // 1초 안에 도달

//                 traj.points.push_back(point);

//                 // 퍼블리시
//                 traj_pub.publish(traj);
//                 ROS_INFO("Published joint trajectory to /arm_controller/command");


//             }
//             else
//             {
//                 ROS_WARN("----------------Singularity----------------------");
//             }
//         }
//         else
//         {
//             ROS_WARN("IK failed!!!IK failed!!!IK failed!!!IK failed!!!");
//         }
//     }


        
        



//     // if (!found_ik)
//     // {
//     //     ROS_ERROR("Failed to find a valid IK solution that is not singular.");
//     // }

//     // ros::shutdown();
//     // return 0;
// }





// /////////////////////////////////////////////////////////////////////////////
// // v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v2v22vv2v22v2v2v2v2v2v2v2v
// /////////////////////////////////////////////////////////////////////////
// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/robot_state/robot_state.h>
// #include <Eigen/LU>  // For rank calculation
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <cmath>  // M_PI 사용 가능하게
// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <ros/topic.h>  // 요거 꼭 추가!



// double max_velocity_1_3 = 1.0;  // rad/s, 예: 감속기 적용된 값
// double max_velocity_4_6 = 6.0;  // rad/s, 예: 감속기 적용된 값


// geometry_msgs::Pose target_pose;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "mk_low");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(2);
//     spinner.start();

//     // PoseSubscriber pose_subscriber;
//     // ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);
//     ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);

//     const std::string PLANNING_GROUP = "arm";  // 사용 중인 planning group 이름

//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();
//     const robot_state::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);
//     const std::string eef_link = move_group.getEndEffectorLink();





//     std::string base_frame = "base_link";
//     moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//     visual_tools.deleteAllMarkers();
//     // visual_tools.loadRemotecontrol();
//     visual_tools.publishAxisLabeled(target_pose, "target_pose");
//     visual_tools.trigger();





//     const double TIMEOUT = 0.1;
//     bool found_ik = false;

//     while(ros::ok()){
        


//         geometry_msgs::PoseConstPtr msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/target_pose", nh, ros::Duration());


//         if (!msg)
//         {
//             ROS_WARN("No target_pose received within timeout.");
//             break;
//         }

//         target_pose.position.x = msg->position.x;
//         target_pose.position.y = msg->position.y;
//         target_pose.position.z = msg->position.z;
//         target_pose.orientation.x = msg->orientation.x;
//         target_pose.orientation.y = msg->orientation.y;
//         target_pose.orientation.z = msg->orientation.z;
//         target_pose.orientation.w = msg->orientation.w;

//         std::string base_frame = "base_link";
//         moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//         visual_tools.deleteAllMarkers();
//         // visual_tools.loadRemotecontrol();
//         visual_tools.publishAxisLabeled(target_pose, "target_pose");    //loop 안에는 이 둘만 있으면 된다?
//         visual_tools.trigger(); // loop안에는 이 둘만 있으면 된다?

//         // Waypoints의 갱신된 값을 출력 (디버깅용)
//         // ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
//         //     target_pose.position.x,
//         //     target_pose.position.y,
//         //     target_pose.position.z,
//         //     target_pose.orientation.x,
//         //     target_pose.orientation.y,
//         //     target_pose.orientation.z,
//         //     target_pose.orientation.w);

        
//         ros::Time start = ros::Time::now();
//         found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, TIMEOUT);
//         if (found_ik)
//         {
//             Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
//             Eigen::MatrixXd jacobian;

//             kinematic_state->getJacobian(
//                 joint_model_group,
//                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
//                 reference_point,
//                 jacobian);

//             int rank = Eigen::FullPivLU<Eigen::MatrixXd>(jacobian).rank();
//             int dof = joint_model_group->getVariableCount();

//             // ROS_INFO_STREAM("Jacobian rank: " << rank << " / DOF: " << dof);

//             if (rank == dof)
//             {
//                 std::vector<double> joint_values;
//                 kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

//                 // ROS_INFO("good_good_good_good_good_good_good");
//                 // for (size_t i = 0; i < joint_values.size(); ++i)
//                 // {
//                 //     double deg = joint_values[i] * (180.0 / M_PI);  // 라디안 → 도
//                 //     ROS_INFO("Joint %lu: %f deg", i, deg);
//                 // }


//                 // 관절 trajectory 메시지 생성
//                 trajectory_msgs::JointTrajectory traj;
//                 traj.joint_names = joint_model_group->getVariableNames();  // 조인트 이름들 자동으로 가져오기


//                 double max_required_time = 0.0;
//                 // 이전 위치와의 차이로 이동 거리 계산
//                 std::vector<double> current_joint_positions;
//                 move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, current_joint_positions);

//                 for (size_t i = 0; i < joint_values.size(); ++i)
//                 {
//                     double delta = fabs(joint_values[i] - current_joint_positions[i]);
//                     double required_time = delta / (i < 3 ? max_velocity_1_3 : max_velocity_4_6);
//                     if (required_time > max_required_time)
//                         max_required_time = required_time;
//                 }



//                 trajectory_msgs::JointTrajectoryPoint point;
//                 point.positions = joint_values;
//                 point.time_from_start = ros::Duration(max_required_time);  // 1초 안에 도달
//                 //// point.time_from_start = ros::Duration(5.0);  // 1초 안에 도달
//                 ROS_INFO("max_required_time : %f", max_required_time);

//                 traj.points.push_back(point);

//                 // 퍼블리시
//                 traj_pub.publish(traj);
//                 // ROS_INFO("Published joint trajectory to /arm_controller/command");


//             }
//             else
//             {
//                 ROS_WARN("----------------Singularity----------------------");
//             }
//         }
//         else
//         {
//             ROS_WARN("IK failed!!!IK failed!!!IK failed!!!IK failed!!!");
//         }
//         ros::Time end = ros::Time::now();
//         ros::Duration elapsed_time = end - start;
        
//         ROS_INFO("operating time: %.6f sec", elapsed_time.toSec()); 
//     }
// }









// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/robot_state/robot_state.h>
// #include <Eigen/LU>  // For rank calculation
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <cmath>  // M_PI 사용 가능하게
// #include <trajectory_msgs/JointTrajectory.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>
// #include <ros/topic.h>  // 요거 꼭 추가!



// double max_velocity_1_3 = 1.0;  // rad/s, 예: 감속기 적용된 값
// double max_velocity_4_6 = 6.0;  // rad/s, 예: 감속기 적용된 값


// geometry_msgs::Pose target_pose;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "mk_low");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(2);
//     spinner.start();

//     // PoseSubscriber pose_subscriber;
//     // ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);
//     ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);

//     const std::string PLANNING_GROUP = "arm";  // 사용 중인 planning group 이름

//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();
//     const robot_state::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);
//     const std::string eef_link = move_group.getEndEffectorLink();





//     std::string base_frame = "base_link";
//     moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//     visual_tools.deleteAllMarkers();
//     // visual_tools.loadRemotecontrol();
//     visual_tools.publishAxisLabeled(target_pose, "target_pose");
//     visual_tools.trigger();





//     const double TIMEOUT = 0.1;
//     bool found_ik = false;

//     while(ros::ok()){
        


//         geometry_msgs::PoseConstPtr msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/target_pose", nh, ros::Duration());


//         if (!msg)
//         {
//             ROS_WARN("No target_pose received within timeout.");
//             break;
//         }

//         target_pose.position.x = msg->position.x;
//         target_pose.position.y = msg->position.y;
//         target_pose.position.z = msg->position.z;
//         target_pose.orientation.x = msg->orientation.x;
//         target_pose.orientation.y = msg->orientation.y;
//         target_pose.orientation.z = msg->orientation.z;
//         target_pose.orientation.w = msg->orientation.w;

//         std::string base_frame = "base_link";
//         moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//         visual_tools.deleteAllMarkers();
//         // visual_tools.loadRemotecontrol();
//         visual_tools.publishAxisLabeled(target_pose, "target_pose");    //loop 안에는 이 둘만 있으면 된다?
//         visual_tools.trigger(); // loop안에는 이 둘만 있으면 된다?

//         // Waypoints의 갱신된 값을 출력 (디버깅용)
//         // ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
//         //     target_pose.position.x,
//         //     target_pose.position.y,
//         //     target_pose.position.z,
//         //     target_pose.orientation.x,
//         //     target_pose.orientation.y,
//         //     target_pose.orientation.z,
//         //     target_pose.orientation.w);

        
//         ros::Time start = ros::Time::now();
//         found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, TIMEOUT);
//         if (found_ik)
//         {
//             Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
//             Eigen::MatrixXd jacobian;

//             kinematic_state->getJacobian(
//                 joint_model_group,
//                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
//                 reference_point,
//                 jacobian);

//             int rank = Eigen::FullPivLU<Eigen::MatrixXd>(jacobian).rank();
//             int dof = joint_model_group->getVariableCount();

//             // ROS_INFO_STREAM("Jacobian rank: " << rank << " / DOF: " << dof);

//             if (rank == dof)
//             {
//                 std::vector<double> joint_values;
//                 kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

//                 // ROS_INFO("good_good_good_good_good_good_good");
//                 // for (size_t i = 0; i < joint_values.size(); ++i)
//                 // {
//                 //     double deg = joint_values[i] * (180.0 / M_PI);  // 라디안 → 도
//                 //     ROS_INFO("Joint %lu: %f deg", i, deg);
//                 // }


//                 // 관절 trajectory 메시지 생성
//                 trajectory_msgs::JointTrajectory traj;
//                 traj.joint_names = joint_model_group->getVariableNames();  // 조인트 이름들 자동으로 가져오기


//                 double max_required_time = 0.0;
//                 // 이전 위치와의 차이로 이동 거리 계산
//                 std::vector<double> current_joint_positions;
//                 move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, current_joint_positions);

//                 for (size_t i = 0; i < joint_values.size(); ++i)
//                 {
//                     double delta = fabs(joint_values[i] - current_joint_positions[i]);
//                     double required_time = delta / (i < 3 ? max_velocity_1_3 : max_velocity_4_6);
//                     if (required_time > max_required_time)
//                         max_required_time = required_time;
//                 }



//                 // trajectory_msgs::JointTrajectoryPoint point;
//                 // point.positions = joint_values;
//                 // point.time_from_start = ros::Duration(max_required_time);  // 1초 안에 도달
//                 // point.time_from_start = ros::Duration(5.0);  // 1초 안에 도달
//                 // // ROS_INFO("%f", max_required_time);

//                 // traj.points.push_back(point);

//                 // // 퍼블리시
//                 // traj_pub.publish(traj);
//                 // ROS_INFO("Published joint trajectory to /arm_controller/command");
//                 // 감속을 위한 세팅
//                 int num_steps = 20;  // 예: 총 5 단계로 감속
//                 // double total_time = 5.0;  // 전체 소요 시간 (초)
//                 double total_time = max_required_time * 3.0;
//                 double time_step = total_time / num_steps;

//                 for (int step = 1; step <= num_steps; ++step)
//                 {
//                     double ratio = static_cast<double>(step) / num_steps;
//                     // double alpha = static_cast<double>(step) / num_steps;
//                     // double alpha ;
//                     // if (step > num_steps / 2) alpha = pow(ratio, 0.5); 
//                     // else alpha = sin(M_PI/2 * ratio);
//                     double alpha = pow(ratio, 0.8); 
//                     trajectory_msgs::JointTrajectoryPoint point;

//                     point.positions.resize(joint_values.size());
//                     for (size_t i = 0; i < joint_values.size(); ++i)
//                     {
//                         point.positions[i] = current_joint_positions[i] + alpha * (joint_values[i] - current_joint_positions[i]);
//                     }

//                     point.time_from_start = ros::Duration(step * time_step);  // 점점 느려지는 시간 간격
//                     traj.points.push_back(point);
//                 }

//                 traj_pub.publish(traj);

//             }
//             else
//             {
//                 ROS_WARN("----------------Singularity----------------------");
//             }
//         }
//         else
//         {
//             ROS_WARN("IK failed!!!IK failed!!!IK failed!!!IK failed!!!");
//         }
//         ros::Time end = ros::Time::now();
//         ros::Duration elapsed_time = end - start;
        
//         ROS_INFO("operating time: %.6f sec", elapsed_time.toSec()); 
//     }
// }






#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/LU>  // For rank calculation
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath>  // M_PI 사용 가능하게
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/topic.h>  // 요거 꼭 추가!



double max_velocity_1_3 = 1.0;  // rad/s, 예: 감속기 적용된 값
double max_velocity_4_6 = 6.0;  // rad/s, 예: 감속기 적용된 값


geometry_msgs::Pose target_pose;
geometry_msgs::Pose before_pose;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mk_low_mobile");    //ros 노드를 초기화, 노드 시작한다고 등록하는 함수, 노드의 이름
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // PoseSubscriber pose_subscriber;
    // ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);

    const std::string PLANNING_GROUP = "arm";  // 사용 중인 planning group 이름

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();  // 현재 로봇의 상태를 담고 있는 객체를 kinematic_state에 저장

    //joint_model_group은 "arm"이라는 플래닝 그룹의 관절에 대한 정보(구조적 정보 : 이름, 순서, 타입, kinematics 등 정적인 구조 정보. 실제 joint 위치 속도 등 x) 저장
    const robot_state::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);    
    
    const std::string eef_link = move_group.getEndEffectorLink();





    const moveit::core::RobotModelConstPtr& robot_model = move_group.getRobotModel();
    planning_scene::PlanningScene planning_scene(robot_model);

    auto isStateValid = [&planning_scene](moveit::core::RobotState *state, const moveit::core::JointModelGroup *group, const double *ik_solution) -> bool
    {
        state->setJointGroupPositions(group, ik_solution);
        return !planning_scene.isStateColliding(*state, group->getName());
    };







    std::string base_frame = "base_link";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();
    // visual_tools.loadRemotecontrol();
    visual_tools.publishAxisLabeled(target_pose, "target_pose");
    visual_tools.trigger();





    const double TIMEOUT = 0.1;
    bool found_ik = false;

    while(ros::ok()){
        


        geometry_msgs::PoseConstPtr msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/target_pose", nh, ros::Duration());


        if (!msg)
        {
            ROS_WARN("No target_pose received within timeout.");
            break;
        }

        target_pose.position.x = msg->position.x;
        target_pose.position.y = msg->position.y;
        target_pose.position.z = msg->position.z;
        target_pose.orientation.x = msg->orientation.x;
        target_pose.orientation.y = msg->orientation.y;
        target_pose.orientation.z = msg->orientation.z;
        target_pose.orientation.w = msg->orientation.w;

        // std::string base_frame = "base_link";
        // moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
        visual_tools.deleteAllMarkers();
        // visual_tools.loadRemotecontrol();
        visual_tools.publishAxisLabeled(target_pose, "target_pose");    //loop 안에는 이 둘만 있으면 된다?
        visual_tools.trigger(); // loop안에는 이 둘만 있으면 된다?

        // Waypoints의 갱신된 값을 출력 (디버깅용)
        // ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f",
        //     target_pose.position.x,
        //     target_pose.position.y,
        //     target_pose.position.z,
        //     target_pose.orientation.x,
        //     target_pose.orientation.y,
        //     target_pose.orientation.z,
        //     target_pose.orientation.w);

        
        ros::Time start = ros::Time::now(); //연산 시간 측정용


        // found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, TIMEOUT);
        // found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, TIMEOUT, moveit::core::GroupStateValidityCallbackFn());
        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, TIMEOUT, isStateValid);

        if (before_pose == target_pose) found_ik = 0;   // target_pose를 한번만 발행해도 여러번 ik를 풀어냄. 따라서 이전 포즈와 비교해서 한번만 실행되게
        if (found_ik)
        {
            planning_scene.setCurrentState(*kinematic_state);  // ← 여기 딱 한 줄 추가


            Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian;

            kinematic_state->getJacobian(
                joint_model_group,
                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                reference_point,
                jacobian);

            int rank = Eigen::FullPivLU<Eigen::MatrixXd>(jacobian).rank();
            int dof = joint_model_group->getVariableCount();

            // ROS_INFO_STREAM("Jacobian rank: " << rank << " / DOF: " << dof);

            if (rank == dof)
            {
                // kinematic_state = move_group.getCurrentState();

                std::vector<double> joint_values;
                kinematic_state->copyJointGroupPositions(joint_model_group, joint_values); //kinematic_state에 로봇에 있는 로봇의 전체 상태값 중, joint_model_group에 있는 관절의 값만 joint_values 값에 저장

                // ROS_INFO("good_good_good_good_good_good_good");
                // for (size_t i = 0; i < joint_values.size(); ++i)
                // {
                //     double deg = joint_values[i] * (180.0 / M_PI);  // 라디안 → 도
                //     ROS_INFO("Joint %lu: %f deg", i, deg);
                // }


                // 관절 trajectory 메시지 생성
                trajectory_msgs::JointTrajectory traj;
                traj.joint_names = joint_model_group->getVariableNames();  // 조인트 이름들 자동으로 가져오기


                double max_required_time = 0.0;
                // 이전 위치와의 차이로 이동 거리 계산
                std::vector<double> current_joint_positions;
                move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, current_joint_positions);

                for (size_t i = 0; i < joint_values.size(); ++i)
                {
                    double delta = fabs(joint_values[i] - current_joint_positions[i]);
                    // double required_time = delta / (i < 3 ? max_velocity_1_3 : max_velocity_4_6);
                    double required_time = delta / max_velocity_1_3;
                    if (required_time > max_required_time)
                        max_required_time = required_time;
                }
                
                if (max_required_time < 1.0) {
                    ROS_INFO("required_time : %f", max_required_time);
                    max_required_time = 1.0;
                }


                // trajectory_msgs::JointTrajectoryPoint point;
                // point.positions = joint_values;
                // point.time_from_start = ros::Duration(max_required_time);  // 1초 안에 도달
                // point.time_from_start = ros::Duration(5.0);  // 1초 안에 도달
                // // ROS_INFO("%f", max_required_time);

                // traj.points.push_back(point);

                // // 퍼블리시
                // traj_pub.publish(traj);
                // ROS_INFO("Published joint trajectory to /arm_controller/command");
                // 감속을 위한 세팅
                int num_steps = 120;  // 예: 총 5 단계로 감속
                // double total_time = max_required_time;
                double time_step = max_required_time / num_steps;
                double tunning_time = 0.0;
                double before_time = 0.0;
                // double time_interval 
                double tunning = 0.0;
                for (int step = 1; step <= num_steps; ++step)
                {
                    double ratio = static_cast<double>(step) / num_steps;
                    // double alpha = static_cast<double>(step) / num_steps;
                    // double alpha ;
                    // if (step > num_steps / 2) alpha = pow(ratio, 0.5); 
                    // else alpha = sin(M_PI/2 * ratio);
                    double alpha = ratio; 
                    trajectory_msgs::JointTrajectoryPoint point;

                    point.positions.resize(joint_values.size());
                    for (size_t i = 0; i < joint_values.size(); ++i)
                    {
                        point.positions[i] = current_joint_positions[i] + alpha * (joint_values[i] - current_joint_positions[i]);
                    }
                    // tunning = pow(step - (num_steps/2), 2);
                    
                    if (step <= num_steps/2) {
                        tunning = 100 * pow(2, -step);    //2.5랑 6이랑 시간 차이 많이 안나는듯
                        tunning = (tunning / pow((num_steps/2), 2)) * time_step * 5;
                    }
                    else {
                        tunning = pow(step - (num_steps/2), 2);
                        tunning = (tunning / pow((num_steps/2), 2)) * time_step * 5;
                    }
                    // if (tunning < 0.0) tunning = 0.0 ;



                    // tunning = pow(step - (num_steps/2), 2);
                    // tunning = (tunning / pow((num_steps/2), 2)) * time_step * 5;

                    
                    tunning_time = tunning_time + time_step + tunning;
                    point.time_from_start = ros::Duration(tunning_time);  // 점점 느려지는 시간 간격
                    // point.velocities.resize(joint_values.size(), 0.0);  // 정지 상태로 도달, 진동 개 심해지는데?
                    traj.points.push_back(point);
                    // before_time = tunning_time

                    before_pose = target_pose;
                }
                ROS_INFO("tunning_time / max_required_time : %f", tunning_time / max_required_time);
                ROS_INFO("max_required_time : %f", max_required_time);

                traj_pub.publish(traj);

            }
            else
            {
                ROS_WARN("----------------Singularity----------------------");
            }
        }
        else
        {
            ROS_WARN("IK failed!!!IK failed!!!IK failed!!!IK failed!!!");
        }
        ros::Time end = ros::Time::now();
        ros::Duration elapsed_time = end - start;
        
        // ROS_INFO("operating time: %.6f sec", elapsed_time.toSec()); 
    }
}









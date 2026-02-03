#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/LU>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/topic.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

double max_velocity = 1.0;  // rad/s


geometry_msgs::Pose target_pose;
geometry_msgs::Pose before_pose;
 int dof = 6;

ros::Publisher traj_pub;
double TIMEOUT = 0.1;
bool found_ik = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mk");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);


    const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();  // 현재 로봇의 상태를 담고 있는 객체를 kinematic_state에 저장


    //joint_model_group은 "arm"이라는 플래닝 그룹의 관절에 대한 정보(구조적 정보 : 이름, 순서, 타입, kinematics 등 정적인 구조 정보. 실제 joint 위치 속도 등 x) 저장
    const robot_state::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(PLANNING_GROUP);    
    const std::string eef_link = move_group.getEndEffectorLink();

    std::string base_frame = "base_link";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);

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
        
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(target_pose, "target_pose");
        visual_tools.trigger();

        ros::Time start = ros::Time::now(); //연산 시간 측정용

        kinematic_state->setVariablePositions(move_group.getCurrentJointValues());
        found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, TIMEOUT);
        bool pose_cp = true;
        if (before_pose == target_pose) pose_cp = false;   // target_pose를 한번만 발행해도 여러번 ik를 풀어냄. 따라서 이전 포즈와 비교해서 한번만 실행되게
        if (found_ik && pose_cp)
        {
            Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian;

            kinematic_state->getJacobian(
                joint_model_group,
                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                reference_point,
                jacobian);

            int rank = Eigen::FullPivLU<Eigen::MatrixXd>(jacobian).rank();
            
            if (rank == dof)
            {
                std::vector<double> joint_values;
                kinematic_state->copyJointGroupPositions(joint_model_group, joint_values); //kinematic_state에 로봇에 있는 로봇의 전체 상태값 중, joint_model_group에 있는 관절의 값만 joint_values 값에 저장

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
                    double required_time = delta / max_velocity;
                    if (required_time > max_required_time)
                        max_required_time = required_time;
                }

                traj.header.stamp = ros::Time::now();

                trajectory_msgs::JointTrajectoryPoint point;
                point.positions = joint_values;

                point.time_from_start = ros::Duration(max_required_time);
                traj.points.push_back(point);
                traj_pub.publish(traj);
                before_pose = target_pose; 

                ROS_INFO("max_required_time : %f", max_required_time);

                ros::Time end = ros::Time::now();
                ros::Duration elapsed_time = end - start;
                ROS_INFO("operating time: %.6f sec", elapsed_time.toSec()); 
            }
            else
            {
                ROS_WARN("Singularity");
            }
        }

        else {
            ROS_INFO("ik fail");
        }
         
    }
}







// ===================== 기존 include 동일 =====================
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
#include <geometry_msgs/PoseStamped.h>   // PoseStamped 사용

// ===================== 글로벌 변수 =====================
bool flag = false;

void boolCallback(const std_msgs::Bool::ConstPtr& msg)
{
    flag = msg->data;
    ROS_INFO("Received bool: %s", flag ? "true" : "false");
}

double max_velocity_1_3 = 1.0;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose before_pose;
geometry_msgs::Pose zero_pose;

double TIMEOUT = 0.1;
bool found_ik = false;
int dof = 6;

ros::Publisher traj_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mk_low");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/arm_controller/command", 10);

    ros::Publisher elapsed_time_pub =
        nh.advertise<std_msgs::Float64>("/elapsed_time", 10);

    // vision_pp_flag → /switch로 변경
    ros::Subscriber bool_sub = nh.subscribe("/switch", 10, boolCallback);

    // 초기 자세 (안 쓰더라도 그대로 둠)
    zero_pose.position.x = 0.165;
    zero_pose.position.y = 0.0;
    zero_pose.position.z = 0.86;
    zero_pose.orientation.w = 1.0;

    const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    robot_state::RobotStatePtr kinematic_state = move_group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group =
        kinematic_state->getJointModelGroup(PLANNING_GROUP);

    std::string base_frame = "base_link";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();

    while (ros::ok())
    {
        // geometry_msgs::Pose → PoseStamped, "/target_pose" → "/fusion_pose"
        geometry_msgs::PoseStamped::ConstPtr msg =
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                "/fusion_pose", nh, ros::Duration());

        if (!msg)
        {
            ROS_WARN("No fusion_pose received within timeout.");
            break;
        }

        // PoseStamped → Pose로 변환
        target_pose = msg->pose;

        // Rviz visualization update
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(target_pose, "target_pose");
        visual_tools.trigger();

        if (flag == true)
        {
            ros::Time start = ros::Time::now();
            // 현재 조인트 상태를 기준으로 IK 수행
            kinematic_state->setVariablePositions(move_group.getCurrentJointValues());

            found_ik = kinematic_state->setFromIK(
                joint_model_group, target_pose, TIMEOUT);

            bool ff = true;
            if (before_pose == target_pose) ff = false;

            if (found_ik && ff)
            {
                Eigen::Vector3d reference_point(0, 0, 0);
                Eigen::MatrixXd jacobian;

                kinematic_state->getJacobian(
                    joint_model_group,
                    kinematic_state->getLinkModel(
                        joint_model_group->getLinkModelNames().back()),
                    reference_point,
                    jacobian);

                int rank = Eigen::FullPivLU<Eigen::MatrixXd>(jacobian).rank();

                if (rank == dof)
                {
                    // ===== IK 결과 조인트값 =====
                    std::vector<double> joint_values;
                    kinematic_state->copyJointGroupPositions(
                        joint_model_group, joint_values);

                    // 현재 조인트
                    std::vector<double> cur_joints;
                    move_group.getCurrentState()->copyJointGroupPositions(
                        joint_model_group, cur_joints);

                    // ===== trajectory 생성 (보간 없이 단일 포인트) =====
                    trajectory_msgs::JointTrajectory traj;
                    traj.joint_names = joint_model_group->getVariableNames();

                    double max_required_time = 0.0;
                    for (size_t i = 0; i < joint_values.size(); i++)
                    {
                        double delta = std::fabs(joint_values[i] - cur_joints[i]);
                        double req = delta / max_velocity_1_3;
                        if (req > max_required_time)
                            max_required_time = req;
                    }

                    // 최소 이동 시간 보장 (너무 짧으면 튐)
                    if (max_required_time < 0.01)
                        max_required_time = 0.01;

                    // ★ 보간 삭제: 최종 목표 조인트 한 점만 설정
                    trajectory_msgs::JointTrajectoryPoint point;
                    point.positions = joint_values;
                    point.time_from_start = ros::Duration(max_required_time);
                    traj.points.push_back(point);

                    ROS_INFO("max_required_time : %f", max_required_time);

                    // 너무 오래 걸리는 궤적이면 스킵
                    if (max_required_time < 3.5)
                    {
                        traj_pub.publish(traj);
                        before_pose = target_pose;
                    }
                }
                else
                {
                    ROS_WARN("----- Singularity detected -----");
                }
            }
            else
            {
                ROS_WARN("IK failed!!!");
            }
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "mk_low stop!!!");
        }
    }

    return 0;
}

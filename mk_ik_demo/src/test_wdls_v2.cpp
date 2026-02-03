// ===================== 기존 include 동일 =====================
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/LU>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cmath>
#include <vector>
#include <initializer_list>
#include <stdexcept>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/topic.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>




// ===================== 글로벌 변수 및 함수 정의 =====================
inline std::vector<double> deg2rad(std::initializer_list<double> deg);

bool switch_flag = false;
bool pick_flag = false;
bool place_flag = false;

double max_velocity_1_3 = 1.0;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose before_pose;
geometry_msgs::Pose zero_pose;

double TIMEOUT = 0.1;
bool found_ik = false;
int dof = 6;

ros::Publisher traj_pub;

std::vector<std::string> joint_names;

std::vector<double> ready_pos = deg2rad({-90, 10, 110, 100, 90, 0});

void publishTraj(const std::vector<double>& joints_pos, double duration_time);
void switchCallback(const std_msgs::Bool::ConstPtr& msg);
void pickCallback(const std_msgs::Bool::ConstPtr& msg);
void placeCallback(const std_msgs::Bool::ConstPtr& msg);
bool fallbackRRT(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& pose);   // cartecias path 실패 시 실행
bool executeCartesianPick(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& target_pose);  // pick 기능용 함수


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mk_low");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);   //flag 처리용 별도 thread
    spinner.start();

    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/arm_controller/command", 10);

    ros::Publisher elapsed_time_pub =
        nh.advertise<std_msgs::Float64>("/elapsed_time", 10);

    // flag 구독
    ros::Subscriber bool_switch_sub = nh.subscribe("/flag/switch", 10, switchCallback);
    ros::Subscriber bool_pick_sub = nh.subscribe("/flag/pick_flag", 10, pickCallback);
    ros::Subscriber bool_place_sub = nh.subscribe("/flag/place_flag", 10, placeCallback);

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
    joint_names = joint_model_group->getVariableNames();

    std::string base_frame = "base_link";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();

    while (ros::ok())
    {
        geometry_msgs::PoseStamped::ConstPtr msg =
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                "/fusion_pose", nh, ros::Duration());

        if (!msg)
        {
            ROS_WARN("No fusion_pose received within timeout.");
            break;
        }

        // PoseStamped > Pose로 변환
        target_pose = msg->pose;

        // Rviz visualization update
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(target_pose, "target_pose");
        visual_tools.trigger();

        if (switch_flag == true)
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

                    ROS_INFO("max_required_time : %f", max_required_time);
                    

                    // 너무 오래 걸리는 궤적이면 스킵
                    if (max_required_time < 3.5)
                    {
                        // traj_pub.publish(traj);
                        publishTraj(joint_values, max_required_time);
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
        else {  // VT 꺼졌을 때
            if ((pick_flag == true) && (switch_flag == false) && (place_flag == false)) {
                ROS_INFO_THROTTLE(1.0, "Pick Start!!!!!!!!");
                
                publishTraj(ready_pos, 2.0);

                ros::Duration(2.2).sleep();
                move_group.stop();
                move_group.clearPoseTargets();
                move_group.setStartStateToCurrentState();
                
                executeCartesianPick(move_group, target_pose);

                publishTraj(ready_pos, 2.0);
                ros::Duration(2.2).sleep();

                pick_flag = false;
            }   // place는 callback에 정의 (fusion_pose 토픽 미발행으로 인한 블로킹)
        }
    }

    return 0;
}


inline std::vector<double> deg2rad(std::initializer_list<double> deg)
{
    if (deg.size() != 6)
        throw std::runtime_error("deg2rad requires exactly 6 elements");

    std::vector<double> rad;
    rad.reserve(6);

    for (double d : deg)
        rad.push_back(d * M_PI / 180.0);

    return rad;
}


void publishTraj(const std::vector<double>& joints_pos, double duration_time)
{
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = joints_pos;

    point.time_from_start = ros::Duration(duration_time);

    traj.points.push_back(point);
    traj_pub.publish(traj);
}

void switchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    switch_flag = msg->data;
    // ROS_INFO("Received switch flag: %s", switch_flag ? "true" : "false");
}

void pickCallback(const std_msgs::Bool::ConstPtr& msg)
{
    pick_flag = msg->data;
    // ROS_INFO("Received pick flag: %s", pick_flag ? "true" : "false");
}


void placeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    place_flag = msg->data;
    // ROS_INFO("Received pick flag: %s", place_flag ? "true" : "false");

    if ((place_flag == true) && (switch_flag == false) && (pick_flag == false)) {
        ROS_INFO_THROTTLE(1.0, "Place Start!!!!!!!!");
        std::vector<double> turn_pos  = deg2rad({90, 10, 110, 100, 90, 0});
        std::vector<double> place_pos  = deg2rad({90, -45, 90, 130, 90, 0});
        // std::vector<double> place_pos  = deg2rad({90, -45, 100, 140, 90, 0});    //joint3 증가하는만큼 joint4도 증가

        publishTraj(ready_pos, 2.0);
        ros::Duration(2.2).sleep();
        publishTraj(turn_pos, 2.0);
        ros::Duration(2.2).sleep();
        publishTraj(place_pos, 2.0);
        ros::Duration(3.2).sleep();
        publishTraj(turn_pos, 2.0);
        ros::Duration(2.2).sleep();
        publishTraj(ready_pos, 2.0);
        ros::Duration(2.2).sleep();

        place_flag = false;
    }
}



/* pick 용  함수 */
bool fallbackRRT(   // cartecias path 실패 시 실행
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& pose)
{
    move_group.clearPoseTargets();
    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success =
        (move_group.plan(plan) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        ROS_WARN("RRT failed!! RRT failed!! RRT failed!!");
        return false;
    }

    move_group.execute(plan);
    return true;
}

bool executeCartesianPick(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::Pose& target_pose)
{
    const double eef_step = 0.01;

    /* ===================== (1) approach > target ===================== */
    std::vector<geometry_msgs::Pose> waypoints_approach;

    geometry_msgs::Pose approach = target_pose;
    approach.position.z += 0.15;

    waypoints_approach.push_back(approach);
    waypoints_approach.push_back(target_pose);

    moveit_msgs::RobotTrajectory traj_approach;
    double frac1 = move_group.computeCartesianPath(
        waypoints_approach, eef_step, traj_approach);

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    if (frac1 >= 0.9)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        plan1.trajectory_ = traj_approach;
        move_group.execute(plan1);
    }
    else
    {
        ROS_WARN("Cartesian approach failed");
        if (!fallbackRRT(move_group, target_pose))
            return false;
    }

    /* ===================== (2) wait ===================== */
    ROS_INFO("Reached target pose, waiting 1 seconds");
    ros::Duration(1.0).sleep();

    /* ===================== (3) target > retreat ===================== */
    std::vector<geometry_msgs::Pose> waypoints_retreat;

    geometry_msgs::Pose retreat = target_pose;
    retreat.position.z += 0.1;

    waypoints_retreat.push_back(retreat);

    moveit_msgs::RobotTrajectory traj_retreat;
    double frac2 = move_group.computeCartesianPath(
        waypoints_retreat, eef_step, traj_retreat);

    if (frac2 >= 0.9)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan2;
        plan2.trajectory_ = traj_retreat;
        move_group.execute(plan2);
    }
    else
    {
        ROS_WARN("Cartesian retreat failed");
        if (!fallbackRRT(move_group, retreat))
            return false;
    }

    return true;
}
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <trajectory_msgs/JointTrajectory.h>
geometry_msgs::Pose target_pose;
    



// Pose 콜백 함수
class PoseSubscriber
{
private:
public:
    void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        // if (pose_update == 0) pose_update = 1;

        ROS_INFO("Received target pose!");

        // waypoints[1]에 Pose의 xyz 값 저장
        target_pose.position.x = msg->position.x;
        target_pose.position.y = msg->position.y;
        target_pose.position.z = msg->position.z;
        target_pose.orientation.x = msg->orientation.x;
        target_pose.orientation.y = msg->orientation.y;
        target_pose.orientation.z = msg->orientation.z;
        target_pose.orientation.w = msg->orientation.w;

        // Waypoints의 갱신된 값을 출력 (디버깅용)
        ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w);

        std::string base_frame = "base_link";
        moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
        visual_tools.deleteAllMarkers();
        // visual_tools.loadRemotecontrol();
        visual_tools.publishAxisLabeled(target_pose, "target_pose");
        visual_tools.trigger();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mk_test");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::NodeHandle nh;

    target_pose.position.x = 0.165;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.86;
    // target_pose.position.x = 0.115;
    // target_pose.position.y = -0.047241;
    // target_pose.position.z = 0.25955;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    PoseSubscriber pose_subscriber;
    ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);

    
    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("dynamixel_trajectory", 10);

    // MoveIt 그룹 생성
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    // 경로 계획 알고리즘 설정
    // move_group.setPlannerId("OMPL");
    move_group.setPlannerId("RRT");
    move_group.setPlanningTime(0.1);


    
    geometry_msgs::Pose zero_pose;
    zero_pose.position.x = 0.165;
    zero_pose.position.y = 0;
    zero_pose.position.z = 0.86;
    // target_pose.position.x = 0.115;
    // target_pose.position.y = -0.047241;
    // target_pose.position.z = 0.25955;
    zero_pose.orientation.x = 0.0;
    zero_pose.orientation.y = 0.0;
    zero_pose.orientation.z = 0.0;
    zero_pose.orientation.w = 1.0;

    geometry_msgs::Pose before_pose;
    before_pose = target_pose;
    // compare_pose.position.x = msg->position.x;
    // compare_pose.position.y = msg->position.y;
    // compare_pose.position.z = msg->position.z;
    // if ((compare_pose.position.x == target_pose.position.x) && (compare_pose.position.y == target_pose.position.y) && (compare_pose.position.z == target_pose.position.z)) {
    //     ROS_INFO("/////////////////same pose///////////////");
    // }

    ros::Rate rate(10); // 10Hz


    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        {
            ROS_INFO("start zero position!!!!!!!!!!!!!!");
            move_group.execute(plan);
            before_pose = target_pose;
        }
    else
        {
            ROS_WARN("start failed!!!!!!!");
        }


    while(ros::ok()){
        if (before_pose != target_pose) {
            // 목표 위치 설정
            move_group.setPoseTarget(target_pose);

            // 경로 계획 실행
            // moveit::planning_interface::MoveGroupInterface::Plan plan;
            success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success)
            {
                ROS_INFO("Path planning successful!");
                move_group.execute(plan);
                before_pose = target_pose;

                // Trajectory 메시지 생성 및 Publish
                trajectory_msgs::JointTrajectory trajectory = plan.trajectory_.joint_trajectory;
                traj_pub.publish(trajectory);
                ROS_INFO("Trajectory published!");

            }
            else
            {
                ROS_WARN("Path planning failed! return zero pose");
                // move_group.setPoseTarget(zero_pose);
                // move_group.plan(plan);
                // move_group.execute(plan);
                // before_pose = zero_pose; 
            }
        }
        else {
            ROS_INFO("///////////////////same pose/////////////////////////////////");
        }
        
    }
    

    ros::shutdown();
    return 0;
}




// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/Pose.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// int main(int argc, char** argv) {
//   // ROS 초기화
//   ros::init(argc, argv, "moveit_pose_example");
//   ros::NodeHandle nh;
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   // MoveIt 인터페이스 초기화
//   static const std::string PLANNING_GROUP = "arm"; // 사용 중인 플래닝 그룹 이름
//   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

//   // 현재 상태 출력
//   ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
//   ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // 목표 pose 설정
//   geometry_msgs::Pose target_pose;
// //   target_pose.orientation.w = 1.0;
//   target_pose.position.x = 0.0;
//   target_pose.position.y = 0.0;
//   target_pose.position.z = 0.25955;
//   move_group.setPoseTarget(target_pose);

//   // Plan 생성
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   if (success) {
//     ROS_INFO("Planning successful. Executing the plan...");
//     move_group.move();  // 계획된 경로 실행
//   } else {
//     ROS_WARN("Planning failed.");
//   }

//   // 종료 처리
//   ros::shutdown();
//   return 0;
// }

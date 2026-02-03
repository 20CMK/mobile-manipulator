// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <std_msgs/Bool.h>

// bool flag = false;

// void boolCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//     flag = msg->data;
//     ROS_INFO("Received bool: %s", flag ? "true" : "false");
// }


// geometry_msgs::Pose target_pose;
    



// // Pose 콜백 함수
// class PoseSubscriber
// {
// private:
// public:
//     void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
//     {
//         // if (pose_update == 0) pose_update = 1;

//         ROS_INFO("Received target pose!");

//         // waypoints[1]에 Pose의 xyz 값 저장
//         target_pose.position.x = msg->position.x;
//         target_pose.position.y = msg->position.y;
//         target_pose.position.z = msg->position.z;
//         target_pose.orientation.x = msg->orientation.x;
//         target_pose.orientation.y = msg->orientation.y;
//         target_pose.orientation.z = msg->orientation.z;
//         target_pose.orientation.w = msg->orientation.w;

//         // // Waypoints의 갱신된 값을 출력 (디버깅용)
//         // ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
//         //         target_pose.position.x,
//         //         target_pose.position.y,
//         //         target_pose.position.z,
//         //         target_pose.orientation.x,
//         //         target_pose.orientation.y,
//         //         target_pose.orientation.z,
//         //         target_pose.orientation.w);

//         std::string base_frame = "base_link";
//         moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
//         visual_tools.deleteAllMarkers();
//         // visual_tools.loadRemotecontrol();
//         visual_tools.publishAxisLabeled(target_pose, "target_pose");
//         visual_tools.trigger();
//     }
// };


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "mk_test");
//     ros::AsyncSpinner spinner(4); // Use 4 threads
//     spinner.start();
//     ros::NodeHandle nh;


//     PoseSubscriber pose_subscriber;
//     ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);
//     ros::Subscriber bool_sub = nh.subscribe("/vision_pp_flag", 10, boolCallback);

//     // MoveIt 그룹 생성
//     moveit::planning_interface::MoveGroupInterface move_group("arm");
//     moveit::planning_interface::PlanningSceneInterface planning_scene;

//     // 경로 계획 알고리즘 설정
//     // move_group.setPlannerId("OMPL");
    
//     // move_group.setPlanningTime(0.1);


    
//     geometry_msgs::Pose zero_pose;
//     zero_pose.position.x = 0.165;
//     zero_pose.position.y = 0;
//     zero_pose.position.z = 0.86;
//     // target_pose.position.x = 0.115;
//     // target_pose.position.y = -0.047241;
//     // target_pose.position.z = 0.25955;
//     zero_pose.orientation.x = 0.0;
//     zero_pose.orientation.y = 0.0;
//     zero_pose.orientation.z = 0.0;
//     zero_pose.orientation.w = 1.0;

//     geometry_msgs::Pose before_pose;
//     before_pose = target_pose;
//     // compare_pose.position.x = msg->position.x;
//     // compare_pose.position.y = msg->position.y;
//     // compare_pose.position.z = msg->position.z;
//     // if ((compare_pose.position.x == target_pose.position.x) && (compare_pose.position.y == target_pose.position.y) && (compare_pose.position.z == target_pose.position.z)) {
//     //     ROS_INFO("/////////////////same pose///////////////");
//     // }


//     // move_group.setPoseTarget(target_pose);
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     // if (success)
//     //     {
//     //         ROS_INFO("start zero position!!!!!!!!!!!!!!");
//     //         move_group.execute(plan);
//     //         before_pose = target_pose;
//     //     }
//     // else
//     //     {
//     //         ROS_WARN("start failed!!!!!!!");
//     //     }
//     std::vector<double> joint_positions;

//     while(ros::ok()){

//         if (flag == true) {
//             if (before_pose != target_pose) {
                



//                 move_group.setPlannerId("SHOMP");


//                 // 조인트 수에 맞게 각도를 radian 단위로 설정
// joint_positions.clear();
// joint_positions.push_back(-1.57);  // joint1
// joint_positions.push_back(0.0); // joint2
// joint_positions.push_back(1.57);  // joint3
// joint_positions.push_back(1.57);  // joint4
// joint_positions.push_back(1.57);  // joint5
// joint_positions.push_back(0.0);  // joint6

// // 목표 joint 값 설정
// move_group.setJointValueTarget(joint_positions);

// // 계획 후 실행
// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// if (success)
// {
//     move_group.execute(my_plan);
// }
// else
// {
//     ROS_WARN("Joint space planning failed.");
// }






//                 target_pose.position.z += 0.3;
//                 // 목표 위치 설정
//                 move_group.setPoseTarget(target_pose);

//                 // 경로 계획 실행
//                 // moveit::planning_interface::MoveGroupInterface::Plan plan;
//                 success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//                 if (success)
//                 {
//                     ROS_INFO("Path planning successful!");
//                     move_group.execute(plan);
//                     before_pose = target_pose;
//                 }
//                 else
//                 {
//                     ROS_WARN("Path planning failed! return zero pose");
//                     // move_group.setPoseTarget(zero_pose);
//                     // move_group.plan(plan);
//                     // move_group.execute(plan);
//                     // before_pose = zero_pose; 
//                 }

//                 /////////////// 두번째 포인트  ///////////////////////
//                 target_pose.position.z -= 0.3;

//                 std::vector<geometry_msgs::Pose> waypoints;
//                 // geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
//                 // waypoints.push_back(start_pose);
//                 waypoints.push_back(target_pose);

//                 moveit_msgs::RobotTrajectory trajectory;
//                 const double eef_step = 0.01;         // end-effector 간격(m)

//                 double fraction = move_group.computeCartesianPath(
//                     waypoints,
//                     eef_step,
//                     trajectory,
//                     true  // avoid_collisions
//                 );
//                 if (fraction > 0.7)  // 거의 100% 궤적 생성됨
//                 {
//                     ROS_INFO("Cartesian path success (%.2f%% achieved)", fraction * 100.0);

//                     // trajectory 실행
//                     moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
//                     cartesian_plan.trajectory_ = trajectory;
//                     move_group.execute(cartesian_plan);
//                     before_pose = target_pose;
//                 }
//                 else
//                 {
//                     ROS_WARN("Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
//                 }


//                 ///////////////// 세번째  포인트 ///////////////////////
//                 target_pose.position.z += 0.3;
//                 waypoints.clear();
//                 waypoints.push_back(target_pose);
//                 fraction = move_group.computeCartesianPath(
//                     waypoints,
//                     eef_step,
//                     trajectory,
//                     true  // avoid_collisions
//                 );
//                 if (fraction > 0.99)  // 거의 100% 궤적 생성됨
//                 {
//                     ROS_INFO("Cartesian path success (%.2f%% achieved)", fraction * 100.0);

//                     // trajectory 실행
//                     moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
//                     cartesian_plan.trajectory_ = trajectory;
//                     move_group.execute(cartesian_plan);
//                     before_pose = target_pose;
//                 }
//                 else
//                 {
//                     ROS_WARN("Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
//                 }





//                 // /////////////네번째 포인트 ///////////////////////////
//                joint_positions.clear();
//                std::vector<double> current_joint_values = move_group.getCurrentJointValues();

// joint_positions.push_back(1.57);  // joint1
// // joint2 ~ joint6는 현재 값을 그대로 사용
// for (size_t i = 1; i < current_joint_values.size(); ++i) {
//     joint_positions.push_back(current_joint_values[i]);
// }

// // 목표 joint 값 설정
// move_group.setJointValueTarget(joint_positions);

// // 계획 후 실행
// success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// if (success)
// {
//     move_group.execute(my_plan);
// }
// else
// {
//     ROS_WARN("Joint space planning failed.");
// }






//                 /////////////다섯번째 포인트 ///////////////////////////
//                 target_pose.position.x = -0.4;
//                 target_pose.position.y = 0.0;
//                 target_pose.position.z = 0.4;
//                 target_pose.orientation.x = 0.0;
//                 target_pose.orientation.y = 0.0;
//                 target_pose.orientation.z = 1.0;
//                 target_pose.orientation.w = 0.0;


//                 // 목표 위치 설정
//                 move_group.setPoseTarget(target_pose);

//                 // 경로 계획 실행
//                 // moveit::planning_interface::MoveGroupInterface::Plan plan;
//                 success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//                 if (success)
//                 {
//                     ROS_INFO("Path planning successful!");
//                     move_group.execute(plan);
//                     before_pose = target_pose;
//                 }
//                 else
//                 {
//                     ROS_WARN("Path planning failed! return zero pose");
//                     // move_group.setPoseTarget(zero_pose);
//                     // move_group.plan(plan);
//                     // move_group.execute(plan);
//                     // before_pose = zero_pose; 
//                 }



//                 /////////////여섯번째 /////////////////
//                 target_pose.position.z = 0.2;
//                 waypoints.clear();
//                 waypoints.push_back(target_pose);
//                 fraction = move_group.computeCartesianPath(
//                     waypoints,
//                     eef_step,
//                     trajectory,
//                     true  // avoid_collisions
//                 );
//                 if (fraction > 0.99)  // 거의 100% 궤적 생성됨
//                 {
//                     ROS_INFO("Cartesian path success (%.2f%% achieved)", fraction * 100.0);

//                     // trajectory 실행
//                     moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
//                     cartesian_plan.trajectory_ = trajectory;
//                     move_group.execute(cartesian_plan);
//                     before_pose = target_pose;
//                 }
//                 else
//                 {
//                     ROS_WARN("Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
//                 }


//                 /////////////7번째 /////////////////
//                 target_pose.position.z = 0.4;
//                 waypoints.clear();
//                 waypoints.push_back(target_pose);
//                 fraction = move_group.computeCartesianPath(
//                     waypoints,
//                     eef_step,
//                     trajectory,
//                     true  // avoid_collisions
//                 );
//                 if (fraction > 0.99)  // 거의 100% 궤적 생성됨
//                 {
//                     ROS_INFO("Cartesian path success (%.2f%% achieved)", fraction * 100.0);

//                     // trajectory 실행
//                     moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
//                     cartesian_plan.trajectory_ = trajectory;
//                     move_group.execute(cartesian_plan);
//                     before_pose = target_pose;
//                 }
//                 else
//                 {
//                     ROS_WARN("Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
//                 }



//                 // /////////////8번째 포인트 ///////////////////////////
//                 // move_group.setPlannerId("RRT");


//                 // target_pose = zero_pose ;
//                 // // 목표 위치 설정
//                 // move_group.setPoseTarget(target_pose);

//                 // // 경로 계획 실행
//                 // // moveit::planning_interface::MoveGroupInterface::Plan plan;
//                 // success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//                 // if (success)
//                 // {
//                 //     ROS_INFO("Path planning successful!");
//                 //     move_group.execute(plan);
//                 //     before_pose = target_pose;
//                 // }
//                 // else
//                 // {
//                 //     ROS_WARN("Path planning failed! return zero pose");
//                 //     // move_group.setPoseTarget(zero_pose);
//                 //     // move_group.plan(plan);
//                 //     // move_group.execute(plan);
//                 //     // before_pose = zero_pose; 
//                 // }

//                 std::vector<double> joint_positions;

// // 조인트 수에 맞게 각도를 radian 단위로 설정
// joint_positions.clear();
// joint_positions.push_back(0.0);  // joint1
// joint_positions.push_back(0.0); // joint2
// joint_positions.push_back(0.0);  // joint3
// joint_positions.push_back(0.0);  // joint4
// joint_positions.push_back(0.0);  // joint5
// joint_positions.push_back(0.0);  // joint6

// // 목표 joint 값 설정
// move_group.setJointValueTarget(joint_positions);

// // 계획 후 실행
// success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// if (success)
// {
//     move_group.execute(my_plan);
// }
// else
// {
//     ROS_WARN("Joint space planning failed.");
// }

// flag = false;






//             }
//             else {
//                 // ROS_INFO("///////////////////same pose/////////////////////////////////");
//             }
//         }
        
//     }
    

//     ros::shutdown();
//     return 0;
// }






#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>  // 이 라인을 상단에 추가

bool flag = false;

void boolCallback(const std_msgs::Bool::ConstPtr& msg)
{
    flag = msg->data;
    ROS_INFO("Received bool: %s", flag ? "true" : "false");
}

geometry_msgs::Pose target_pose;

class PoseSubscriber
{
public:
    void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        ROS_INFO("Received target pose!");

        target_pose = *msg;

        moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
        visual_tools.deleteAllMarkers();
        visual_tools.publishAxisLabeled(target_pose, "target_pose");
        visual_tools.trigger();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mk_test");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh;

    PoseSubscriber pose_subscriber;
    ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);
    ros::Subscriber bool_sub = nh.subscribe("/vision_pp_flag", 10, boolCallback);
    ros::Publisher gripper_pub = nh.advertise<std_msgs::Int8>("/gripper_cmd", 10);


    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    geometry_msgs::Pose before_pose = target_pose;

    std::vector<double> joint_positions;

    while(ros::ok()){
        if (flag && before_pose != target_pose) {
            move_group.setPlannerId("SHOMP");

            // 1. 특정 Joint 위치로 이동
            joint_positions = {-1.57, 0.0, 1.57, 1.57, 1.57, 0.0};
            move_group.setJointValueTarget(joint_positions);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success) move_group.execute(my_plan);
            else ROS_WARN("1st Joint space planning failed.");

            // 2. Z축 위로 이동
            target_pose.position.z += 0.3;
            move_group.setPoseTarget(target_pose);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success) {
                move_group.execute(my_plan);
                before_pose = target_pose;
            } else ROS_WARN("2nd pose planning failed!");

            // 3. Z축 아래로 Cartesian 이동
            
            // target_pose.position.y -= 0.05;
            std::vector<geometry_msgs::Pose> waypoints = {target_pose};
            moveit_msgs::RobotTrajectory trajectory;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory, true);

            
            if (fraction > 0.7) {
                moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
                cartesian_plan.trajectory_ = trajectory;
                move_group.execute(cartesian_plan);
                before_pose = target_pose;
            } else ROS_WARN("3rd Cartesian path failed (%.2f%%)", fraction * 100.0);

            // 4. 다시 Z축 위로 Cartesian 복귀
            target_pose.position.z -= 0.3;
            waypoints = {target_pose};
            fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory, true);


            if (fraction > 0.99) {
                moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
                cartesian_plan.trajectory_ = trajectory;
                move_group.execute(cartesian_plan);
                before_pose = target_pose;
            } else ROS_WARN("4th Cartesian path failed (%.2f%%)", fraction * 100.0);
            
            std_msgs::Int8 gripper_msg;
            gripper_msg.data = 0;
            gripper_pub.publish(gripper_msg);
            
            flag = false;
        }
    }

    ros::shutdown();
    return 0;
}

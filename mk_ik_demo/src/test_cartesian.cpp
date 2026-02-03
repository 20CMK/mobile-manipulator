// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <serial/serial.h>
// #include <std_msgs/Float64.h>

// std_msgs::Float64 elapsed_msg;
// geometry_msgs::Pose target_pose;

// // Pose 콜백 함수
// class PoseSubscriber
// {
// private:
// public:
//     void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
//     {
//         // if (pose_update == 0) pose_update = 1;

//         // ROS_INFO("Received target pose!");

//         // waypoints[1]에 Pose의 xyz 값 저장
//         target_pose.position.x = msg->position.x;
//         target_pose.position.y = msg->position.y;
//         target_pose.position.z = msg->position.z;
//         target_pose.orientation.x = msg->orientation.x;
//         target_pose.orientation.y = msg->orientation.y;
//         target_pose.orientation.z = msg->orientation.z;
//         target_pose.orientation.w = msg->orientation.w;

//         // Waypoints의 갱신된 값을 출력 (디버깅용)
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
//     // initializeSerial();
//     ros::init(argc, argv, "mk_cartesian");
//     ros::AsyncSpinner spinner(4); // Use 4 threads
//     spinner.start();
//     ros::NodeHandle nh;

//     target_pose.position.x = 0.165;
//     target_pose.position.y = 0.0;
//     target_pose.position.z = 0.86;
//     // target_pose.position.x = 0.115;
//     // target_pose.position.y = -0.047241;
//     // target_pose.position.z = 0.25955;
//     target_pose.orientation.x = 0.0;
//     target_pose.orientation.y = 0.0;
//     target_pose.orientation.z = 0.0;
//     target_pose.orientation.w = 1.0;

//     PoseSubscriber pose_subscriber;
//     ros::Subscriber pose_sub = nh.subscribe("/target_pose", 10, &PoseSubscriber::poseCallback, &pose_subscriber);

//     ros::Publisher elapsed_time_pub = nh.advertise<std_msgs::Float64>("/elapsed_time", 10);

//     // MoveIt 그룹 생성
//     moveit::planning_interface::MoveGroupInterface move_group("arm");
//     moveit::planning_interface::PlanningSceneInterface planning_scene;

//     // 경로 계획 알고리즘 설정
//     // move_group.setPlannerId("OMPL");
//     move_group.setPlannerId("RRTConnector");
//     move_group.setPlanningTime(1.0);   //원래는 0.1

    
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
    
//     trajectory_msgs::JointTrajectory trajectory;
//     trajectory_msgs::JointTrajectoryPoint current_point;




//     move_group.setPoseTarget(target_pose);
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success)
//         {
//             ROS_INFO("start zero position!!!!!!!!!!!!!!");
//             move_group.execute(plan);
            
//             // trajectory = plan.trajectory_.joint_trajectory;
//             // // plan.trajectory_ = trajectory;
//             // // Last joints
//             // trajectory_msgs::JointTrajectoryPoint last_point = trajectory.points.back();
//             // //opencr로 전송
//             // std::vector<double> joint_angles = current_point.positions;
//             // sendJointAnglesToOpenCR(joint_angles);
            

//             before_pose = target_pose;
//         }
//     else
//         {
//             ROS_WARN("start failed!!!!!!!");
//         }

//     // === 로봇 모델 및 상태 준비 ===
//             robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//             robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//             const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
//             robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//             kinematic_state->setToDefaultValues();

            
//     while(ros::ok()){
//         if (before_pose != target_pose) {
//             ros::Time start = ros::Time::now(); //연산 시간 측정용
//             // 목표 위치 설정
//             move_group.setPoseTarget(target_pose);

            

//             // === IK 계산 ===
//             bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 1.0);

//             if (found_ik)
//             {
//                 // IK 결과를 자코비안 계산에 반영
//                 Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0); // End-effector 중심 기준
//                 Eigen::MatrixXd jacobian;
//                 kinematic_state->getJacobian(
//                     joint_model_group,
//                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
//                     reference_point_position,
//                     jacobian);

//                 // ROS_INFO_STREAM("Jacobian Matrix:\n" << jacobian);

//                 // 랭크 분석
//                 Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
//                 int rank = svd.rank();
//                 int dof = jacobian.cols();

//                 if (rank < dof)
//                 {
//                     // ROS_WARN("⚠️  Singularity detected! Jacobian is rank-deficient.");
//                     // // 경로 계획 실행
//                     // // moveit::planning_interface::MoveGroupInterface::Plan plan;
//                     // success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
//                     // if (success)
//                     // {
//                     //     // ROS_INFO("Path planning successful!");
//                     //     ros::Time end = ros::Time::now();
//                     //     ros::Duration elapsed_time = end - start;
//                     //     ROS_INFO("operating time: %.6f sec", elapsed_time.toSec()); 
//                     //     elapsed_msg.data = elapsed_time.toSec();
//                     //     elapsed_time_pub.publish(elapsed_msg);

//                     //     move_group.execute(plan);
//                     //     before_pose = target_pose;
    
//                     // }
//                     // else
//                     // {
//                     //     // ROS_WARN("Path planning failed!");
//                     // }
//                 }
//                 else
//                 {
//                     // ROS_INFO("✅  No singularity detected. Jacobian is full-rank.");
//                     // Cartesian 경로 설정
//                     std::vector<geometry_msgs::Pose> waypoints;
//                     waypoints.push_back(target_pose);

//                     moveit_msgs::RobotTrajectory trajectory;
//                     const double eef_step = 0.001;  // End-effector 간격
//                     double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

//                     if (fraction > 0.1)
//                     {
//                         // ROS_INFO("Cartesian path planning successful! fraction = %.2f", fraction);

//                         // 경로 실행
//                         moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
//                         cartesian_plan.trajectory_ = trajectory;

                        
//                         ros::Time end = ros::Time::now();
//                         ros::Duration elapsed_time = end - start;
//                         ROS_INFO("operating time: %.6f sec", elapsed_time.toSec()); 
//                         elapsed_msg.data = elapsed_time.toSec();
//                         elapsed_time_pub.publish(elapsed_msg);


//                         move_group.execute(cartesian_plan);      //블로킹해서 이게 끝나야 다음 코드 실행됨
//                         // move_group.asyncExecute(cartesian_plan);    //논블로킹
//                         before_pose = target_pose;
//                     }
//                     else
//                     {
//                         // // ROS_WARN("Cartesian path failed (fraction = %.2f). Falling back to RRT.", fraction);
//                         // move_group.setPoseTarget(target_pose);

//                         // ros::Time end = ros::Time::now();
//                         // ros::Duration elapsed_time = end - start;
//                         // ROS_INFO("operating time: %.6f sec", elapsed_time.toSec()); 
//                         // elapsed_msg.data = elapsed_time.toSec();
//                         // elapsed_time_pub.publish(elapsed_msg);


//                         // if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
//                         // {
//                         //     move_group.execute(plan);
//                         //     before_pose = target_pose;
//                         // }
//                     }

//                 }
                
                    
                    
//             }
//             else   // IK fail
//             {
//                 ROS_WARN("❌ IK failed: no solution for the given pose.");
//             }
            
//         }
//         else  // before_pose == current_pose
//         {
//             // ROS_INFO("///////////////////same pose/////////////////////////////////");
//         }

        
        
//     }
    

//     ros::shutdown();
//     return 0;
// }








// // #include <ros/ros.h>
// // #include <moveit/move_group_interface/move_group_interface.h>
// // #include <moveit/planning_scene_interface/planning_scene_interface.h>
// // #include <moveit_visual_tools/moveit_visual_tools.h>
// // #include <tf2/LinearMath/Quaternion.h>
// // #include <tf2/LinearMath/Matrix3x3.h>
// // #include <serial/serial.h>
// // geometry_msgs::Pose target_pose;
// // std::mutex pose_mutex;

// // // Pose 콜백 함수
// // class PoseSubscriber
// // {
// // private:
// // public:
// //     void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
// //     {
// //         // if (pose_update == 0) pose_update = 1;

// //         // ROS_INFO("Received target pose!");
// //         std::lock_guard<std::mutex> lock(pose_mutex);
        

// //         // waypoints[1]에 Pose의 xyz 값 저장
// //         target_pose.position.x = msg->position.x;
// //         target_pose.position.y = msg->position.y;
// //         target_pose.position.z = msg->position.z;
// //         target_pose.orientation.x = msg->orientation.x;
// //         target_pose.orientation.y = msg->orientation.y;
// //         target_pose.orientation.z = msg->orientation.z;
// //         target_pose.orientation.w = msg->orientation.w;

// //         // Waypoints의 갱신된 값을 출력 (디버깅용)
// //         ROS_INFO("Updated target_pose -> x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
// //                 target_pose.position.x,
// //                 target_pose.position.y,
// //                 target_pose.position.z,
// //                 target_pose.orientation.x,
// //                 target_pose.orientation.y,
// //                 target_pose.orientation.z,
// //                 target_pose.orientation.w);

// //         std::string base_frame = "base_link";
// //         moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
// //         visual_tools.deleteAllMarkers();
// //         // visual_tools.loadRemotecontrol();
// //         visual_tools.publishAxisLabeled(target_pose, "target_pose");
// //         visual_tools.trigger();
// //     }
// // };



// // int main(int argc, char** argv)
// // {
// //     // initializeSerial();
// //     ros::init(argc, argv, "mk_cartesian");
// //     ros::AsyncSpinner spinner(2); // Use 4 threads
// //     spinner.start();
// //     ros::NodeHandle nh;

// //     // target_pose.position.x = 0.165;
// //     // target_pose.position.y = 0.0;
// //     // target_pose.position.z = 0.86;
// //     // // target_pose.position.x = 0.115;
// //     // // target_pose.position.y = -0.047241;
// //     // // target_pose.position.z = 0.25955;
// //     // target_pose.orientation.x = 0.0;
// //     // target_pose.orientation.y = 0.0;
// //     // target_pose.orientation.z = 0.0;
// //     // target_pose.orientation.w = 1.0;

// //     PoseSubscriber pose_subscriber;
// //     ros::Subscriber pose_sub = nh.subscribe("/target_pose", 1, &PoseSubscriber::poseCallback, &pose_subscriber);

// //     // MoveIt 그룹 생성
// //     moveit::planning_interface::MoveGroupInterface move_group("arm");
// //     moveit::planning_interface::PlanningSceneInterface planning_scene;

// //     // 경로 계획 알고리즘 설정
// //     // move_group.setPlannerId("OMPL");
// //     // move_group.setPlannerId("RRT");
// //     move_group.setPlanningTime(0.001);


    
// //     geometry_msgs::Pose zero_pose;
// //     zero_pose.position.x = 0.165;
// //     zero_pose.position.y = 0;
// //     zero_pose.position.z = 0.86;
// //     // target_pose.position.x = 0.115;
// //     // target_pose.position.y = -0.047241;
// //     // target_pose.position.z = 0.25955;
// //     zero_pose.orientation.x = 0.0;
// //     zero_pose.orientation.y = 0.0;
// //     zero_pose.orientation.z = 0.0;
// //     zero_pose.orientation.w = 1.0;


// //     // compare_pose.position.x = msg->position.x;
// //     // compare_pose.position.y = msg->position.y;
// //     // compare_pose.position.z = msg->position.z;
// //     // if ((compare_pose.position.x == target_pose.position.x) && (compare_pose.position.y == target_pose.position.y) && (compare_pose.position.z == target_pose.position.z)) {
// //     //     ROS_INFO("/////////////////same pose///////////////");
// //     // }
    
// //     trajectory_msgs::JointTrajectory trajectory;
// //     trajectory_msgs::JointTrajectoryPoint current_point;



    
// //     // move_group.setPoseTarget(target_pose);
// //     moveit::planning_interface::MoveGroupInterface::Plan plan;
// //     // bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
// //     // if (success)
// //     //     {
// //     //         ROS_INFO("start zero position!!!!!!!!!!!!!!");
// //     //         move_group.execute(plan);
// //     //     }
// //     // else
// //     //     {
// //     //         ROS_WARN("start failed!!!!!!!");
// //     //     }


// //     while(ros::ok()){
// //         // std::lock_guard<std::mutex> lock(pose_mutex);
// //         // 목표 위치 설정
// //         move_group.setPoseTarget(target_pose);

// //         // 경로 계획 실행
// //         // moveit::planning_interface::MoveGroupInterface::Plan plan;
// //         bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// //         if (success)
// //         {
// //             ROS_INFO("Path planning successful!");
// //             move_group.execute(plan);

// //         }
// //         else
// //         {
// //             ROS_WARN("Path planning failed! return zero pose");
// //         }
        
// //     }
    

// //     ros::shutdown();
// //     return 0;
// // }





// //                 // // 관절값을 실제 로봇(시뮬레이션 포함)에 적용
// //                 // moveit::planning_interface::MoveGroupInterface::Plan plan;
// //                 // trajectory_msgs::JointTrajectoryPoint point;
// //                 // point.positions = joint_values;
// //                 // point.time_from_start = ros::Duration(0.01);  // 1초 내에 도달         이거 꼭 알아보기

// //                 // // 계획 생성
// //                 // // plan.trajectory_.joint_trajectory.joint_names = joint_model_group->getVariableNames();
// //                 // // plan.trajectory_.joint_trajectory.points.push_back(point);
// //                 // plan.trajectory_.joint_trajectory.points.clear();  // 초기화
// //                 // plan.trajectory_.joint_trajectory.joint_names = joint_model_group->getVariableNames();
// //                 // plan.trajectory_.joint_trajectory.points.push_back(point);

// //                 // // 실행
// //                 // move_group.execute(plan);




#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <thread>

class HighPerfPublisher {
public:
    HighPerfPublisher()
        : it_(nh_)
    {
        pub_ = it_.advertise("/mk/image_raw", 1);

        // GStreamer backend로 영상 입력
        cap_.open("v4l2src device=/dev/video4 ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink", cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            ROS_ERROR("❌ GStreamer 카메라 열기 실패");
            ros::shutdown();
        }

        running_ = true;
        thread_ = std::thread(&HighPerfPublisher::publishLoop, this);
    }

    ~HighPerfPublisher() {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

    void publishLoop() {
        ros::Rate rate(30);  // 30 FPS
        while (ros::ok() && running_) {
            cv::Mat frame;
            cap_ >> frame;
            if (frame.empty()) {
                ROS_WARN("⚠️ 빈 프레임 수신");
                continue;
            }

            std_msgs::Header header;
            header.stamp = ros::Time::now();
            cv_bridge::CvImage img_bridge(header, "bgr8", frame);

            pub_.publish(img_bridge.toImageMsg());
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    cv::VideoCapture cap_;
    std::thread thread_;
    bool running_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "high_perf_compressed_pub");
    HighPerfPublisher node;
    ros::spin();
    return 0;
}

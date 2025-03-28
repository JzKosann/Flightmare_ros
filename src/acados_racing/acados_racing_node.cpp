// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mav_msgs/QuadThrusts.h>
#include <mav_msgs/QuadCurState.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

#include "flightlib/common/command.hpp"

// trajectory
#include <geometry_msgs/Transform.h> // 用于表示位置和方�?
#include <nav_msgs/Path.h>           // 路径消息类型
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h> // 用于轨迹消息

#include <acados_nmpc_controller/ros_node.hpp>
#include <thread>

using namespace flightlib;
std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
QuadState quad_state;
Command quad_command;
std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();

void acados_ros::UnityCallback(const mav_msgs::QuadThrusts::ConstPtr &msg)
{
    static uint32_t seq_i;
    static ros::Time last_time;
    seq_i++;
    Scalar t = 0.0;
    Vector<4> thrusts{msg->thrusts_1, msg->thrusts_2, msg->thrusts_3, msg->thrusts_4};
    // Vector<4> thrusts{2.490,2.49,2.49,2.49};
    Command cmd(t, thrusts);
    Scalar ctl_dt = 0.01;
    // 检查命令是否有效
    if (cmd.valid())
    {
        std::cout << "Command is valid." << std::endl;
        quad_ptr->run(cmd, ctl_dt);
    }

    // 检查控制模式
    if (cmd.isSingleRotorThrusts())
    {
        std::cout << "Using single rotor thrusts mode." << std::endl;
        std::cout << "Thrusts: " << cmd.thrusts.transpose() << std::endl;
    }
    std::cout<<quad_ptr->getDynamics()<<std::endl; // 打印动态方程参数
    // 发布无人机状态信息
    quad_ptr->getState(&quad_state);
    mav_msgs::QuadCurState quad_state_msg;
    quad_state_msg.x = quad_state.x[QS::POSX];
    quad_state_msg.y = quad_state.x[QS::POSY];
    quad_state_msg.z = quad_state.x[QS::POSZ];
    quad_state_msg.vx = quad_state.x[QS::VELX];
    quad_state_msg.vy = quad_state.x[QS::VELY];
    quad_state_msg.vz = quad_state.x[QS::VELZ];
    quad_state_msg.qw = quad_state.x[QS::ATTW];
    quad_state_msg.qx = quad_state.x[QS::ATTX];
    quad_state_msg.qy = quad_state.x[QS::ATTY];
    quad_state_msg.qz = quad_state.x[QS::ATTZ];
    quad_state_msg.p = quad_state.x[QS::OMEX];
    quad_state_msg.q = quad_state.x[QS::OMEY];
    quad_state_msg.r = quad_state.x[QS::OMEZ];
    ros::Time Unity_time = ros::Time::now();
    quad_state_msg.time = Unity_time.toSec();
    acados_ros::QuadCurStates_pub.publish(quad_state_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.orientation.w = quad_state_msg.qw;
    pose_msg.pose.orientation.x = quad_state_msg.qx;
    pose_msg.pose.orientation.y = quad_state_msg.qy;
    pose_msg.pose.orientation.z = quad_state_msg.qz;
    pose_msg.pose.position.x = quad_state_msg.x;
    pose_msg.pose.position.y = quad_state_msg.y;
    pose_msg.pose.position.z = quad_state_msg.z;
    pose_msg.header.stamp = Unity_time;
    pose_msg.header.frame_id = "world";
    pose_msg.header.seq = seq_i;
    // pose_msg.header.seq=
    acados_ros::Oritantion_pub.publish(pose_msg);

    // geometry_msgs::TwistStamped twist_msg;
    // twist_msg.twist.linear.x = quad_state_msg.vx;
    // twist_msg.twist.linear.y = quad_state_msg.vy;
    // twist_msg.twist.linear.z = quad_state_msg.vz;
    // twist_msg.twist.angular.x = quad_state_msg.p;
    // twist_msg.twist.angular.y = quad_state_msg.q;
    // twist_msg.twist.angular.z = quad_state_msg.r;
    // twist_msg.header.stamp = Unity_time;
    // twist_msg.header.frame_id = "world";
    // twist_msg.header.seq = seq_i;
    // acados_ros::Twist_pub.publish(twist_msg);

    printf("pos: x:%f, y:%f, z:%f \r\n vel: x:%f, y:%f, z:%f \r\nquat: w:%f, x:%f, y:%f, z:%f \r\n w: p:%f, q:%f, r:%f\r\n", quad_state.x[QS::POSX], quad_state.x[QS::POSY], quad_state.x[QS::POSZ],quad_state.x[QS::VELX], quad_state.x[QS::VELY], quad_state.x[QS::VELZ],
        quad_state.x[QS::ATTW], quad_state.x[QS::ATTX], quad_state.x[QS::ATTY], quad_state.x[QS::ATTZ],quad_state.x[QS::OMEX], quad_state.x[QS::OMEY], quad_state.x[QS::OMEZ]);
    
    printf("delay: %f ms \r\n", (Unity_time.toSec() - last_time.toSec()) * 1000);
    last_time = Unity_time;
    unity_bridge_ptr->getRender(0);
    unity_bridge_ptr->handleOutput();
    // cv::Mat img;
    // rgb_camera->getRGBImage(img);
    // sensor_msgs::ImagePtr rgb_msg =
    //     cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    // ros::Time timestamp = ros::Time::now();
    // rgb_msg->header.stamp = timestamp;
    // acados_ros::rgb_pub.publish(rgb_msg);
    // ROS_INFO("img sent!");
}

void acados_ros::Timer_callback(const ros::TimerEvent &e)
{
}

// static void InitializeEnv() {}
int main(int argc, char *argv[])
{
    // initialize ROS
    ros::init(argc, argv, "acados_drone_racing_scene");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
    ros::Rate(50.0);
    ros::Time::init();
    // 创建订阅者 订阅发布的控制预期位置
    acados_ros::QuadThrusts_sub = nh.subscribe("flightmare_control/thrusts", 1, acados_ros::UnityCallback);
    acados_ros::QuadCurStates_pub = nh.advertise<mav_msgs::QuadCurState>("flightmare_control/state_info", 1);
    acados_ros::Oritantion_pub = nh.advertise<geometry_msgs::PoseStamped>("flightmare_control/Oritantion", 1);
    acados_ros::Twist_pub=nh.advertise<geometry_msgs::TwistStamped>("flightmare_control/Twist", 1);
    image_transport::ImageTransport it(pnh);
    acados_ros::rgb_pub = it.advertise("/flightmare_control/rgb", 1);
    // 创建发布者
    // unity 场景设置--------------------------------------------------------------------------//
    // Flightmare(Unity3D)
    SceneID scene_id{UnityScene::WAREHOUSE}; // INDUSTRIAL
    bool unity_ready{false};
    // unity quadrotor

    Vector<3> quad_size(0.5, 0.5, 0.5);
    quad_ptr->setSize(quad_size);

    std::string object_id = "unity_gate";
    std::string prefab_id = "rpg_gate";
    std::shared_ptr<StaticObject> gate_1 =
        std::make_shared<StaticObject>(object_id, prefab_id);
    gate_1->setPosition(Eigen::Vector3f(-2, 8, 2.5));
    gate_1->setQuaternion(
        Quaternion(0.9855, 0.0, 0.0, -0.1690));

    std::string object_id_2 = "unity_gate_2";
    std::shared_ptr<StaticGate> gate_2 =
        std::make_shared<StaticGate>(object_id_2);
    gate_2->setPosition(Eigen::Vector3f(9, 9, 2.5));
    gate_2->setQuaternion(
        Quaternion(0.3896, 0.0000, 0.0000, -0.9209));

    std::string object_id_3 = "unity_gate_3";
    std::shared_ptr<StaticGate> gate_3 =
        std::make_shared<StaticGate>(object_id_3);
    gate_3->setPosition(Eigen::Vector3f(10, 0, 2.5));
    gate_3->setQuaternion(Quaternion(0.0000, 0.0000, 0.0000, 1.0000));

    std::string object_id_4 = "unity_gate_4";
    std::shared_ptr<StaticGate> gate_4 =
        std::make_shared<StaticGate>(object_id_4);
    gate_4->setPosition(Eigen::Vector3f(8, -8, 2.5));
    gate_4->setQuaternion(
        Quaternion(0.8776, 0.0000, 0.0000, -0.4794));

    std::string object_id_5 = "unity_gate_5";
    std::shared_ptr<StaticGate> gate_5 =
        std::make_shared<StaticGate>(object_id_5);
    gate_5->setPosition(Eigen::Vector3f(0, -10, 2.5));
    gate_5->setQuaternion(
        Quaternion(0.7071, 0.0000, 0.0000, 0.7071));

    std::string object_id_6 = "unity_gate_6";
    std::shared_ptr<StaticGate> gate_6 =
        std::make_shared<StaticGate>(object_id_6);
    gate_6->setPosition(Eigen::Vector3f(-8, -7, 6.2));
    gate_6->setQuaternion(
        Quaternion(0.7933, 0.0000, 0.0000, 0.6088));

    std::string object_id_7 = "unity_gate_7";
    std::shared_ptr<StaticGate> gate_7 =
        std::make_shared<StaticGate>(object_id_7);
    gate_7->setPosition(Eigen::Vector3f(-8, -7, 2.5));
    gate_7->setQuaternion(
        Quaternion(0.7933, 0.0000, 0.0000, 0.6088));

    // unity 场景设置--------------------------------------------------------------------------//

    // Initialize camera
    Vector<3> B_r_BC(0.0, 0.0, 0.3);
    Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
    std::cout << R_BC << std::endl;
    rgb_camera->setFOV(90);
    rgb_camera->setWidth(640);
    rgb_camera->setHeight(360);
    rgb_camera->setRelPose(B_r_BC, R_BC);
    rgb_camera->setPostProcesscing(std::vector<bool>{
        false, false, false}); // depth, segmentation, optical flow
    quad_ptr->addRGBCamera(rgb_camera);

    // Start racing
    ros::Time t0 = ros::Time::now();

    unity_bridge_ptr->addStaticObject(gate_1);
    unity_bridge_ptr->addStaticObject(gate_2);
    unity_bridge_ptr->addStaticObject(gate_3);
    unity_bridge_ptr->addStaticObject(gate_4);
    unity_bridge_ptr->addStaticObject(gate_5);
    unity_bridge_ptr->addStaticObject(gate_6);
    unity_bridge_ptr->addStaticObject(gate_7);
    unity_bridge_ptr->addQuadrotor(quad_ptr);
    mav_msgs::QuadCurState quad_state_msg;
    // for (int cnt; cnt <= 500; cnt++)// 轮发 唤醒py
    //         acados_ros::QuadCurStates_pub.publish(quad_state_msg);
    ROS_INFO("aroused!");
    // initialization
    quad_state.setZero();
    quad_ptr->reset(quad_state);
    // connect unity
    unity_ready = unity_bridge_ptr->connectUnity(scene_id);

    ros::Timer timer = nh.createTimer(ros::Duration(0.005), acados_ros::Timer_callback);

    FrameID frame_id = 0;

    if (unity_ready)
    {
        // render_process.
        ros::spin();
        // std::thread render_thread{render_process};
    }
    else
        ROS_ERROR("Unity not ready!");

    return 0;
}

// #endif

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/static_gate.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

// trajectory
#include <geometry_msgs/Transform.h>  // 用于表示位置和方�?
#include <nav_msgs/Path.h>            // 路径消息类型
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/trajectory_point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>  // 用于轨迹消息

using namespace flightlib;
// static void InitializeEnv() {}
int main(int argc, char* argv[]) {
  // initialize ROS
  ros::init(argc, argv, "acados_drone_racing_scene");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");
  ros::Rate(50.0);

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr = UnityBridge::getInstance();
  SceneID scene_id{UnityScene::WAREHOUSE};  // INDUSTRIAL
  bool unity_ready{false};

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>();
  QuadState quad_state;
  // initialization
  quad_state.setZero();
  quad_ptr->reset(quad_state);
  Vector<3> quad_size(0.5, 0.5, 0.5);
  quad_ptr->setSize(quad_size);

  std::string object_id = "unity_gate";
  std::string prefab_id = "rpg_gate";
  std::shared_ptr<StaticObject> gate_1 =
    std::make_shared<StaticObject>(object_id, prefab_id);
  gate_1->setPosition(Eigen::Vector3f(-10, 10, 2.5));
  gate_1->setQuaternion(
    Quaternion(std::cos(1 * M_PI_4), 0.0, 0.0, std::sin(1 * M_PI_4)));

  std::string object_id_2 = "unity_gate_2";
  std::shared_ptr<StaticGate> gate_2 =
    std::make_shared<StaticGate>(object_id_2);
  gate_2->setPosition(Eigen::Vector3f(-10, -10.0, 2.5));
  gate_2->setQuaternion(
    Quaternion(std::cos(0.33 * M_PI_4), 0.0, 0.0, std::sin(0.33 * M_PI_4)));

  std::string object_id_3 = "unity_gate_3";
  std::shared_ptr<StaticGate> gate_3 =
    std::make_shared<StaticGate>(object_id_3);
  gate_3->setPosition(Eigen::Vector3f(0, -10, 2.5));
  gate_3->setQuaternion(Quaternion(0.0, 0.0, 0.0, 1.0));

  std::string object_id_4 = "unity_gate_4";
  std::shared_ptr<StaticGate> gate_4 =
    std::make_shared<StaticGate>(object_id_4);
  gate_4->setPosition(Eigen::Vector3f(10, -10, 2.5));
  gate_4->setQuaternion(
    Quaternion(std::cos(1.25 * M_PI_4), 0.0, 0.0, std::cos(1.25 * M_PI_4)));

  std::string object_id_5 = "unity_gate_5";
  std::shared_ptr<StaticGate> gate_5 =
    std::make_shared<StaticGate>(object_id_5);
  gate_5->setPosition(Eigen::Vector3f(10, 0, 2.5));
  gate_5->setQuaternion(
    Quaternion(std::cos(1 * M_PI_4), 0.0, 0.0, std::sin(1 * M_PI_4)));



  // Initialize camera
  image_transport::Publisher rgb_pub;
  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);
  std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera->setFOV(90);
  rgb_camera->setWidth(640);
  rgb_camera->setHeight(480);
  rgb_camera->setRelPose(B_r_BC, R_BC);
  rgb_camera->setPostProcesscing(std::vector<bool>{
    false, false, false});  // depth, segmentation, optical flow
  quad_ptr->addRGBCamera(rgb_camera);


  // Start racing
  ros::Time t0 = ros::Time::now();

  unity_bridge_ptr->addStaticObject(gate_1);
  unity_bridge_ptr->addStaticObject(gate_2);
  unity_bridge_ptr->addStaticObject(gate_3);
  unity_bridge_ptr->addStaticObject(gate_4);
  unity_bridge_ptr->addStaticObject(gate_5);
  unity_bridge_ptr->addQuadrotor(quad_ptr);
  // connect unity
  unity_ready = unity_bridge_ptr->connectUnity(scene_id);

  FrameID frame_id = 0;




  while (ros::ok() && unity_ready) {
    ros::Time _timeStamp = ros::Time::now();


    unity_bridge_ptr->getRender(0);
    unity_bridge_ptr->handleOutput();


    // get img
    // cv::Mat img;
    // rgb_camera->getRGBImage(img);
    // sensor_msgs::ImagePtr rgb_msg =
    //   cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    // rgb_msg->header.stamp = _timeStamp;
    // rgb_pub.publish(rgb_msg);
    //
    frame_id += 1;
  }

  return 0;
}

// #endif

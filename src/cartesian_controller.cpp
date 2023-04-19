// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_cartesian_control/cartesian_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <stdlib.h>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <franka_cartesian_control/Delta.h>

namespace franka_cartesian_control {
bool CartesianPoseTactile::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");
    // Uncomment to set the check of the starting position
    // std::array<double, 7> q_start{{1.66, -0.545, -0.159, -2.735, -0.091, 2.187, 0.865}};
    // for (size_t i = 0; i < q_start.size(); i++) {
    //   if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
    //     ROS_ERROR_STREAM(
    //         "CartesianPoseExampleController: Robot is not in the expected starting position for "
    //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
    //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
    //     return false;
    //   }
    // }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }
  service = node_handle.advertiseService("delta_pose", &CartesianPoseTactile::response, this);
  node_handle.param<double>("/trajectory_time", trajectory_time, 3.0);
  if (trajectory_time <= 1.0){
    ROS_ERROR("Trajectory time specified must be greater than 1.0");
    return false;
  }
  delta_z = 0;
  delta_x = 0;
  delta_y = 0;
  ROS_INFO("Trajectory time: %f", trajectory_time);
  return true;
}

double CartesianPoseTactile::quintic(double ref, double r_, ros::Duration time, double dt){
  double a0 = r_;
  double a3 = 10 * (ref - r_) / pow(dt, 3);
  double a4 = -15 * (ref - r_) / pow(dt, 4);
  double a5 = 6 * (ref - r_) / pow(dt, 5);
  double t = time.toSec();
  double res = 0;
  if (t > dt){
    return ref;
  }else{
    res = a0 + a3 * pow(t,3) + a4 * pow(t,4) + a5 * pow(t,5);
  }
  return res;
}

bool CartesianPoseTactile::response(franka_cartesian_control::Delta::Request& req, franka_cartesian_control::Delta::Response& res){
  delta_x = req.dx;
  delta_y = req.dy;
  delta_z = req.dz;
  read_message = true;
  pose_now = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  ROS_INFO("Pose now: x - %f | y - %f | z - %f", pose_now[12], pose_now[13], pose_now[14]);
  pose_cmd = pose_now;
  res.result = true;
}

void CartesianPoseTactile::starting(const ros::Time& /* time */) {
  pose_now = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianPoseTactile::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  if (read_message){
    elapsed_time_ = ros::Duration(0.0);
    read_message = false;
  }

  double target_x = quintic(pose_now[12] + delta_x, pose_now[12], elapsed_time_, trajectory_time);
  double target_y = quintic(pose_now[13] + delta_y, pose_now[13], elapsed_time_, trajectory_time);
  double target_z = quintic(pose_now[14] + delta_z, pose_now[14], elapsed_time_, trajectory_time);

  pose_cmd[12] = target_x;
  pose_cmd[13] = target_y;
  pose_cmd[14] = target_z;
  // ROS_INFO("Delta_x %f | Delta_y %f | Delta_z %f", target_x, target_y, target_z);
  cartesian_pose_handle_->setCommand(pose_cmd);
}
}
PLUGINLIB_EXPORT_CLASS(franka_cartesian_control::CartesianPoseTactile,
                       controller_interface::ControllerBase)

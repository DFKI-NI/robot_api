/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2018, DFKI GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Ioan Sucan, Martin Günther, Alexander Sung */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_api_msgs/action/move_it_macro.hpp"
#include <thread>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

// gripper
#include <control_msgs/action/parallel_gripper_command.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sstream>

namespace robot
{
std::string tf_prefix_ = "mobipick";

struct GraspPoseDefine
{
  Eigen::Isometry3d grasp_pose;
  std::float_t gripper_width;
};

void openGripper(trajectory_msgs::msg::JointTrajectory& posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = tf_prefix_ + "gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.1;

  posture.points[0].effort.resize(1);
  posture.points[0].effort[0] = 30;
  posture.points[0].time_from_start = rclcpp::Duration::from_seconds(5.0);
}

void closedGripper(trajectory_msgs::msg::JointTrajectory& posture, std::float_t gripper_width = 0.63)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = tf_prefix_ + "gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] =
      gripper_width;  // closed around power drill: 0.65; fully closed: 0.76  TODO: should be 0.42 for top grasp

  posture.points[0].effort.resize(1);
  posture.points[0].effort[0] = 80;
  posture.points[0].time_from_start = rclcpp::Duration::from_seconds(5.0);
}

moveit::core::MoveItErrorCode move(moveit::planning_interface::MoveGroupInterface& group, double dx = 0.0,
                                                 double dy = 0.0, double dz = 0.0, double droll = 0.0,
                                                 double dpitch = 0.0, double dyaw = 0.0)
{
  moveit::core::RobotState start_state(*group.getCurrentState());
  group.setStartState(start_state);
  Eigen::Isometry3d pose;
  geometry_msgs::msg::PoseStamped current_pose = group.getCurrentPose();
  geometry_msgs::msg::PoseStamped target_pose = current_pose;
  tf2::fromMsg(current_pose.pose, pose);
  pose.translate(Eigen::Vector3d(dx, dy, dz));
  pose.rotate(Eigen::AngleAxisd(dyaw, Eigen::Vector3d(0.0, 0.0, 1.0)));
  pose.rotate(Eigen::AngleAxisd(dpitch, Eigen::Vector3d(0.0, 1.0, 0.0)));
  pose.rotate(Eigen::AngleAxisd(droll, Eigen::Vector3d(1.0, 0.0, 0.0)));
  target_pose.pose = tf2::toMsg(pose);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("moveit_macros"), "Target pose frame: " << target_pose.header.frame_id);
  group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto error_code = group.plan(my_plan);
  bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(rclcpp::get_logger("moveit_macros"), "Move planning (pose goal) %s", success ? "" : "FAILED");
  if (success)
  {
    error_code = group.execute(my_plan);
  }
  return error_code;
}

moveit::core::MoveItErrorCode moveToCartPose(moveit::planning_interface::MoveGroupInterface& group,
                                                           Eigen::Isometry3d cartesian_pose,
                                                           std::string base_frame = tf_prefix_ + "ur5_base_link",
                                                           std::string target_frame = tf_prefix_ + "gripper_tcp")
{
  moveit::core::RobotState start_state(*group.getCurrentState());
  group.setStartState(start_state);
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = base_frame;
  target_pose.pose = tf2::toMsg(cartesian_pose);
  group.setPoseTarget(target_pose, target_frame);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto error_code = group.plan(my_plan);
  bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(rclcpp::get_logger("moveit_macros"), "Move planning (pose goal) %s", success ? "" : "FAILED");
  if (success)
  {
    error_code = group.execute(my_plan);
  }
  return error_code;
}

}  // namespace robot

using namespace robot;

class MoveItMacroAction : public rclcpp::Node
{
public:
  using MoveItMacro = robot_api_msgs::action::MoveItMacro;
  using GoalHandleMoveItMacro = rclcpp_action::ServerGoalHandle<MoveItMacro>;
  using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<ParallelGripperCommand>;

  MoveItMacroAction(const rclcpp::NodeOptions& options) : Node("moveit_macros", options)
  {
    this->declare_parameter<std::string>("tf_prefix", "mobipick");
    tf_prefix_ = this->get_parameter("tf_prefix").as_string();
    if (!tf_prefix_.empty() && tf_prefix_.back() != '/')
    {
        tf_prefix_ += "/";
    }

    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](auto*){});

    group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "arm");
    group_ptr_->setPlanningTime(45.0);
    group_ptr_->setPlannerId("RRTConnect");

    planning_scene_interface_ptr_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    gripper_ac_ptr_ = rclcpp_action::create_client<ParallelGripperCommand>(this, "gripper_hw");
    if (!gripper_ac_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Gripper action server not available after waiting");
    } else {
        RCLCPP_INFO(this->get_logger(), "Connected to gripper action server");
    }

    registerMoveItFunctions();

    this->action_server_ = rclcpp_action::create_server<MoveItMacro>(
        this,
        "moveit_macros",
        std::bind(&MoveItMacroAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveItMacroAction::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveItMacroAction::handle_accepted, this, std::placeholders::_1));
  }

  bool hasAttachedObjects()
  {
    return !planning_scene_interface_ptr_->getAttachedObjects().empty();
  }

  geometry_msgs::msg::PoseStamped getCurrentPose()
  {
    return group_ptr_->getCurrentPose();
  }

  bool moveArmToTarget(const std::string& target_name)
  {
    group_ptr_->setPlannerId("RRTConnect");
    group_ptr_->setStartStateToCurrentState();
    group_ptr_->setNamedTarget(target_name);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode error_code = group_ptr_->plan(my_plan);
    if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Planning to " << target_name << " pose FAILED");
      return false;
    }
    error_code = group_ptr_->execute(my_plan);
    if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Moving to " << target_name << " pose FAILED");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Moving to " << target_name << " pose SUCCESSFUL");
    return true;
  }

  bool releaseGripper()
  {
    auto gripper_goal = ParallelGripperCommand::Goal();
    gripper_goal.command.position = std::vector<double>{0.1};
    gripper_goal.command.effort = std::vector<double>{30.0};
    if (!gripper_ac_ptr_->wait_for_action_server(std::chrono::seconds(0)))
    {
        RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
        return false;
    }
    auto goal_handle_future = gripper_ac_ptr_->async_send_goal(gripper_goal);
    if (rclcpp::spin_until_future_complete(std::shared_ptr<rclcpp::Node>(this, [](auto*){}), goal_handle_future, std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "send goal call failed");
        return false;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        return false;
    }
    auto result_future = gripper_ac_ptr_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(std::shared_ptr<rclcpp::Node>(this, [](auto*){}), result_future, std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "get result call failed");
        return false;
    }
    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Gripper move SUCCESSFUL, detach all Objects");
      auto attached_objects = planning_scene_interface_ptr_->getAttachedObjects();
      std::vector<std::string> objects_to_remove;
      for (auto&& object : attached_objects)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Detach object " << object.first);
        group_ptr_->detachObject(object.first);
        objects_to_remove.push_back(object.first);
      }
      planning_scene_interface_ptr_->removeCollisionObjects(objects_to_remove);
      group_ptr_->clearPathConstraints();
      return true;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Gripper move FAILED");
      return false;
    }
  }


private:
  typedef bool (MoveItMacroAction::*moveit_function_t)();
  std::map<std::string, moveit_function_t> moveit_functions;

  moveit::planning_interface::MoveGroupInterfacePtr group_ptr_;
  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_ptr_;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_ac_ptr_;
  rclcpp_action::Server<MoveItMacro>::SharedPtr action_server_;

  void registerMoveItFunctions()
  {
    moveit_functions["HasAttachedObjects"] = &MoveItMacroAction::hasAttachedObjects;
    moveit_functions["ReleaseGripper"] = &MoveItMacroAction::releaseGripper;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveItMacro::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with name %s", goal->name.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveItMacro> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveItMacro> goal_handle)
  {
    std::thread{std::bind(&MoveItMacroAction::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveItMacro> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveItMacro::Result>();
    if (goal->type == "target")
    {
      result->result = moveArmToTarget(goal->name);
    }
    else if (goal->type == "function")
    {
      auto it = moveit_functions.find(goal->name);
      if (it != moveit_functions.end())
      {
        result->result = (this->*(it->second))();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Invalid function name '%s' in moveit macro.", goal->name.c_str());
        result->result = false;
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid type '%s' used in moveit macro.", goal->type.c_str());
      result->result = false;
    }

    if (rclcpp::ok()) {
        goal_handle->succeed(result);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MoveItMacroAction>(node_options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

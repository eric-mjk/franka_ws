#pragma once

// dual_arm/dual_arm_task.hpp

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace mtc = moveit::task_constructor;

namespace dual_arm
{

struct RobotConfig
{
  std::string left_arm_group   = "left_fr3_arm";
  std::string right_arm_group  = "right_fr3_arm";
  std::string left_hand_group  = "left_fr3_hand";
  std::string right_hand_group = "right_fr3_hand";
  std::string left_eef         = "left_hand";
  std::string right_eef        = "right_hand";
  std::string left_hand_frame  = "left_fr3_hand";
  std::string right_hand_frame = "right_fr3_hand";
  std::string gripper_open     = "open";
  std::string gripper_close    = "close";
  std::string arm_home         = "ready";
};

struct PickPlaceParams
{
  std::string object_id       = "target_object";
  std::string support_surface = "table";
  geometry_msgs::msg::PoseStamped pick_pose;
  geometry_msgs::msg::PoseStamped place_pose;
  double approach_min    = 0.05;
  double approach_max    = 0.15;
  double lift_min        = 0.05;
  double lift_max        = 0.20;
  double retreat_min     = 0.05;
  double retreat_max     = 0.20;
  int    max_ik_solutions = 8;
  double connect_timeout  = 5.0;
};

class DualArmTask
{
public:
  explicit DualArmTask(const rclcpp::Node::SharedPtr & node,
                       const RobotConfig & cfg = RobotConfig{});

  void setupPlanningScene(
    const std::string & object_id   = "target_object",
    const std::string & surface_id  = "table",
    const geometry_msgs::msg::PoseStamped & object_pose  = defaultObjectPose(),
    const geometry_msgs::msg::PoseStamped & surface_pose = defaultSurfacePose());

  mtc::Task createMoveHomeTask(const std::string & arm_group);
  mtc::Task createLeftArmPickPlaceTask(const PickPlaceParams & params);
  mtc::Task createRightArmPickPlaceTask(const PickPlaceParams & params);
  bool      planAndExecute(mtc::Task & task, std::size_t max_solutions = 5);

private:
  rclcpp::Node::SharedPtr node_;
  RobotConfig cfg_;

  std::shared_ptr<mtc::solvers::PipelinePlanner>           sampling_planner_;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath>             cartesian_planner_;

  void initPlanners();

  std::unique_ptr<mtc::SerialContainer> buildPickContainer(
    const PickPlaceParams & params,
    const std::string & arm_group,
    const std::string & hand_group,
    const std::string & hand_frame,
    const moveit::core::RobotModelConstPtr & robot_model,
    mtc::Stage *& current_state_ptr_out,
    mtc::Stage *& attach_stage_ptr_out);

  std::unique_ptr<mtc::SerialContainer> buildPlaceContainer(
    const PickPlaceParams & params,
    const std::string & arm_group,
    const std::string & hand_group,
    const std::string & hand_frame,
    const moveit::core::RobotModelConstPtr & robot_model,
    mtc::Stage * attach_stage_ptr);

  static geometry_msgs::msg::PoseStamped defaultObjectPose();
  static geometry_msgs::msg::PoseStamped defaultSurfacePose();
};

}  // namespace dual_arm
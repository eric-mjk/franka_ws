// mtc_handover.cpp
//
// STAGE ARCHITECTURE (correct MTC handover pattern):
//
//  [FixedState]                   ← generator, seeds arm at "ready" with finger=0
//  [Connect: right arm → pick]    ← planner moves right arm to pre-grasp
//  [SerialContainer: pick]
//    MoveRelative approach         ← right arm moves toward box (Cartesian)
//    GenerateGraspPose + IK        ← generates right arm grasp configs
//    (ModifyPlanningScene: allow collision)  [real hw only]
//    (MoveTo: close right hand)             [real hw only]
//    ModifyPlanningScene: attach box
//    MoveRelative: lift            ← lifts box
//  [Connect: both arms → handover] ← OMPL moves both arms to pre-handover config
//  [SerialContainer: handover]
//    GenerateGraspPose + IK        ← generates left arm grasp configs on box
//    (ModifyPlanningScene: allow collision)  [real hw only]
//    (MoveTo: close left hand)              [real hw only]
//    ModifyPlanningScene: attach left / detach right
//    MoveRelative: right arm retreat  ← right arm backs away (world frame)
//  [Connect: left arm → place]    ← OMPL moves left arm to pre-place
//  [SerialContainer: place]
//    GeneratePlacePose + IK        ← generates place configs
//    (ModifyPlanningScene: restore collision)  [real hw only]
//    (MoveTo: open left hand)               [real hw only]
//    ModifyPlanningScene: detach left
//    MoveRelative: retreat upward
//  [MoveTo: left arm → home]
//
// KEY DESIGN RULES:
//  - MoveRelative INSIDE a container = Cartesian motion relative to IK solution
//  - Connect OUTSIDE a container = OMPL joint-space motion between states
//  - Never put a MoveRelative BEFORE the first IK stage in a container —
//    it has no valid start state to propagate from at planning time.
//    (The pre-approach BEFORE GenerateGraspPose is correct because the IK
//     generator propagates states BACKWARD through it.)
//  - The handover container has NO pre-approach: Connect already brings
//    the left arm near the box. GenerateGraspPose seeds the IK directly.

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>
#include <thread>
#include <memory>
#include <vector>
#include <string>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#  include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#  include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#  include <tf2_eigen/tf2_eigen.hpp>
#else
#  include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("handover.mtc");
namespace mtc = moveit::task_constructor;

static constexpr char RIGHT_ARM[]   = "right_fr3_arm";
static constexpr char LEFT_ARM[]    = "left_fr3_arm";
static constexpr char RIGHT_HAND[]  = "right_fr3_hand";
static constexpr char LEFT_HAND[]   = "left_fr3_hand";
static constexpr char RIGHT_EEF[]   = "right_fr3_hand";
static constexpr char LEFT_EEF[]    = "left_fr3_hand";
static constexpr char RIGHT_FRAME[] = "right_fr3_hand";
static constexpr char LEFT_FRAME[]  = "left_fr3_hand";
static constexpr char HOME[]        = "ready";
static constexpr char OPEN[]        = "open";
static constexpr char CLOSE[]       = "close";
static constexpr char OBJ[]         = "target_box";
static constexpr char SURFACE[]     = "table";

// Verify with: ros2 topic echo /joint_states --once
static const std::vector<std::string> FINGER_JOINTS = {
  "right_fr3_finger_joint1",
  "right_fr3_finger_joint2",
  "left_fr3_finger_joint1",
  "left_fr3_finger_joint2",
};

class MTCTaskNode
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions & options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  bool use_fake_hardware_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("mtc_node", options))
{
  use_fake_hardware_ = node_->declare_parameter<bool>("use_fake_hardware", true);
  RCLCPP_INFO(LOGGER, "use_fake_hardware = %s",
    use_fake_hardware_ ? "true (FixedState, no gripper stages)" : "false (real hw)");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCTaskNode::getNodeBaseInterface()
{ return node_->get_node_base_interface(); }

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  {  // Table
    moveit_msgs::msg::CollisionObject obj;
    obj.id = SURFACE; obj.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX; prim.dimensions = {1.2, 1.6, 0.05};
    geometry_msgs::msg::Pose p;
    p.position.x = 0.5; p.position.z = -0.025; p.orientation.w = 1.0;
    obj.primitives.push_back(prim); obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;
    psi.applyCollisionObject(obj);
  }
  {  // Target box — right arm side (positive y)
    moveit_msgs::msg::CollisionObject obj;
    obj.id = OBJ; obj.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX; prim.dimensions = {0.06, 0.06, 0.12};
    geometry_msgs::msg::Pose p;
    p.position.x = 0.5; p.position.y = 0.2; p.position.z = 0.06;
    p.orientation.w = 1.0;
    obj.primitives.push_back(prim); obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;
    psi.applyCollisionObject(obj);
  }
  RCLCPP_INFO(LOGGER, "Planning scene set up.");
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("Handover Task");
  task.loadRobotModel(node_);

  task.setProperty("group",    std::string(RIGHT_ARM));
  task.setProperty("eef",      std::string(RIGHT_EEF));
  task.setProperty("ik_frame", std::string(RIGHT_FRAME));

  // ── Planners ────────────────────────────────────────────────────────────────
  auto sampling  = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling->setPlannerId("RRTConnect");

  auto interp    = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setMaxVelocityScalingFactor(0.1);
  cartesian->setMaxAccelerationScalingFactor(0.1);
  cartesian->setStepSize(0.005);
  cartesian->setMinFraction(0.9);  // require 90% Cartesian success

  // Grasp frame: 10 cm offset + rotation to align hand z-axis with approach
  auto make_grasp_frame = []() -> Eigen::Isometry3d {
    Eigen::Isometry3d f = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    f.linear() = q.matrix();
    f.translation().z() = 0.10;
    return f;
  };

  // ── Stage 0: starting state ─────────────────────────────────────────────────
  // fake_hw: FixedState with finger joints = 0 (matches IK-generated states)
  // real_hw: CurrentState (actual joint values)
  mtc::Stage * current_state_ptr = nullptr;
  if (use_fake_hardware_) {
    auto robot_model = task.getRobotModel();
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    auto & rs = scene->getCurrentStateNonConst();

    // Start from zero/default for ALL joints (safe baseline)
    rs.setToDefaultValues();

    const auto * r_jmg = robot_model->getJointModelGroup(RIGHT_ARM);
    const auto * l_jmg = robot_model->getJointModelGroup(LEFT_ARM);

    // Try to set the "ready" named state — log clearly if it doesn't exist
    if (r_jmg) {
      if (rs.setToDefaultValues(r_jmg, HOME)) {
        RCLCPP_INFO(LOGGER, "Right arm set to named state '%s'", HOME);
      } else {
        RCLCPP_WARN(LOGGER,
          "Named state '%s' NOT found for group '%s'. "
          "Check your SRDF. Using zero configuration instead.", HOME, RIGHT_ARM);
        // Print all available named states for this group
        for (const auto & ns : r_jmg->getDefaultStateNames()) {
          RCLCPP_WARN(LOGGER, "  Available right arm state: '%s'", ns.c_str());
        }
      }
    } else {
      RCLCPP_ERROR(LOGGER, "JointModelGroup '%s' not found! Check SRDF group names.", RIGHT_ARM);
    }

    if (l_jmg) {
      if (rs.setToDefaultValues(l_jmg, HOME)) {
        RCLCPP_INFO(LOGGER, "Left arm set to named state '%s'", HOME);
      } else {
        RCLCPP_WARN(LOGGER,
          "Named state '%s' NOT found for group '%s'. "
          "Check your SRDF. Using zero configuration instead.", HOME, LEFT_ARM);
        for (const auto & ns : l_jmg->getDefaultStateNames()) {
          RCLCPP_WARN(LOGGER, "  Available left arm state: '%s'", ns.c_str());
        }
      }
    } else {
      RCLCPP_ERROR(LOGGER, "JointModelGroup '%s' not found! Check SRDF group names.", LEFT_ARM);
    }

    // Clamp finger joints to 0 (fake_hw parks them at ~0.035)
    for (const auto & jname : FINGER_JOINTS) {
      const auto * jm = robot_model->getJointModel(jname);
      if (!jm) { RCLCPP_WARN(LOGGER, "Finger joint '%s' not found — skipping", jname.c_str()); continue; }
      std::vector<double> zero(jm->getVariableCount(), 0.0);
      rs.setJointPositions(jm, zero);
    }
    rs.update();

    // Populate the scene with the same collision objects as setupPlanningScene().
    // FixedState uses this scene AS-IS — no merge with the monitored planning scene —
    // so GenerateGraspPose would find an empty world and produce 0 candidates
    // without these objects.
    {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = SURFACE; obj.header.frame_id = "world";
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = prim.BOX; prim.dimensions = {1.2, 1.6, 0.05};
      geometry_msgs::msg::Pose p;
      p.position.x = 0.5; p.position.z = -0.025; p.orientation.w = 1.0;
      obj.primitives.push_back(prim); obj.primitive_poses.push_back(p);
      obj.operation = obj.ADD;
      scene->processCollisionObjectMsg(obj);
    }
    {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = OBJ; obj.header.frame_id = "world";
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = prim.BOX; prim.dimensions = {0.06, 0.06, 0.12};
      geometry_msgs::msg::Pose p;
      p.position.x = 0.5; p.position.y = 0.2; p.position.z = 0.06;
      p.orientation.w = 1.0;
      obj.primitives.push_back(prim); obj.primitive_poses.push_back(p);
      obj.operation = obj.ADD;
      scene->processCollisionObjectMsg(obj);
    }

    // Check if the start state is in self-collision — if so, Connect rejects in <1ms
    collision_detection::CollisionRequest creq;
    creq.contacts = true; creq.max_contacts = 5;
    collision_detection::CollisionResult cres;
    scene->checkSelfCollision(creq, cres, rs);
    if (cres.collision) {
      RCLCPP_ERROR(LOGGER,
        "START STATE IS IN SELF-COLLISION! Connect will reject immediately. "
        "Fix: use a valid named state in SRDF or adjust joint values.");
      for (const auto & c : cres.contacts) {
        RCLCPP_ERROR(LOGGER, "  Collision: %s <-> %s",
          c.first.first.c_str(), c.first.second.c_str());
      }
    } else {
      RCLCPP_INFO(LOGGER, "Start state collision check: OK (no self-collision)");
    }

    // Log the actual joint values being used
    std::vector<double> joint_vals;
    rs.copyJointGroupPositions(r_jmg, joint_vals);
    std::string jv_str;
    for (double v : joint_vals) jv_str += std::to_string(v) + " ";
    RCLCPP_INFO(LOGGER, "Right arm joint values: [%s]", jv_str.c_str());
    if (l_jmg) {
      rs.copyJointGroupPositions(l_jmg, joint_vals);
      jv_str.clear();
      for (double v : joint_vals) jv_str += std::to_string(v) + " ";
      RCLCPP_INFO(LOGGER, "Left arm joint values:  [%s]", jv_str.c_str());
    }

    auto s = std::make_unique<mtc::stages::FixedState>("fixed start state");
    s->setState(scene);
    current_state_ptr = s.get();
    task.add(std::move(s));
    RCLCPP_INFO(LOGGER, "FixedState added as start stage.");
  } else {
    auto s = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = s.get();
    task.add(std::move(s));
  }

  // ── Stage 1: open right hand [real hw only] ──────────────────────────────
  if (!use_fake_hardware_) {
    auto s = std::make_unique<mtc::stages::MoveTo>("open right hand", interp);
    s->setGroup(RIGHT_HAND);
    s->setGoal(OPEN);
    task.add(std::move(s));
  }

  // ── Stage 2: Connect — right arm to pre-grasp ───────────────────────────
  {
    auto s = std::make_unique<mtc::stages::Connect>(
      "right arm -> pick",
      mtc::stages::Connect::GroupPlannerVector{{RIGHT_ARM, sampling}});
    s->setTimeout(10.0);
    s->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(s));
  }

  // ── Stages 3–N: pick container ──────────────────────────────────────────
  // Pattern: approach (Cartesian, backward from grasp) → IK → grasp → lift
  // The MoveRelative approach propagates BACKWARD through the IK stage —
  // MTC solves it in reverse, so putting approach BEFORE GenerateGraspPose
  // is correct. The IK generator seeds the approach endpoint.
  mtc::Stage * attach_stage_ptr = nullptr;
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
    pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach: move toward box along hand z-axis (solved in reverse by MTC)
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian);
      s->properties().set("marker_ns", "approach");
      s->properties().set("link", std::string(RIGHT_FRAME));
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setMinMaxDistance(0.05, 0.15);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = RIGHT_FRAME;
      vec.vector.z = 1.0;  // +z = approach direction in hand frame
      s->setDirection(vec);
      pick->insert(std::move(s));
    }

    // Generate grasp pose + IK
    {
      auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->properties().set("marker_ns", "grasp_pose");
      gen->setPreGraspPose(OPEN);
      gen->setObject(OBJ);
      gen->setAngleDelta(M_PI / 12);  // sample 24 orientations around object
      gen->setMonitoredStage(current_state_ptr);

      auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp IK", std::move(gen));
      ik->setMaxIKSolutions(8);
      ik->setMinSolutionDistance(0.1);
      ik->setIKFrame(make_grasp_frame(), RIGHT_FRAME);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      pick->insert(std::move(ik));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (right hand,box)");
      s->allowCollisions(OBJ, *task.getRobotModel()->getJointModelGroup(RIGHT_HAND), true);
      pick->insert(std::move(s));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::MoveTo>("close right hand", interp);
      s->setGroup(RIGHT_HAND); s->setGoal(CLOSE);
      pick->insert(std::move(s));
    }

    // Attach box — this stage pointer is monitored by the handover IK generator
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach box -> right");
      s->attachObject(OBJ, RIGHT_FRAME);
      attach_stage_ptr = s.get();
      pick->insert(std::move(s));
    }

    // Lift: move upward in world frame after grasp
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian);
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setMinMaxDistance(0.05, 0.15);
      s->setIKFrame(RIGHT_FRAME);
      s->properties().set("marker_ns", "lift");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      s->setDirection(vec);
      pick->insert(std::move(s));
    }

    task.add(std::move(pick));
  }

  // ── Connect — both arms to handover position ─────────────────────────────
  // This is where OMPL plans both arms simultaneously into a configuration
  // where the left arm can reach the box held by the right arm.
  {
    auto s = std::make_unique<mtc::stages::Connect>(
      "move to handover",
      mtc::stages::Connect::GroupPlannerVector{
        {RIGHT_ARM, sampling},
        {LEFT_ARM,  sampling}});
    s->setTimeout(20.0);
    s->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(s));
  }

  // ── Handover container ───────────────────────────────────────────────────
  // NO MoveRelative pre-approach here. The Connect stage already brought
  // both arms into a pre-handover configuration. GenerateGraspPose generates
  // the left arm's grasp pose on the box in its current (post-Connect) position.
  mtc::Stage * handover_attach_ptr = nullptr;
  {
    auto handover = std::make_unique<mtc::SerialContainer>("handover");
    // Only expose eef+group — do NOT expose ik_frame (type mismatch: the task
    // level ik_frame is a string for the right arm; left arm IK uses Eigen overload)
    task.properties().exposeTo(handover->properties(), {"eef", "group"});
    handover->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    handover->properties().set("group", std::string(LEFT_ARM));
    handover->properties().set("eef",   std::string(LEFT_EEF));

    // Generate left-arm grasp pose on the box + IK
    // Monitored stage is attach_stage_ptr (box is now attached to right hand)
    {
      auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate handover pose");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->properties().set("marker_ns", "handover_pose");
      gen->setPreGraspPose(OPEN);
      gen->setObject(OBJ);
      gen->setAngleDelta(M_PI / 12);
      gen->setMonitoredStage(attach_stage_ptr);

      auto ik = std::make_unique<mtc::stages::ComputeIK>("handover IK", std::move(gen));
      ik->setMaxIKSolutions(8);
      ik->setMinSolutionDistance(0.1);
      ik->setIKFrame(make_grasp_frame(), LEFT_FRAME);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      handover->insert(std::move(ik));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::MoveTo>("open left hand", interp);
      s->setGroup(LEFT_HAND); s->setGoal(OPEN);
      handover->insert(std::move(s));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (left hand,box)");
      s->allowCollisions(OBJ, *task.getRobotModel()->getJointModelGroup(LEFT_HAND), true);
      handover->insert(std::move(s));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::MoveTo>("close left hand", interp);
      s->setGroup(LEFT_HAND); s->setGoal(CLOSE);
      handover->insert(std::move(s));
    }

    // Transfer: left arm takes ownership, right arm releases
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach box -> left");
      s->attachObject(OBJ, LEFT_FRAME);
      handover_attach_ptr = s.get();
      handover->insert(std::move(s));
    }
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach box <- right");
      s->detachObject(OBJ, RIGHT_FRAME);
      handover->insert(std::move(s));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("restore collision (right hand,box)");
      s->allowCollisions(OBJ, *task.getRobotModel()->getJointModelGroup(RIGHT_HAND), false);
      handover->insert(std::move(s));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::MoveTo>("open right hand", interp);
      s->setGroup(RIGHT_HAND); s->setGoal(OPEN);
      handover->insert(std::move(s));
    }

    // Right arm retreats sideways (world frame) after releasing box
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("right arm retreat", cartesian);
      s->properties().set("group", std::string(RIGHT_ARM));
      s->setIKFrame(RIGHT_FRAME);
      s->setMinMaxDistance(0.05, 0.20);
      s->properties().set("marker_ns", "right_retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.y = -1.0;  // retreat in -y (away from left arm side)
      s->setDirection(vec);
      handover->insert(std::move(s));
    }

    task.add(std::move(handover));
  }

  // ── Connect — left arm to place ─────────────────────────────────────────
  {
    auto s = std::make_unique<mtc::stages::Connect>(
      "left arm -> place",
      mtc::stages::Connect::GroupPlannerVector{{LEFT_ARM, sampling}});
    s->setTimeout(10.0);
    s->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(s));
  }

  // ── Place container ──────────────────────────────────────────────────────
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    place->properties().set("group", std::string(LEFT_ARM));
    place->properties().set("eef",   std::string(LEFT_EEF));

    // Generate place pose + IK
    {
      auto gen = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->properties().set("marker_ns", "place_pose");
      gen->setObject(OBJ);
      gen->setMonitoredStage(handover_attach_ptr);

      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = "world";
      place_pose.pose.position.x =  0.5;
      place_pose.pose.position.y = -0.2;  // left arm side
      place_pose.pose.position.z =  0.06;
      place_pose.pose.orientation.w = 1.0;
      gen->setPose(place_pose);

      auto ik = std::make_unique<mtc::stages::ComputeIK>("place IK", std::move(gen));
      ik->setMaxIKSolutions(4);
      ik->setMinSolutionDistance(0.1);
      ik->setIKFrame(make_grasp_frame(), LEFT_FRAME);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(ik));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::MoveTo>("open left hand", interp);
      s->setGroup(LEFT_HAND); s->setGoal(OPEN);
      place->insert(std::move(s));
    }

    if (!use_fake_hardware_) {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("restore collision (left hand,box)");
      s->allowCollisions(OBJ, *task.getRobotModel()->getJointModelGroup(LEFT_HAND), false);
      place->insert(std::move(s));
    }

    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach box <- left");
      s->detachObject(OBJ, LEFT_FRAME);
      place->insert(std::move(s));
    }

    // Retreat upward after placing
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setMinMaxDistance(0.05, 0.15);
      s->setIKFrame(LEFT_FRAME);
      s->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      s->setDirection(vec);
      place->insert(std::move(s));
    }

    task.add(std::move(place));
  }

  // ── Return home ──────────────────────────────────────────────────────────
  {
    auto s = std::make_unique<mtc::stages::MoveTo>("left arm -> home", interp);
    s->setGroup(LEFT_ARM);
    s->setGoal(HOME);
    task.add(std::move(s));
  }

  return task;
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try {
    task_.init();
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Init failed:\n" << e);
    return;
  }

  task_.enableIntrospection(true);

  RCLCPP_INFO(LOGGER, "=== Planning (max 5 solutions) ===");
  auto plan_ret = task_.plan(5);
  const bool plan_ok = static_cast<bool>(plan_ret) || !task_.solutions().empty();

  RCLCPP_INFO(LOGGER, "Total task solutions: %zu  (planning took ~%s)",
    task_.solutions().size(), plan_ok ? "OK" : "FAILED");

  if (!plan_ok) {
    RCLCPP_ERROR(LOGGER, "Planning FAILED. Running diagnostic mini-task to isolate failing stage...");

    // ── Diagnostic: minimal pick-only task ──────────────────────────────────
    // Build the smallest possible task that exercises the same IK setup.
    // If THIS also fails instantly, ComputeIK / eef config is wrong.
    // If it succeeds, the failure is in Connect or the handover container.
    mtc::Task diag;
    diag.stages()->setName("DIAG: pick only");
    diag.loadRobotModel(node_);
    diag.setProperty("group",    std::string(RIGHT_ARM));
    diag.setProperty("eef",      std::string(RIGHT_EEF));
    diag.setProperty("ik_frame", std::string(RIGHT_FRAME));

    auto diag_cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    diag_cartesian->setMaxVelocityScalingFactor(0.1);
    diag_cartesian->setMaxAccelerationScalingFactor(0.1);
    diag_cartesian->setStepSize(0.005);
    diag_cartesian->setMinFraction(0.5);  // relaxed for diagnostic

    auto diag_sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

    // Re-use the same FixedState scene already built
    mtc::Stage * diag_fixed_ptr = nullptr;
    {
      auto robot_model = diag.getRobotModel();
      auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
      auto & rs = scene->getCurrentStateNonConst();
      rs.setToDefaultValues();
      const auto * r_jmg = robot_model->getJointModelGroup(RIGHT_ARM);
      const auto * l_jmg = robot_model->getJointModelGroup(LEFT_ARM);
      if (r_jmg) rs.setToDefaultValues(r_jmg, HOME);
      if (l_jmg) rs.setToDefaultValues(l_jmg, HOME);
      for (const auto & jname : FINGER_JOINTS) {
        const auto * jm = robot_model->getJointModel(jname);
        if (jm) { std::vector<double> z(jm->getVariableCount(),0.0); rs.setJointPositions(jm,z); }
      }
      rs.update();
      {
        moveit_msgs::msg::CollisionObject obj;
        obj.id = SURFACE; obj.header.frame_id = "world";
        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX; prim.dimensions = {1.2, 1.6, 0.05};
        geometry_msgs::msg::Pose p;
        p.position.x = 0.5; p.position.z = -0.025; p.orientation.w = 1.0;
        obj.primitives.push_back(prim); obj.primitive_poses.push_back(p);
        obj.operation = obj.ADD;
        scene->processCollisionObjectMsg(obj);
      }
      {
        moveit_msgs::msg::CollisionObject obj;
        obj.id = OBJ; obj.header.frame_id = "world";
        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX; prim.dimensions = {0.06, 0.06, 0.12};
        geometry_msgs::msg::Pose p;
        p.position.x = 0.5; p.position.y = 0.2; p.position.z = 0.06;
        p.orientation.w = 1.0;
        obj.primitives.push_back(prim); obj.primitive_poses.push_back(p);
        obj.operation = obj.ADD;
        scene->processCollisionObjectMsg(obj);
      }
      auto s = std::make_unique<mtc::stages::FixedState>("diag start");
      s->setState(scene);
      diag_fixed_ptr = s.get();
      diag.add(std::move(s));
    }

    // Connect right arm to pick position
    {
      auto s = std::make_unique<mtc::stages::Connect>(
        "diag connect",
        mtc::stages::Connect::GroupPlannerVector{{RIGHT_ARM, diag_sampling}});
      s->setTimeout(5.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      diag.add(std::move(s));
    }

    // Minimal pick container: approach + IK only (no attach/lift)
    {
      auto pick = std::make_unique<mtc::SerialContainer>("diag pick");
      diag.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
      pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

      // Approach
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("diag approach", diag_cartesian);
        s->properties().set("link", std::string(RIGHT_FRAME));
        s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        s->setMinMaxDistance(0.01, 0.15);  // very relaxed min
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = RIGHT_FRAME; vec.vector.z = 1.0;
        s->setDirection(vec);
        pick->insert(std::move(s));
      }

      // IK — use NULL grasp frame offset first to test basic IK reachability
      {
        auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("diag grasp pose");
        gen->properties().configureInitFrom(mtc::Stage::PARENT);
        gen->setPreGraspPose(OPEN);
        gen->setObject(OBJ);
        gen->setAngleDelta(M_PI / 6);  // fewer samples for speed
        gen->setMonitoredStage(diag_fixed_ptr);

        auto ik = std::make_unique<mtc::stages::ComputeIK>("diag IK", std::move(gen));
        ik->setMaxIKSolutions(4);
        ik->setMinSolutionDistance(0.1);
        // Try with ZERO grasp frame offset first — just test if basic IK works
        Eigen::Isometry3d zero_frame = Eigen::Isometry3d::Identity();
        ik->setIKFrame(zero_frame, RIGHT_FRAME);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        pick->insert(std::move(ik));
      }

      diag.add(std::move(pick));
    }

    try { diag.init(); } catch (const mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER, "DIAG init failed: " << e);
      return;
    }
    auto diag_ret = diag.plan(3);
    const bool diag_ok = static_cast<bool>(diag_ret) || !diag.solutions().empty();
    if (diag_ok) {
      RCLCPP_WARN(LOGGER,
        "DIAG pick-only task SUCCEEDED (%zu solutions). "
        "The failure is in the full task Connect or handover container, "
        "not in the basic IK. The grasp_frame offset or handover geometry is the issue.",
        diag.solutions().size());
    } else {
      RCLCPP_ERROR(LOGGER,
        "DIAG pick-only task also FAILED. ComputeIK has no solutions for this eef/group config. "
        "Check: (1) eef group '%s' exists in SRDF, (2) kinematics plugin is configured, "
        "(3) box position (x=0.5, y=0.2, z=0.06) is reachable by right arm from 'ready'.",
        RIGHT_EEF);
    }
    return;
  }

  RCLCPP_INFO(LOGGER, "Planning succeeded (%zu solutions)", task_.solutions().size());
  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(LOGGER, "Solution published to RViz. Executing...");

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Execution failed (code %d)", result.val);
    return;
  }
  RCLCPP_INFO(LOGGER, "Handover task complete.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });
  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();
  RCLCPP_INFO(LOGGER, "Node alive for RViz inspection. Ctrl-C to exit.");
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
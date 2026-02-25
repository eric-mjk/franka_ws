// MTC_handover/src/mtc.cpp
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MTC_handover");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("mtc_handover_node", options) }
  {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  void setupPlanningScene()
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";

    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // CYLINDER dims: [height, radius]
    object.primitives[0].dimensions = { 0.10, 0.02 };

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.50;
    pose.position.y = 0.00;
    pose.position.z = 0.05;
    pose.orientation.w = 1.0;

    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);

    RCLCPP_INFO(LOGGER, "PlanningScene: spawned object");
  }

  void doTask()
  {
    task_ = createTask();

    try {
      task_.init();
    } catch (mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      return;
    }

    if (!task_.plan(5)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed: " << result.val);
      return;
    }

    RCLCPP_INFO(LOGGER, "Task done");
  }

private:
  mtc::Task createTask()
  {
    // ============================
    // ✅ 너 dual_arm SRDF에 맞춰 여기만 수정
    // ============================
    const std::string RIGHT_ARM  = "right_fr3_arm";
    const std::string RIGHT_HAND = "right_fr3_hand";
    const std::string RIGHT_TCP  = "right_fr3_hand_tcp";   // 또는 right_panda_hand 같은 실제 TCP

    const std::string LEFT_ARM   = "left_fr3_arm";
    const std::string LEFT_HAND  = "left_fr3_hand";
    const std::string LEFT_TCP   = "left_fr3_hand_tcp";
    const std::string WORLD      = "world";
    const std::string OBJECT_ID  = "object";

    mtc::Task task;
    task.stages()->setName("handover_task");
    task.loadRobotModel(node_);

    // Planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(0.01);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;
#pragma GCC diagnostic pop

    // 0) Current State
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // 1) Open both hands (named target "open" 필요)
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("open right hand", interpolation_planner);
      s->setGroup(RIGHT_HAND);
      s->setGoal("open");
      task.add(std::move(s));
    }
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("open left hand", interpolation_planner);
      s->setGroup(LEFT_HAND);
      s->setGoal("open");
      task.add(std::move(s));
    }

    // 2) Move right arm to pick (Connect)
    {
      auto s = std::make_unique<mtc::stages::Connect>(
        "move right to pick",
        mtc::stages::Connect::GroupPlannerVector{ { RIGHT_ARM, sampling_planner } }
      );
      s->setTimeout(8.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    mtc::Stage* attach_right_stage = nullptr;  // left grasp 모니터링용

    // ============================
    // [CONTAINER] RIGHT PICK
    // ============================
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick with right");
      // container에 right-specific properties
      pick->setProperty("group", RIGHT_ARM);
      pick->setProperty("eef", RIGHT_HAND);
      pick->setProperty("ik_frame", RIGHT_TCP);

      // approach object (right)
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("right approach", cartesian_planner);
        s->properties().set("marker_ns", "right_approach");
        s->properties().set("link", RIGHT_TCP);
        s->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        s->setMinMaxDistance(0.08, 0.12);

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = RIGHT_TCP;
        vec.vector.z = 1.0; // TCP 기준 +z 방향(너 tcp 정의에 따라 바꿀 수도)
        s->setDirection(vec);
        pick->insert(std::move(s));
      }

      // generate grasp pose (right) + IK
      {
        auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("right generate grasp");
        gen->properties().configureInitFrom(mtc::Stage::PARENT);
        gen->properties().set("marker_ns", "right_grasp_pose");
        gen->setPreGraspPose("open");
        gen->setObject(OBJECT_ID);
        gen->setAngleDelta(M_PI / 12.0);
        gen->setMonitoredStage(current_state_ptr);

        Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
        grasp_frame_transform.translation().z() = 0.10;

        auto ik = std::make_unique<mtc::stages::ComputeIK>("right grasp IK", std::move(gen));
        ik->setMaxIKSolutions(8);
        ik->setMinSolutionDistance(1.0);
        ik->setIKFrame(grasp_frame_transform, RIGHT_TCP);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        pick->insert(std::move(ik));
      }

      // allow collision (right hand, object)
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (right,object)");
        s->allowCollisions(
          OBJECT_ID,
          task.getRobotModel()->getJointModelGroup(RIGHT_HAND)->getLinkModelNamesWithCollisionGeometry(),
          true
        );
        pick->insert(std::move(s));
      }

      // close right hand
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("close right hand", interpolation_planner);
        s->setGroup(RIGHT_HAND);
        s->setGoal("close");
        pick->insert(std::move(s));
      }

      // attach to right
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object to right");
        s->attachObject(OBJECT_ID, RIGHT_TCP);
        attach_right_stage = s.get();
        pick->insert(std::move(s));
      }

      // lift with right
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("right lift", cartesian_planner);
        s->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        s->setMinMaxDistance(0.08, 0.20);
        s->setIKFrame(RIGHT_TCP);
        s->properties().set("marker_ns", "right_lift");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = WORLD;
        vec.vector.z = 1.0;
        s->setDirection(vec);
        pick->insert(std::move(s));
      }

      task.add(std::move(pick));
    }

    // 3) Connect: left arm approach (right arm holds)
    {
      auto s = std::make_unique<mtc::stages::Connect>(
        "move left to handover",
        mtc::stages::Connect::GroupPlannerVector{ { LEFT_ARM, sampling_planner } }
      );
      s->setTimeout(8.0);
      s->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(s));
    }

    // ============================
    // [CONTAINER] LEFT RECEIVE (handover)
    // - left가 object를 잡고 attach
    // - right에서 detach
    // ============================
    {
      auto handover = std::make_unique<mtc::SerialContainer>("handover to left");
      handover->setProperty("group", LEFT_ARM);
      handover->setProperty("eef", LEFT_HAND);
      handover->setProperty("ik_frame", LEFT_TCP);

      // left approach (optional)
      {
        auto s = std::make_unique<mtc::stages::MoveRelative>("left approach", cartesian_planner);
        s->properties().set("marker_ns", "left_approach");
        s->properties().set("link", LEFT_TCP);
        s->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        s->setMinMaxDistance(0.06, 0.10);

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = LEFT_TCP;
        vec.vector.z = 1.0;
        s->setDirection(vec);
        handover->insert(std::move(s));
      }

      // left generate grasp around CURRENT object (attached to right, so monitoredStage 중요)
      {
        auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("left generate grasp");
        gen->properties().configureInitFrom(mtc::Stage::PARENT);
        gen->properties().set("marker_ns", "left_grasp_pose");
        gen->setPreGraspPose("open");
        gen->setObject(OBJECT_ID);
        gen->setAngleDelta(M_PI / 12.0);

        // ✅ object가 right에 attach된 이후 상태를 보도록
        if (attach_right_stage) gen->setMonitoredStage(attach_right_stage);

        Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
        grasp_frame_transform.translation().z() = 0.08;

        auto ik = std::make_unique<mtc::stages::ComputeIK>("left grasp IK", std::move(gen));
        ik->setMaxIKSolutions(8);
        ik->setMinSolutionDistance(1.0);
        ik->setIKFrame(grasp_frame_transform, LEFT_TCP);
        ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        handover->insert(std::move(ik));
      }

      // allow collision (left hand, object)
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (left,object)");
        s->allowCollisions(
          OBJECT_ID,
          task.getRobotModel()->getJointModelGroup(LEFT_HAND)->getLinkModelNamesWithCollisionGeometry(),
          true
        );
        handover->insert(std::move(s));
      }

      // close left hand
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("close left hand", interpolation_planner);
        s->setGroup(LEFT_HAND);
        s->setGoal("close");
        handover->insert(std::move(s));
      }
      
      // ✅ detach from right first
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object from right");
        s->detachObject(OBJECT_ID, RIGHT_TCP);
        handover->insert(std::move(s));
      }
      
      // ✅ then attach to left
      {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object to left");
        s->attachObject(OBJECT_ID, LEFT_TCP);
        handover->insert(std::move(s));
      }

      // open right hand after transfer (right gripper stage는 container 밖에 둬도 되지만 여기 넣어도 됨)
      {
        auto s = std::make_unique<mtc::stages::MoveTo>("open right hand (release)", interpolation_planner);
        s->setGroup(RIGHT_HAND);
        s->setGoal("open");
        handover->insert(std::move(s));
      }

      task.add(std::move(handover));
    }

    // 4) (선택) 왼팔이 조금 retreat
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("left retreat", cartesian_planner);
      s->properties().set("marker_ns", "left_retreat");
      s->properties().set("link", LEFT_TCP);
      s->properties().set("group", LEFT_ARM);
      s->setMinMaxDistance(0.06, 0.12);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = WORLD;
      vec.vector.x = -0.2;
      s->setDirection(vec);
      task.add(std::move(s));
    }

    return task;
  }

private:
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char** argv)
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

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy  # provided by moveit_py


def plan_and_execute(robot, planning_component, logger):
    """Plan and execute using moveit_py."""
    logger.info("Planning...")
    plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed")
        return False

    logger.info("Executing...")
    robot_trajectory = plan_result.trajectory
    robot.execute(robot_trajectory, controllers=[])
    logger.info("Done.")
    return True


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("fr3_pose_goal")

    # 1) Connect to MoveIt through MoveItPy
    robot = MoveItPy(node_name="moveit_py")

    # 2) IMPORTANT: set this to the Planning Group you see in RViz (MotionPlanning panel)
    PLANNING_GROUP = "fr3_arm"   # <-- if yours differs, change this

    arm = robot.get_planning_component(PLANNING_GROUP)
    logger.info(f"MoveItPy connected. Planning group = {PLANNING_GROUP}")

    # 3) Start from current robot state
    arm.set_start_state_to_current_state()

    # 4) Pose goal (6DOF)
    # IMPORTANT:
    # - frame_id should match RViz "Planning Frame"
    # - pose_link should match RViz "End Effector Link"
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "fr3_link0"   # <-- change if RViz shows a different planning frame

    # Position (meters)
    pose_goal.pose.position.x = 0.40
    pose_goal.pose.position.y = 0.00
    pose_goal.pose.position.z = 0.40

    # Orientation (quaternion). This is "no rotation" relative to the planning frame.
    pose_goal.pose.orientation.w = 1.0

    EE_LINK = "franka_hand"      # <-- change if RViz shows a different end-effector link

    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=EE_LINK)

    # 5) Plan + execute
    plan_and_execute(robot, arm, logger)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
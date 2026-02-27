# MTC_handover/launch/mtc.launch.py
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, rel_path):
    pkg = get_package_share_directory(package_name)
    path = os.path.join(pkg, rel_path)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    dual_arm_share = get_package_share_directory("dual_arm")
    this_share = get_package_share_directory("MTC_handover")

    # 1) include your working bringup (controllers + move_group + rviz)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_arm_share, "launch", "refactor_moveit.launch.py")
        )
    )

    # 2) build same robot_description + semantic for THIS node
    urdf_xacro = os.path.join(dual_arm_share, "config", "dual_fr3.urdf.xacro")
    srdf_xacro = os.path.join(dual_arm_share, "config", "dual_fr3.srdf.xacro")

    robot_description_config = Command([
        FindExecutable(name="xacro"), " ", urdf_xacro,
        " hand:=true",
        " ee_id:=franka_hand",
        " left_robot_ip:=dont-care",
        " right_robot_ip:=dont-care",
        " use_fake_hardware:=true",
        " fake_sensor_commands:=true",
        " ros2_control:=true",
        " left_xyz:=\"0 -0.3 0\"",
        " left_rpy:=\"0 0 0\"",
        " right_xyz:=\"0 0.3 0\"",
        " right_rpy:=\"0 0 0\"",
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    robot_description_semantic_config = Command([
        FindExecutable(name="xacro"), " ", srdf_xacro,
        " hand:=true",
        " ee_id:=franka_hand",
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml("dual_arm", "config/dual_kinematics.yaml") or {}
    ompl_yaml = load_yaml("dual_arm", "config/ompl_planning.yaml") or {}

    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/ResolveConstraintFrames "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    planning_pipelines["ompl"].update(ompl_yaml)

    # 3) run mtc node
    mtc_node = Node(
        package="MTC_handover",
        executable="mtc",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipelines,
        ],
    )

    return LaunchDescription([bringup, mtc_node])
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.conditions import UnlessCondition, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    # -----------------------------
    # Launch args
    # -----------------------------
    left_robot_ip_name = "left_robot_ip"
    right_robot_ip_name = "right_robot_ip"
    use_fake_hw_name = "use_fake_hardware"
    fake_sensor_cmds_name = "fake_sensor_commands"
    load_gripper_name = "load_gripper"
    ee_id_name = "ee_id"
    left_xyz_name = "left_xyz"
    left_rpy_name = "left_rpy"
    right_xyz_name = "right_xyz"
    right_rpy_name = "right_rpy"

    left_robot_ip = LaunchConfiguration(left_robot_ip_name)
    right_robot_ip = LaunchConfiguration(right_robot_ip_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hw_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_cmds_name)
    load_gripper = LaunchConfiguration(load_gripper_name)
    ee_id = LaunchConfiguration(ee_id_name)
    left_xyz = LaunchConfiguration(left_xyz_name)
    left_rpy = LaunchConfiguration(left_rpy_name)
    right_xyz = LaunchConfiguration(right_xyz_name)
    right_rpy = LaunchConfiguration(right_rpy_name)

    # -----------------------------
    # URDF / SRDF (xacro -> string)
    # -----------------------------
    dual_arm_share = get_package_share_directory("dual_arm")

    urdf_xacro = os.path.join(dual_arm_share, "config", "dual_fr3.urdf.xacro")
    robot_description_config = Command([
        FindExecutable(name="xacro"), " ", urdf_xacro,
        " hand:=", load_gripper,
        " ee_id:=", ee_id,
        " left_robot_ip:=", left_robot_ip,
        " right_robot_ip:=", right_robot_ip,
        " use_fake_hardware:=", use_fake_hardware,
        " fake_sensor_commands:=", fake_sensor_commands,
        " ros2_control:=true",
        " left_xyz:=\"", left_xyz, "\"",
        " left_rpy:=\"", left_rpy, "\"",
        " right_xyz:=\"", right_xyz, "\"",
        " right_rpy:=\"", right_rpy, "\"",
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    # ⚠️ SRDF는 “robot name” 포함된 wrapper 파일을 쓰는 게 안전
    # (너가 만든 dual_fr3.srdf.xacro를 추천. franka_arm_dual.srdf.xacro를 바로 쓰면
    #  SRDFDOM에서 "No name given for the robot" 경고가 자주 남)
    srdf_xacro = os.path.join(dual_arm_share, "config", "dual_fr3.srdf.xacro")
    robot_description_semantic_config = Command([
        FindExecutable(name="xacro"), " ", srdf_xacro,
        " hand:=", load_gripper,
        " ee_id:=", ee_id,
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)
    }

    # -----------------------------
    # MoveIt “demo-style” config dict
    # -----------------------------
    # kinematics
    kinematics_yaml = load_yaml("dual_arm", "config/dual_kinematics.yaml") or {}

    # planning pipelines (demo style keys)
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
    # ompl_planning.yaml 내용은 보통 planner_configs 등이므로 'ompl' 아래에 merge
    planning_pipelines["ompl"].update(ompl_yaml)

    # controllers (MoveItSimpleControllerManager)
    moveit_simple_controllers_yaml = load_yaml("dual_arm", "config/dual_fr3_controllers.yaml") or {}
    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # demo처럼 “한 덩어리 dict”로 묶어 move_group에 넣기
    moveit_config_dict = {}
    moveit_config_dict.update(robot_description)
    moveit_config_dict.update(robot_description_semantic)
    moveit_config_dict.update(kinematics_yaml)
    moveit_config_dict.update(planning_pipelines)
    moveit_config_dict.update(moveit_controllers)
    moveit_config_dict.update(trajectory_execution)
    moveit_config_dict.update(planning_scene_monitor)

    # -----------------------------
    # Nodes
    # -----------------------------
    # move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    # RViz (demo와 동일하게 핵심 4개만)
    rviz_config = os.path.join(dual_arm_share, "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipelines,   # ✅ demo처럼 planning_pipelines dict
            kinematics_yaml,
        ],
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control: demo처럼 controllers yaml만 주고, robot_description은 토픽에서 받게 remap
    ros2_controllers_path = os.path.join(dual_arm_share, "config", "dual_ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
            ("joint_states", "arm/joint_states"),  # ros2_control의 joint_states를 /arm/joint_states로
        ],
        on_exit=Shutdown(),
    )

    # controllers spawner (demo 스타일)
    load_controllers = []
    for controller in ["joint_state_broadcaster", "left_arm_controller", "right_arm_controller"]:
        load_controllers.append(
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner", controller,
                     "--controller-manager-timeout", "60",
                     "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )

    # joint state aggregator: /arm/joint_states + gripper joint_states -> /joint_states
    joint_state_aggregator = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_aggregator",
        output="screen",
        parameters=[robot_description, {
            "source_list": [
                "/arm/joint_states",
                "/left/joint_states",
                "/right/joint_states",
            ],
            "rate": 50.0,
        }],
    )

    # Franka robot state broadcaster (실기/리얼 모드에서만)
    franka_robot_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "franka_robot_state_broadcaster",
            "--controller-manager-timeout", "60",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    # gripper bringup (옵션)
    left_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("franka_gripper"), "launch", "gripper.launch.py"])
        ),
        launch_arguments={
            "robot_ip": left_robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "namespace": "left",
            "robot_type": "left_fr3",
        }.items(),
        condition=IfCondition(load_gripper),
    )

    right_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("franka_gripper"), "launch", "gripper.launch.py"])
        ),
        launch_arguments={
            "robot_ip": right_robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "namespace": "right",
            "robot_type": "right_fr3",
        }.items(),
        condition=IfCondition(load_gripper),
    )

    # -----------------------------
    # Launch arguments
    # -----------------------------
    launch_args = [
        DeclareLaunchArgument(left_robot_ip_name, default_value="dont-care"),
        DeclareLaunchArgument(right_robot_ip_name, default_value="dont-care"),
        DeclareLaunchArgument(load_gripper_name, default_value="true"),
        DeclareLaunchArgument(ee_id_name, default_value="franka_hand"),
        DeclareLaunchArgument(use_fake_hw_name, default_value="true"),
        DeclareLaunchArgument(fake_sensor_cmds_name, default_value="true"),
        DeclareLaunchArgument(left_xyz_name, default_value="0 0 0"),
        DeclareLaunchArgument(left_rpy_name, default_value="0 0 0"),
        DeclareLaunchArgument(right_xyz_name, default_value="0.8 0 0"),
        DeclareLaunchArgument(right_rpy_name, default_value="0 0 0"),
    ]

    return LaunchDescription(
        launch_args + [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_aggregator,
            franka_robot_state_broadcaster,
            left_gripper_launch,
            right_gripper_launch,
        ] + load_controllers
    )
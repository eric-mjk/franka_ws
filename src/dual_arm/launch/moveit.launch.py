import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown
)
from launch.conditions import UnlessCondition,IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    left_robot_ip_parameter_name = 'left_robot_ip'
    right_robot_ip_parameter_name = 'right_robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    load_gripper_parameter_name = 'load_gripper'
    ee_id_parameter_name = 'ee_id'
    left_xyz_parameter_name = 'left_xyz'
    left_rpy_parameter_name = 'left_rpy'
    right_xyz_parameter_name = 'right_xyz'
    right_rpy_parameter_name = 'right_rpy'

    left_robot_ip = LaunchConfiguration(left_robot_ip_parameter_name)
    right_robot_ip = LaunchConfiguration(right_robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    namespace = ''  # default to global namespace
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    ee_id = LaunchConfiguration(ee_id_parameter_name)
    left_xyz = LaunchConfiguration(left_xyz_parameter_name)
    left_rpy = LaunchConfiguration(left_rpy_parameter_name)
    right_xyz = LaunchConfiguration(right_xyz_parameter_name)
    right_rpy = LaunchConfiguration(right_rpy_parameter_name)


    ### URDF ###
    franka_xacro_file = os.path.join(
        get_package_share_directory('dual_arm'),
        'config', 'dual_fr3.urdf.xacro'
    )

    robot_description_config = Command(
        [
            FindExecutable(name='xacro'), ' ', franka_xacro_file,
            ' hand:=', load_gripper,
            ' ee_id:=', ee_id,
            ' left_robot_ip:=', left_robot_ip,
            ' right_robot_ip:=', right_robot_ip,
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
            ' ros2_control:=true',

            ' left_xyz:="', left_xyz, '"',
            ' left_rpy:="', left_rpy, '"',
            ' right_xyz:="', right_xyz, '"',
            ' right_rpy:="', right_rpy, '"',
        ]
    )

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}



    ### SRDF ###
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('dual_arm'),
        'config', 'franka_arm_dual.srdf.xacro'
    )

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         franka_semantic_xacro_file, ' hand:=', load_gripper, ' ee_id:=', ee_id]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}



    ### Kinematics ###
    kinematics_yaml = load_yaml(
        'dual_arm', 'config/dual_kinematics.yaml'
    )



    ## OMPL Planning Pipeline ###
    # Keep this code. Extra functionality is added to config yaml
    # Planning Functionality

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'dual_arm', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)



    ### Controllers ###
    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'dual_arm', 'config/dual_fr3_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }




    ### Nodes ###
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'dual_arm'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('dual_arm'),
        'config',
        'dual_ros2_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
        remappings=[('joint_states', 'arm/joint_states')],  # 결과: /arm/joint_states
    )

    load_controllers = []
    for controller in ['joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            )
        )

    joint_state_aggregator = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_aggregator',
        output='screen',
        parameters=[robot_description, {
            'source_list': [
                '/arm/joint_states',        # arm from ros2_control
                '/left/joint_states',   # left gripper
                '/right/joint_states',  # right gripper
            ],
            'rate': 50.0,
        }],
        # gripper를 안 띄우면 굳이 aggregator 필요 없으니 조건 걸어도 되고(선택)
        # condition=IfCondition(load_gripper),
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     namespace=namespace,
    #     parameters=[
    #         {'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'], 'rate': 30}],
    # )

    franka_robot_state_broadcaster_spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'franka_robot_state_broadcaster',
            '--controller-manager-timeout', '60',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    # Edit Confirmed
    left_robot_arg = DeclareLaunchArgument(
        left_robot_ip_parameter_name,
        default_value='dont-care',
        description='Hostname or IP address of the LEFT robot.'
    )

    right_robot_arg = DeclareLaunchArgument(
        right_robot_ip_parameter_name,
        default_value='dont-care',
        description='Hostname or IP address of the RIGHT robot.'
    )

    load_gripper_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value='true',
        description='Whether to load the gripper or not (true or false)'
    )
    ee_id_arg = DeclareLaunchArgument(
        ee_id_parameter_name,
        default_value='franka_hand',
        description='The end-effector id to use. Available options: none, franka_hand, cobot_pump'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='true',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='true',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    left_xyz_arg = DeclareLaunchArgument(left_xyz_parameter_name, default_value='0 0 0')
    left_rpy_arg = DeclareLaunchArgument(left_rpy_parameter_name, default_value='0 0 0')
    right_xyz_arg = DeclareLaunchArgument(right_xyz_parameter_name, default_value='0.8 0 0')
    right_rpy_arg = DeclareLaunchArgument(right_rpy_parameter_name, default_value='0 0 0')




    # Edit Confirmed
    left_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_gripper'),
                'launch',
                'gripper.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip': left_robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'namespace': 'left',
            'robot_type': 'left_fr3',   # ← 여기 추가
        }.items(),
        condition = IfCondition(load_gripper),
    )
    right_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_gripper'),
                'launch',
                'gripper.launch.py'
            ])
        ),
        launch_arguments={
            'robot_ip': right_robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'namespace': 'right',
            'robot_type': 'right_fr3',   # ← 여기 추가
        }.items(),
        condition = IfCondition(load_gripper),
    )


    return LaunchDescription(
        [
            left_robot_arg,
            right_robot_arg,
            load_gripper_arg,
            ee_id_arg,
            use_fake_hardware_arg,
            fake_sensor_commands_arg,

            # ✅ 반드시 "사용되기 전에" 선언돼야 함
            left_xyz_arg,
            left_rpy_arg,
            right_xyz_arg,
            right_rpy_arg,

            rviz_node,
            joint_state_aggregator,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            franka_robot_state_broadcaster_spawn,
            left_gripper_launch,
            right_gripper_launch,
            #  joint_state_publisher,

        ]
        + load_controllers
    )
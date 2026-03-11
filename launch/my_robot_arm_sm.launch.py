from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_launch = os.path.join(
        get_package_share_directory("my_jearm_moveit_config"),
        "launch",
        "moveit_rviz.launch.py",
    )

    # Load MoveIt configuration files
    moveit_config_share = get_package_share_directory("my_jearm_moveit_config")
    
    urdf_file = os.path.join(moveit_config_share, 'config', 'L_JEARM.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    srdf_file = os.path.join(moveit_config_share, 'config', 'L_JEARM.srdf.xacro')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()
    
    kinematics_yaml = load_yaml(os.path.join(moveit_config_share, 'config', 'kinematics.yaml'))
    joint_limits_yaml = load_yaml(os.path.join(moveit_config_share, 'config', 'joint_limits.yaml'))
    planning_pipelines_yaml = load_yaml(os.path.join(moveit_config_share, 'config', 'planning_pipelines.yaml'))
    
    # MoveIt configuration parameters for state machine
    moveit_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
    }
    
    # 加载规划配置（包含 pipeline 和 OMPL 细节）
    if planning_pipelines_yaml:
        moveit_params.update(planning_pipelines_yaml)
    
    # 强制指定使用OMPL规划器
    moveit_params['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    
    # 确保轨迹执行超时参数被正确加载
    # 允许执行时间为轨迹声明时间的100倍
    moveit_params['trajectory_execution'] = {
        'manage_controllers': True,
        'allowed_execution_timeout_scaling': 100.0,
        'execution_duration_monitoring': False  # Disable strict monitoring
    }

    # Fake Trajectory Executor - 接收MoveIt的轨迹执行命令
    fake_executor = Node(
        package="my_robot_arm_sm",
        executable="fake_trajectory_executor",
        output="screen",
    )

    # 状态机节点 - 延迟8秒启动，给move_group和controller时间初始化
    sm_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="my_robot_arm_sm",
                executable="my_robot_arm_sm_node",
                output="screen",
                parameters=[moveit_params],
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=["/usr/lib/x86_64-linux-gnu:", EnvironmentVariable("LD_LIBRARY_PATH", default_value="")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
            ),
            fake_executor,
            sm_node,
        ]
    )


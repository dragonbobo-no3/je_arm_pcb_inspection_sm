from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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
    runtime_env = {
        "PYTHONPATH": "",
        "PYTHONHOME": "",
        "CONDA_PREFIX": "",
        "CONDA_DEFAULT_ENV": "",
        "CONDA_PROMPT_MODIFIER": "",
        "CONDA_SHLVL": "",
        "LD_PRELOAD": "",
        "TZ": "Asia/Shenzhen",
        "RCUTILS_CONSOLE_OUTPUT_FORMAT": "[{severity}] [{name}]: {message}",
        "ROS_DOMAIN_ID": "199",  # Match clean_restart.sh domain ID
        "PATH": ["/usr/bin:/bin:/usr/sbin:/sbin:", EnvironmentVariable("PATH", default_value="")],
        "LD_LIBRARY_PATH": [
            "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    }

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

    moveit_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
    }

    if planning_pipelines_yaml:
        moveit_params.update(planning_pipelines_yaml)

    moveit_params['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    moveit_params['trajectory_execution'] = {
        'manage_controllers': True,
        'allowed_execution_timeout_scaling': 100.0,
        'execution_duration_monitoring': False,
        'allowed_goal_duration_margin': 5.0,
        'action_monitor_rate': 10.0,
    }
    
    # Increase start state tolerance to handle position drift between planning and execution
    moveit_params['start_state_max_bounds_error'] = 0.05  # ← 从 0.01 增加到 0.05
    
    # Allow approximate solutions to handle minor state divergence between planning and execution
    moveit_params['allow_approximate_solution'] = True

    keyboard_server = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-lc",
            (
                "unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL && "
                "export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH && "
                "export ROS_DOMAIN_ID=199 && "
                "source /opt/ros/humble/setup.bash && "
                "source /home/test/ros2_ws/install/setup.bash && "
                "echo 'SMACC2 keyboard ready: s(start), n(next step), w(loop), p(pause), r(resume), b(back), f(fault), u(resource-unavailable)' && "
                "/usr/bin/python3 /home/test/ros2_ws/install/cl_keyboard/lib/cl_keyboard/keyboard_server_node.py; "
                "echo; echo 'keyboard_server_node exited'; "
                "read -r -n 1 -s -p 'Press any key to close...'"
            ),
        ],
        output="screen",
        shell=False,
    )

    sm_node = TimerAction(
        period=5.0,  # Short delay since move_group is already running
        actions=[
            Node(
                package="je_arm_pcb_inspection_sm",
                executable="je_arm_pcb_inspection_sm_node",
                output="screen",
                parameters=[moveit_params],
                arguments=[
                    "--ros-args",
                    "--log-level", "warn",
                    "--log-level", "SmJeArmPcbInspection:=error",
                    "--log-level", "move_group_interface:=warn",
                    "--log-level", "je_arm_pcb_inspection_sm.biz:=info",
                ],
                additional_env=runtime_env,
            )
        ],
    )

    return LaunchDescription(
        [
            keyboard_server,
            sm_node,
        ]
    )

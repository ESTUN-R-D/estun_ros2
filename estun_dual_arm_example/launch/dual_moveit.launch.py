import os
from copy import deepcopy

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_DUAL_ARM_MODEL = "iER7-910-MI"


def load_text(path):
    with open(path, "r") as file:
        return file.read()


def load_yaml(path):
    try:
        with open(path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def resolve_model_kinematics_path(desc_share, robot_model):
    return os.path.join(desc_share, "config", robot_model, "default_kinematics.yaml")


def rewrite_prefixed_text(text, rarm_prefix, larm_prefix):
    return (
        text.replace("rarm_", "__ESTUN_RARM_PREFIX__")
        .replace("larm_", "__ESTUN_LARM_PREFIX__")
        .replace("__ESTUN_RARM_PREFIX__", rarm_prefix)
        .replace("__ESTUN_LARM_PREFIX__", larm_prefix)
    )


def with_prefixed_joint_limits(joint_limits_yaml, prefix):
    if not isinstance(joint_limits_yaml, dict):
        raise RuntimeError("joint_limits.yaml 缺少根映射")
    rewritten = deepcopy(joint_limits_yaml)
    joint_limits = rewritten.get("joint_limits")
    if not isinstance(joint_limits, dict):
        raise RuntimeError("joint_limits.yaml 缺少 joint_limits 映射")

    rewritten["joint_limits"] = {
        f"{prefix}{joint_name}": limit_cfg for joint_name, limit_cfg in joint_limits.items()
    }
    return rewritten


def merge_dual_arm_joint_limits(
    rarm_joint_limits_yaml,
    larm_joint_limits_yaml,
    rarm_prefix,
    larm_prefix,
):
    rarm_prefixed = with_prefixed_joint_limits(rarm_joint_limits_yaml, rarm_prefix)
    larm_prefixed = with_prefixed_joint_limits(larm_joint_limits_yaml, larm_prefix)

    merged = {}
    velocity_scalings = [
        value
        for value in (
            rarm_prefixed.get("default_velocity_scaling_factor"),
            larm_prefixed.get("default_velocity_scaling_factor"),
        )
        if isinstance(value, (int, float))
    ]
    acceleration_scalings = [
        value
        for value in (
            rarm_prefixed.get("default_acceleration_scaling_factor"),
            larm_prefixed.get("default_acceleration_scaling_factor"),
        )
        if isinstance(value, (int, float))
    ]
    if velocity_scalings:
        merged["default_velocity_scaling_factor"] = min(velocity_scalings)
    if acceleration_scalings:
        merged["default_acceleration_scaling_factor"] = min(acceleration_scalings)

    merged["joint_limits"] = {
        **rarm_prefixed["joint_limits"],
        **larm_prefixed["joint_limits"],
    }
    return merged


def build_joint_states_topic(namespace):
    normalized = str(namespace).strip().strip("/")
    if not normalized:
        return "/joint_states"
    return f"/{normalized}/joint_states"


def parse_bool(raw_value, param_name):
    value = str(raw_value).strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"{param_name} 非法: '{raw_value}'，必须为 true/false")


def launch_setup(context, *args, **kwargs):
    dual_share = get_package_share_directory("estun_dual_arm_example")
    desc_share = get_package_share_directory("estun_description")

    rarm_namespace = LaunchConfiguration("rarm_namespace").perform(context)
    larm_namespace = LaunchConfiguration("larm_namespace").perform(context)
    rarm_prefix = LaunchConfiguration("rarm_prefix").perform(context)
    larm_prefix = LaunchConfiguration("larm_prefix").perform(context)
    rarm_model = LaunchConfiguration("rarm_model").perform(context)
    larm_model = LaunchConfiguration("larm_model").perform(context)
    start_control = parse_bool(
        LaunchConfiguration("start_control").perform(context), "start_control"
    )
    start_rviz = parse_bool(
        LaunchConfiguration("start_rviz").perform(context), "start_rviz"
    )

    urdf_file = os.path.join(dual_share, "urdf", "dual_arm.urdf.xacro")
    rarm_kinematics_file = resolve_model_kinematics_path(desc_share, rarm_model)
    larm_kinematics_file = resolve_model_kinematics_path(desc_share, larm_model)
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                urdf_file,
                " rarm_prefix:=",
                rarm_prefix,
                " larm_prefix:=",
                larm_prefix,
                " rarm_model:=",
                rarm_model,
                " larm_model:=",
                larm_model,
                " rarm_kinematics_file:=",
                rarm_kinematics_file,
                " larm_kinematics_file:=",
                larm_kinematics_file,
            ]
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": rewrite_prefixed_text(
            load_text(os.path.join(dual_share, "srdf", "dual_arm.srdf")),
            rarm_prefix,
            larm_prefix,
        )
    }
    kinematics_yaml = load_yaml(os.path.join(dual_share, "config", "kinematics.yaml"))
    joint_limits_yaml = merge_dual_arm_joint_limits(
        load_yaml(os.path.join(desc_share, "config", rarm_model, "joint_limits.yaml")),
        load_yaml(os.path.join(desc_share, "config", larm_model, "joint_limits.yaml")),
        rarm_prefix,
        larm_prefix,
    )
    ompl_yaml = load_yaml(os.path.join(dual_share, "config", "ompl_planning.yaml"))
    moveit_controllers_yaml = load_yaml(
        os.path.join(dual_share, "config", "moveit_controllers.yaml")
    )
    if not all(
        (kinematics_yaml, joint_limits_yaml, ompl_yaml, moveit_controllers_yaml)
    ):
        raise RuntimeError("双臂 MoveIt 配置加载失败")

    robot_description_planning = {"robot_description_planning": joint_limits_yaml}
    planning_pipeline_parameters = [
        {"planning_pipelines": {"pipeline_names": ["ompl"]}},
        {"default_planning_pipeline": "ompl"},
        ompl_yaml,
    ]
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    nodes_to_launch = []
    if start_control:
        nodes_to_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(dual_share, "launch", "dual_control.launch.py")
                ),
                launch_arguments={
                    "rarm_namespace": LaunchConfiguration("rarm_namespace"),
                    "larm_namespace": LaunchConfiguration("larm_namespace"),
                    "rarm_prefix": LaunchConfiguration("rarm_prefix"),
                    "larm_prefix": LaunchConfiguration("larm_prefix"),
                    "rarm_model": LaunchConfiguration("rarm_model"),
                    "larm_model": LaunchConfiguration("larm_model"),
                    "rarm_ip": LaunchConfiguration("rarm_ip"),
                    "larm_ip": LaunchConfiguration("larm_ip"),
                    "rarm_cmd_port": LaunchConfiguration("rarm_cmd_port"),
                    "larm_cmd_port": LaunchConfiguration("larm_cmd_port"),
                    "rarm_servo_port": LaunchConfiguration("rarm_servo_port"),
                    "larm_servo_port": LaunchConfiguration("larm_servo_port"),
                    "rarm_status_port": LaunchConfiguration("rarm_status_port"),
                    "larm_status_port": LaunchConfiguration("larm_status_port"),
                    "rarm_use_fake_hardware": LaunchConfiguration("rarm_use_fake_hardware"),
                    "larm_use_fake_hardware": LaunchConfiguration("larm_use_fake_hardware"),
                    "motion_period_ms": LaunchConfiguration("motion_period_ms"),
                    "control_mode": "plan",
                    "control_node_impl": LaunchConfiguration("control_node_impl"),
                    "use_logical_control_time": LaunchConfiguration(
                        "use_logical_control_time"
                    ),
                    "enable_control_loop_diag_log": LaunchConfiguration(
                        "enable_control_loop_diag_log"
                    ),
                    "controllers_file": LaunchConfiguration("controllers_file"),
                }.items(),
            )
        )

    nodes_to_launch.extend(
        [
            Node(
                package="estun_dual_arm_example",
                executable="joint_state_merger.py",
                output="screen",
                parameters=[
                    {
                        "rarm_joint_states_topic": build_joint_states_topic(
                            rarm_namespace
                        ),
                        "larm_joint_states_topic": build_joint_states_topic(
                            larm_namespace
                        ),
                    }
                ],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"robot_description_kinematics": kinematics_yaml},
                    robot_description_planning,
                    moveit_controllers_yaml,
                    planning_scene_monitor_parameters,
                    *planning_pipeline_parameters,
                ],
            ),
        ]
    )

    if start_rviz:
        nodes_to_launch.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_dual_arm",
                output="screen",
                arguments=["-d", os.path.join(dual_share, "rviz", "dual_moveit.rviz")],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"robot_description_kinematics": kinematics_yaml},
                    robot_description_planning,
                    *planning_pipeline_parameters,
                ],
            )
        )

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("rarm_namespace", default_value="rarm", description="右臂 ROS 命名空间"),
        DeclareLaunchArgument("larm_namespace", default_value="larm", description="左臂 ROS 命名空间"),
        DeclareLaunchArgument("rarm_prefix", default_value="rarm_", description="右臂关节/连杆前缀"),
        DeclareLaunchArgument("larm_prefix", default_value="larm_", description="左臂关节/连杆前缀"),
        DeclareLaunchArgument(
            "rarm_model", default_value=DEFAULT_DUAL_ARM_MODEL, description="右臂机型"
        ),
        DeclareLaunchArgument(
            "larm_model", default_value=DEFAULT_DUAL_ARM_MODEL, description="左臂机型"
        ),
        DeclareLaunchArgument("rarm_ip", default_value="", description="右臂 IP"),
        DeclareLaunchArgument("larm_ip", default_value="", description="左臂 IP"),
        DeclareLaunchArgument("rarm_cmd_port", default_value="61210", description="右臂 CMD 端口"),
        DeclareLaunchArgument("larm_cmd_port", default_value="61210", description="左臂 CMD 端口"),
        DeclareLaunchArgument("rarm_servo_port", default_value="61211", description="右臂 SERVO 端口"),
        DeclareLaunchArgument("larm_servo_port", default_value="61211", description="左臂 SERVO 端口"),
        DeclareLaunchArgument(
            "rarm_status_port", default_value="61212", description="右臂 STATUS 端口"
        ),
        DeclareLaunchArgument(
            "larm_status_port", default_value="61212", description="左臂 STATUS 端口"
        ),
        DeclareLaunchArgument(
            "rarm_use_fake_hardware", default_value="true", description="右臂虚拟硬件"
        ),
        DeclareLaunchArgument(
            "larm_use_fake_hardware", default_value="true", description="左臂虚拟硬件"
        ),
        DeclareLaunchArgument("motion_period_ms", default_value="4", description="ERI 运动周期 (ms)"),
        DeclareLaunchArgument(
            "control_node_impl",
            default_value="auto",
            description="控制节点实现: auto|official|estun",
        ),
        DeclareLaunchArgument(
            "use_logical_control_time",
            default_value="true",
            description="estun_control_node 时间源",
        ),
        DeclareLaunchArgument(
            "enable_control_loop_diag_log",
            default_value="false",
            description="是否输出 estun_control_node 周期诊断日志",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/estun_6dof_controllers.yaml",
            description="控制器配置文件",
        ),
        DeclareLaunchArgument("start_control", default_value="true", description="是否启动双臂控制层"),
        DeclareLaunchArgument("start_rviz", default_value="true", description="是否启动 RViz"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

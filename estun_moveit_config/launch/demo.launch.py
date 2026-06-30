import copy
import os
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    full_path = os.path.join(package_path, file_path)
    try:
        with open(full_path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def resolve_control_mode(control_mode_raw):
    mode = control_mode_raw.strip().lower()
    mode_map = {
        "plan": {
            "control_mode": "plan",
            "hardware_servo_mode": "apos",
            "stream_policy": "fifo",
        },
        "apos": {
            "control_mode": "apos",
            "hardware_servo_mode": "apos",
            "stream_policy": "latest_overwrite",
        },
        "cpos": {
            "control_mode": "cpos",
            "hardware_servo_mode": "cpos",
            "stream_policy": "latest_overwrite",
        },
    }
    if mode not in mode_map:
        raise RuntimeError(
            f"无效 control_mode='{control_mode_raw}'，仅支持: plan | apos | cpos"
        )
    return mode_map[mode]


def add_prefix_if_needed(name, prefix):
    if not prefix:
        return name
    if name.startswith(prefix):
        return name
    return f"{prefix}{name}"


def parse_auto_bool(raw_value, default_value, param_name="bool parameter"):
    value = raw_value.strip().lower()
    if value in ("", "auto"):
        return default_value
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(
        f"无效 {param_name}='{raw_value}'，仅支持: auto|true|false"
    )


def normalize_planning_pipeline(raw_value):
    value = raw_value.strip().lower()
    alias_map = {
        "ompl": "ompl",
        "pilz": "pilz_industrial_motion_planner",
        "pilz_industrial_motion_planner": "pilz_industrial_motion_planner",
    }
    if value not in alias_map:
        raise RuntimeError(
            f"无效 planning pipeline='{raw_value}'，仅支持: ompl | pilz"
        )
    return alias_map[value]


def parse_planning_pipelines(raw_value):
    raw_tokens = [token.strip() for token in raw_value.split(",")]
    pipelines = []
    for token in raw_tokens:
        if not token:
            continue
        normalized = normalize_planning_pipeline(token)
        if normalized not in pipelines:
            pipelines.append(normalized)
    if not pipelines:
        raise RuntimeError("planning_pipelines 不能为空，示例: ompl,pilz")
    return pipelines


def with_pilz_cartesian_limits(robot_description_planning_yaml, pilz_cartesian_limits_yaml):
    merged = (
        copy.deepcopy(robot_description_planning_yaml)
        if robot_description_planning_yaml
        else {}
    )
    cartesian_limits = (pilz_cartesian_limits_yaml or {}).get("cartesian_limits")
    if not isinstance(cartesian_limits, dict):
        raise RuntimeError("启用了 Pilz，但未提供有效 cartesian_limits 配置")
    merged["cartesian_limits"] = copy.deepcopy(cartesian_limits)
    return merged


def load_robot_cartesian_limits_yaml(robot_model):
    model_limits_path = os.path.join("config", robot_model, "cartesian_limits.yaml")
    model_limits_yaml = load_yaml("estun_description", model_limits_path)
    if model_limits_yaml:
        return model_limits_yaml
    return load_yaml("estun_moveit_config", "config/pilz_cartesian_limits.yaml")


def resolve_effective_default_pipeline(default_pipeline, pipeline_names, enable_rviz):
    if default_pipeline != "pilz_industrial_motion_planner" or not enable_rviz:
        return default_pipeline
    if "ompl" in pipeline_names:
        print(
            "[WARN] default_planning_pipeline=pilz 在 RViz 默认请求(planner_id 为空)下可能失败，"
            "已自动回退为 ompl。若需 Pilz，请在请求中设置 planner_id=PTP|LIN|CIRC。"
        )
        return "ompl"
    raise RuntimeError(
        "default_planning_pipeline=pilz 且启用了 RViz，但 planning_pipelines 中未包含 ompl。"
        "请添加 ompl 或关闭 RViz，或由上层客户端显式设置 planner_id=PTP|LIN|CIRC。"
    )


def parse_positive_int(raw_value, field_name):
    try:
        value = int(str(raw_value).strip())
    except (TypeError, ValueError):
        raise RuntimeError(f"{field_name} 非法: '{raw_value}'，必须为正整数")
    if value <= 0:
        raise RuntimeError(f"{field_name} 非法: {value}，必须 > 0")
    return value


def parse_non_negative_float(raw_value, field_name):
    try:
        value = float(str(raw_value).strip())
    except (TypeError, ValueError):
        raise RuntimeError(f"{field_name} 非法: '{raw_value}'，必须为非负数")
    if value < 0.0:
        raise RuntimeError(f"{field_name} 非法: {value}，必须 >= 0")
    return value


def validate_robot_ip(robot_ip_raw, use_fake_hardware_raw):
    value = str(robot_ip_raw).strip()
    fake = str(use_fake_hardware_raw).strip().lower() in ("true", "1", "yes", "on")
    if fake:
        return value
    if not value:
        raise RuntimeError(
            "use_fake_hardware=false 时必须显式传入 robot_ip，例如 robot_ip:=<ROBOT_IP>"
        )
    return value


def should_delay_moveit_start(use_fake_hardware, control_node_impl):
    impl = control_node_impl.strip().lower()
    fake = use_fake_hardware.strip().lower() in ("true", "1", "yes", "on")
    selected_impl = "official" if impl == "auto" and fake else impl
    selected_impl = "estun" if impl == "auto" and not fake else selected_impl
    return (not fake) and selected_impl == "estun"


def extract_group_chain_links(srdf_text, group_name):
    if not srdf_text:
        return None, None
    try:
        root = ET.fromstring(srdf_text)
    except ET.ParseError:
        return None, None

    group = root.find(f".//group[@name='{group_name}']")
    if group is not None:
        chain = group.find("chain")
        if chain is not None:
            return chain.get("base_link"), chain.get("tip_link")

    chain = root.find(".//group/chain")
    if chain is None:
        return None, None
    return chain.get("base_link"), chain.get("tip_link")


def with_servo_runtime_overrides(
    raw_yaml,
    move_group_name,
    planning_frame,
    ee_frame,
    command_out_topic,
    publish_period_sec,
):
    if not raw_yaml:
        raise RuntimeError("加载 Servo 配置失败: config/servo.yaml")
    patched = copy.deepcopy(raw_yaml)
    patched["move_group_name"] = move_group_name
    patched["planning_frame"] = planning_frame
    patched["ee_frame_name"] = ee_frame
    patched["robot_link_command_frame"] = ee_frame
    patched["command_out_topic"] = command_out_topic
    patched["publish_period"] = publish_period_sec
    return patched


def with_prefixed_moveit_controllers(raw_yaml, prefix):
    if not raw_yaml:
        return raw_yaml
    patched = copy.deepcopy(raw_yaml)
    manager = patched.get("moveit_simple_controller_manager", {})
    controller_names = manager.get("controller_names", [])
    for controller_name in controller_names:
        controller_cfg = manager.get(controller_name, {})
        joints = controller_cfg.get("joints")
        if isinstance(joints, list):
            controller_cfg["joints"] = [add_prefix_if_needed(j, prefix) for j in joints]
    return patched


def with_prefixed_joint_limits(raw_yaml, prefix):
    if not raw_yaml:
        return raw_yaml
    patched = copy.deepcopy(raw_yaml)
    joint_limits = patched.get("joint_limits")
    if not isinstance(joint_limits, dict):
        return patched

    renamed_limits = {}
    for joint_name, joint_cfg in joint_limits.items():
        renamed_limits[add_prefix_if_needed(joint_name, prefix)] = joint_cfg
    patched["joint_limits"] = renamed_limits
    return patched


def load_srdf_with_prefix(conf_pkg, robot_model, prefix):
    srdf_file = os.path.join(conf_pkg, "config", robot_model, f"{robot_model}.srdf")
    try:
        with open(srdf_file, "r") as file:
            srdf_text = file.read()
    except FileNotFoundError:
        return ""

    if not prefix:
        return srdf_text

    root = ET.fromstring(srdf_text)

    for chain in root.findall(".//chain"):
        base_link = chain.get("base_link")
        tip_link = chain.get("tip_link")
        if base_link:
            chain.set("base_link", add_prefix_if_needed(base_link, prefix))
        if tip_link:
            chain.set("tip_link", add_prefix_if_needed(tip_link, prefix))

    for group_state_joint in root.findall(".//group_state/joint"):
        joint_name = group_state_joint.get("name")
        if joint_name:
            group_state_joint.set("name", add_prefix_if_needed(joint_name, prefix))

    for virtual_joint in root.findall(".//virtual_joint"):
        child_link = virtual_joint.get("child_link")
        if child_link:
            virtual_joint.set("child_link", add_prefix_if_needed(child_link, prefix))

    for disable_collision in root.findall(".//disable_collisions"):
        link1 = disable_collision.get("link1")
        link2 = disable_collision.get("link2")
        if link1:
            disable_collision.set("link1", add_prefix_if_needed(link1, prefix))
        if link2:
            disable_collision.set("link2", add_prefix_if_needed(link2, prefix))

    return ET.tostring(root, encoding="unicode")


def launch_setup(context, *args, **kwargs):
    prefix_val = LaunchConfiguration("prefix").perform(context)
    use_fake_hardware_val = LaunchConfiguration("use_fake_hardware").perform(context)
    robot_ip_val = LaunchConfiguration("robot_ip").perform(context)
    cmd_port_val = LaunchConfiguration("cmd_port").perform(context)
    servo_port_val = LaunchConfiguration("servo_port").perform(context)
    status_port_val = LaunchConfiguration("status_port").perform(context)
    motion_period_ms_val = LaunchConfiguration("motion_period_ms").perform(context)
    control_mode_val = LaunchConfiguration("control_mode").perform(context)
    control_node_impl_val = LaunchConfiguration("control_node_impl").perform(context)
    use_logical_control_time_val = LaunchConfiguration("use_logical_control_time").perform(context)
    enable_control_loop_diag_log_val = LaunchConfiguration(
        "enable_control_loop_diag_log"
    ).perform(context)
    moveit_start_delay_sec_val = LaunchConfiguration(
        "moveit_start_delay_sec"
    ).perform(context)
    start_moveit_val = LaunchConfiguration("start_moveit").perform(context)
    start_rviz_val = LaunchConfiguration("start_rviz").perform(context)
    start_servo_val = LaunchConfiguration("start_servo").perform(context)
    planning_pipelines_val = LaunchConfiguration("planning_pipelines").perform(context)
    default_planning_pipeline_val = LaunchConfiguration(
        "default_planning_pipeline"
    ).perform(context)
    robot_model_val = LaunchConfiguration("model").perform(context)
    controllers_file_val = LaunchConfiguration("controllers_file").perform(context)
    robot_ip_val = validate_robot_ip(robot_ip_val, use_fake_hardware_val)

    mode_cfg = resolve_control_mode(control_mode_val)
    is_plan_mode = mode_cfg["control_mode"] == "plan"
    enable_moveit = parse_auto_bool(start_moveit_val, is_plan_mode)
    enable_rviz = parse_auto_bool(start_rviz_val, is_plan_mode)
    is_apos_mode = mode_cfg["control_mode"] == "apos"
    enable_servo = parse_auto_bool(start_servo_val, is_apos_mode and enable_moveit)
    if enable_servo and not is_apos_mode:
        raise RuntimeError(
            "当前 Servo 输出固定对接 /forward_position_controller/commands，"
            "仅支持 control_mode:=apos。"
        )
    if enable_servo and not enable_moveit:
        print("[WARN] start_servo=true 需要 move_group，已自动启用 start_moveit=true")
        enable_moveit = True
    requested_moveit_start_delay_sec = parse_non_negative_float(
        moveit_start_delay_sec_val,
        "moveit_start_delay_sec",
    )
    moveit_start_delay_sec = (
        requested_moveit_start_delay_sec
        if should_delay_moveit_start(use_fake_hardware_val, control_node_impl_val)
        else 0.0
    )
    planning_pipeline_names = parse_planning_pipelines(planning_pipelines_val)
    default_planning_pipeline = normalize_planning_pipeline(default_planning_pipeline_val)
    if default_planning_pipeline not in planning_pipeline_names:
        raise RuntimeError(
            "default_planning_pipeline 必须出现在 planning_pipelines 中。"
            f" default={default_planning_pipeline}, pipelines={planning_pipeline_names}"
        )
    effective_default_planning_pipeline = resolve_effective_default_pipeline(
        default_pipeline=default_planning_pipeline,
        pipeline_names=planning_pipeline_names,
        enable_rviz=enable_rviz,
    )

    desc_pkg = get_package_share_directory("estun_description")
    conf_pkg = get_package_share_directory("estun_moveit_config")
    estun_hw_share = get_package_share_directory("estun_hardware")
    xacro_file = os.path.join(estun_hw_share, "robot", f"{robot_model_val}.urdf.xacro")

    actual_path = os.path.join(desc_pkg, "config", robot_model_val, "calibration.yaml")
    nominal_path = os.path.join(desc_pkg, "config", robot_model_val, "default_kinematics.yaml")
    if os.path.exists(actual_path):
        target_calib_file = actual_path
    elif os.path.exists(nominal_path):
        target_calib_file = nominal_path
    else:
        target_calib_file = ""

    robot_description_config = Command(
        [
            "xacro ",
            xacro_file,
            " prefix:=",
            prefix_val,
            " use_mock:=",
            use_fake_hardware_val,
            " robot_ip:=",
            robot_ip_val,
            " cmd_port:=",
            cmd_port_val,
            " servo_port:=",
            servo_port_val,
            " status_port:=",
            status_port_val,
            " motion_period_ms:=",
            motion_period_ms_val,
            " control_mode:=",
            mode_cfg["control_mode"],
            " kinematics_file:=",
            target_calib_file,
        ]
    )
    robot_description = {"robot_description": robot_description_config}
    robot_description_semantic_text = load_srdf_with_prefix(conf_pkg, robot_model_val, prefix_val)
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_text
    }

    kinematics_yaml = load_yaml("estun_moveit_config", "config/kinematics.yaml")
    joint_limits_path = os.path.join("config", robot_model_val, "joint_limits.yaml")
    raw_joint_limits_yaml = load_yaml("estun_description", joint_limits_path)
    if not raw_joint_limits_yaml:
        raise RuntimeError(
            f"加载机型 joint_limits 失败: estun_description/{joint_limits_path}"
        )
    joint_limits_yaml = with_prefixed_joint_limits(raw_joint_limits_yaml, prefix_val)
    raw_moveit_controllers_yaml = load_yaml(
        "estun_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers_yaml = with_prefixed_moveit_controllers(
        raw_moveit_controllers_yaml, prefix_val
    )
    ompl_yaml = load_yaml("estun_moveit_config", "config/ompl_planning.yaml")
    pilz_yaml = load_yaml("estun_moveit_config", "config/pilz_planning.yaml")
    pilz_cartesian_limits_yaml = load_robot_cartesian_limits_yaml(robot_model_val)
    raw_servo_yaml = load_yaml("estun_moveit_config", "config/servo.yaml")

    planning_pipeline_parameters = [
        {"planning_pipelines": {"pipeline_names": planning_pipeline_names}},
        {"default_planning_pipeline": effective_default_planning_pipeline},
    ]
    robot_description_planning_yaml = copy.deepcopy(joint_limits_yaml) if joint_limits_yaml else {}
    if "ompl" in planning_pipeline_names:
        if not ompl_yaml:
            raise RuntimeError("启用了 OMPL，但 config/ompl_planning.yaml 加载失败")
        planning_pipeline_parameters.append(ompl_yaml)
    if "pilz_industrial_motion_planner" in planning_pipeline_names:
        if not pilz_yaml:
            raise RuntimeError("启用了 Pilz，但 config/pilz_planning.yaml 加载失败")
        planning_pipeline_parameters.append(pilz_yaml)
        robot_description_planning_yaml = with_pilz_cartesian_limits(
            robot_description_planning_yaml,
            pilz_cartesian_limits_yaml,
        )

    servo_params_yaml = None
    if enable_servo:
        motion_period_ms = parse_positive_int(motion_period_ms_val, "motion_period_ms")
        base_link, tip_link = extract_group_chain_links(
            srdf_text=robot_description_semantic_text,
            group_name="estun_arm",
        )
        if not base_link:
            base_link = add_prefix_if_needed("base_link", prefix_val)
        if not tip_link:
            tip_link = add_prefix_if_needed("flange", prefix_val)
        servo_params_yaml = with_servo_runtime_overrides(
            raw_yaml=raw_servo_yaml,
            move_group_name="estun_arm",
            planning_frame=base_link,
            ee_frame=tip_link,
            command_out_topic="/forward_position_controller/commands",
            publish_period_sec=float(motion_period_ms) / 1000.0,
        )
    moveit_manage_controllers = False
    trajectory_execution = {
        "moveit_manage_controllers": moveit_manage_controllers,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Keep disabled to avoid false timeout on slower/indirect controller paths.
        "trajectory_execution.execution_duration_monitoring": False,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    publish_robot_description_semantic = {
        "publish_robot_description_semantic": True,
    }

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(estun_hw_share, "launch", "estun_control.launch.py")
        ),
        launch_arguments={
            "prefix": prefix_val,
            "use_fake_hardware": use_fake_hardware_val,
            "model": robot_model_val,
            "robot_ip": robot_ip_val,
            "cmd_port": cmd_port_val,
            "servo_port": servo_port_val,
            "status_port": status_port_val,
            "motion_period_ms": motion_period_ms_val,
            "control_mode": control_mode_val,
            "control_node_impl": control_node_impl_val,
            "use_logical_control_time": use_logical_control_time_val,
            "enable_control_loop_diag_log": enable_control_loop_diag_log_val,
            "controllers_file": controllers_file_val,
        }.items(),
    )

    nodes_to_launch = [control_launch]
    moveit_nodes_to_launch = []

    if enable_moveit:
        moveit_nodes_to_launch.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"robot_description_kinematics": kinematics_yaml},
                    (
                        {"robot_description_planning": robot_description_planning_yaml}
                        if robot_description_planning_yaml
                        else {}
                    ),
                    trajectory_execution,
                    moveit_controllers_yaml,
                    planning_scene_monitor_parameters,
                    publish_robot_description_semantic,
                    *planning_pipeline_parameters,
                ],
            )
        )

    if enable_servo:
        moveit_nodes_to_launch.append(
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                output="screen",
                parameters=[
                    {"moveit_servo": servo_params_yaml},
                    robot_description,
                    robot_description_semantic,
                    {"robot_description_kinematics": kinematics_yaml},
                    (
                        {"robot_description_planning": robot_description_planning_yaml}
                        if robot_description_planning_yaml
                        else {}
                    ),
                ],
            )
        )

    if enable_rviz:
        moveit_nodes_to_launch.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(conf_pkg, "rviz", "moveit.rviz")],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"robot_description_kinematics": kinematics_yaml},
                    *planning_pipeline_parameters,
                ],
            )
        )

    if moveit_nodes_to_launch:
        if moveit_start_delay_sec > 0.0:
            print(
                "[INFO] MoveIt/RViz 延迟启动: "
                f"{moveit_start_delay_sec:.1f}s，等待真机控制链路完成首包检查。"
            )
            nodes_to_launch.append(
                TimerAction(
                    period=moveit_start_delay_sec,
                    actions=moveit_nodes_to_launch,
                )
            )
        else:
            nodes_to_launch.extend(moveit_nodes_to_launch)

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value="", description="机器人命名前缀"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="虚拟硬件"),
        DeclareLaunchArgument("model", default_value="ER20-1780-A6", description="机型"),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="机器人控制器 IP；真机必填，虚拟硬件可留空",
        ),
        DeclareLaunchArgument("cmd_port", default_value="61210", description="CMD"),
        DeclareLaunchArgument("servo_port", default_value="61211", description="SERVO"),
        DeclareLaunchArgument("status_port", default_value="61212", description="STATUS"),
        DeclareLaunchArgument(
            "motion_period_ms",
            default_value="4",
            description="ERI 运动周期 (ms)。4ms 对应约 250Hz。",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="plan",
            description="控制模式: plan|apos|cpos",
        ),
        DeclareLaunchArgument(
            "control_node_impl",
            default_value="auto",
            description="控制节点实现: auto|official|estun",
        ),
        DeclareLaunchArgument(
            "use_logical_control_time",
            default_value="true",
            description="estun_control_node 时间源: true=逻辑时钟, false=墙钟",
        ),
        DeclareLaunchArgument(
            "enable_control_loop_diag_log",
            default_value="false",
            description="是否输出 estun_control_node 周期诊断日志: true|false",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/estun_6dof_controllers.yaml",
            description="控制器配置文件（相对 estun_hardware share 或绝对路径）",
        ),
        DeclareLaunchArgument(
            "start_moveit",
            default_value="auto",
            description="是否启动 move_group: auto|true|false（auto: plan=true, apos/cpos=false）",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="auto",
            description="是否启动 RViz: auto|true|false（auto: plan=true, apos/cpos=false）",
        ),
        DeclareLaunchArgument(
            "start_servo",
            default_value="auto",
            description="是否启动 MoveIt Servo: auto|true|false（auto: apos+moveit=true，其余=false）",
        ),
        DeclareLaunchArgument(
            "moveit_start_delay_sec",
            default_value="4.0",
            description="真机 Estun 控制节点下 MoveIt/RViz 延迟启动秒数，用于避开硬件首包失败时的退出抖动。",
        ),
        DeclareLaunchArgument(
            "planning_pipelines",
            default_value="ompl,pilz",
            description="规划管线，逗号分隔: ompl,pilz",
        ),
        DeclareLaunchArgument(
            "default_planning_pipeline",
            default_value="ompl",
            description="默认规划管线: ompl|pilz",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

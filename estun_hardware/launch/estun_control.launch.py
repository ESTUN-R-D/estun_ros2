import os
import sys
import tempfile
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command


def _import_controller_params_transform():
    try:
        import estun_hardware_launch.controller_params_transform as helper_module

        return helper_module
    except ModuleNotFoundError:
        launch_file = Path(__file__).resolve()
        python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
        candidate_roots = [
            launch_file.parent.parent,
            launch_file.parents[3] / "local" / "lib" / python_version / "dist-packages",
        ]
        for candidate_root in candidate_roots:
            helper_path = (
                candidate_root
                / "estun_hardware_launch"
                / "controller_params_transform.py"
            )
            if not helper_path.is_file():
                continue
            candidate_root_str = str(candidate_root)
            if candidate_root_str not in sys.path:
                sys.path.insert(0, candidate_root_str)
        try:
            import estun_hardware_launch.controller_params_transform as helper_module

            return helper_module
        except ModuleNotFoundError as exc:
            searched = ", ".join(str(root) for root in candidate_roots)
            raise RuntimeError(
                "无法导入控制器参数编排模块 estun_hardware_launch.controller_params_transform；"
                f"已检查候选包根目录: {searched}"
            ) from exc


_controller_params_transform = _import_controller_params_transform()

build_runtime_controller_params = _controller_params_transform.build_runtime_controller_params
with_motion_period_update_rate = _controller_params_transform.with_motion_period_update_rate
with_node_namespace = _controller_params_transform.with_node_namespace
with_prefixed_controllers = _controller_params_transform.with_prefixed_controllers


class QuotedStringDumper(yaml.SafeDumper):
    pass


def _quoted_str_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style='"')


QuotedStringDumper.add_representer(str, _quoted_str_representer)


def load_yaml_from_path(full_path):
    try:
        with open(full_path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def resolve_controllers_yaml_path(controllers_file):
    expanded_path = os.path.expanduser(controllers_file)
    if os.path.isabs(expanded_path):
        return expanded_path
    package_path = get_package_share_directory("estun_hardware")
    return os.path.join(package_path, expanded_path)


def resolve_control_mode(control_mode_raw):
    mode = control_mode_raw.strip().lower()
    mode_map = {
        "plan": {
            "control_mode": "plan",
            "hardware_servo_mode": "apos",
            "default_controller": "estun_arm_controller",
            "stream_policy": "fifo",
        },
        "apos": {
            "control_mode": "apos",
            "hardware_servo_mode": "apos",
            "default_controller": "forward_position_controller",
            "stream_policy": "latest_overwrite",
        },
        "cpos": {
            "control_mode": "cpos",
            "hardware_servo_mode": "cpos",
            "default_controller": "cartesian_forward_controller",
            "stream_policy": "latest_overwrite",
        },
    }
    if mode not in mode_map:
        raise RuntimeError(
            f"无效 control_mode='{control_mode_raw}'，仅支持: plan | apos | cpos"
        )
    return mode_map[mode]


def normalize_namespace(namespace):
    value = namespace.strip().strip("/")
    return value


def scoped_controller_manager(namespace):
    normalized = normalize_namespace(namespace)
    if not normalized:
        return "/controller_manager"
    return f"/{normalized}/controller_manager"


def parse_motion_period_ms(raw_value):
    try:
        value = int(str(raw_value).strip())
    except (TypeError, ValueError):
        raise RuntimeError(f"motion_period_ms 非法: '{raw_value}'，必须为正整数")
    if value <= 0:
        raise RuntimeError(f"motion_period_ms 非法: {value}，必须 > 0")
    return value


def parse_bool(raw_value, param_name):
    value = str(raw_value).strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(
        f"{param_name} 非法: '{raw_value}'，必须为 true/false"
    )


def resolve_control_node_impl(control_node_impl_raw, use_fake_hardware):
    impl = control_node_impl_raw.strip().lower()
    if impl not in ("auto", "official", "estun"):
        raise RuntimeError(
            f"无效 control_node_impl='{control_node_impl_raw}'，仅支持: auto | official | estun"
        )
    if impl == "auto":
        return "official" if use_fake_hardware else "estun"
    return impl


def validate_robot_ip(robot_ip_raw, use_fake_hardware):
    value = str(robot_ip_raw).strip()
    if use_fake_hardware:
        return value
    if not value:
        raise RuntimeError(
            "use_fake_hardware=false 时必须显式传入 robot_ip，例如 robot_ip:=<ROBOT_IP>"
        )
    return value


def dump_yaml_to_tempfile(data, prefix):
    fd, path = tempfile.mkstemp(prefix=prefix, suffix=".yaml")
    with os.fdopen(fd, "w") as file:
        yaml.dump(data, file, sort_keys=False, Dumper=QuotedStringDumper)
    return path


def cleanup_tempfiles(paths):
    for path in paths:
        if not path:
            continue
        try:
            os.unlink(path)
        except FileNotFoundError:
            continue
        except OSError as exc:
            print(f"[WARN] 删除临时控制器 YAML 失败: {path} ({exc})", flush=True)


def cleanup_tempfiles_on_shutdown(paths):
    cleanup_tempfiles(paths)
    return []


def launch_setup(context, *args, **kwargs):
    tempfiles_to_cleanup = []
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
    robot_model_val = LaunchConfiguration("model").perform(context)
    controllers_file_val = LaunchConfiguration("controllers_file").perform(context).strip()
    namespace_val = LaunchConfiguration("namespace").perform(context)
    normalized_namespace = normalize_namespace(namespace_val)
    controller_manager_ns = scoped_controller_manager(namespace_val)

    if not controllers_file_val:
        raise RuntimeError("controllers_file 不能为空")
    use_fake_hardware = parse_bool(use_fake_hardware_val, "use_fake_hardware")
    robot_ip_val = validate_robot_ip(robot_ip_val, use_fake_hardware)
    use_logical_control_time = parse_bool(
        use_logical_control_time_val, "use_logical_control_time"
    )
    enable_control_loop_diag_log = parse_bool(
        enable_control_loop_diag_log_val, "enable_control_loop_diag_log"
    )
    selected_control_node_impl = resolve_control_node_impl(
        control_node_impl_val, use_fake_hardware
    )
    motion_period_ms = parse_motion_period_ms(motion_period_ms_val)

    mode_cfg = resolve_control_mode(control_mode_val)
    default_controller_to_spawn = mode_cfg["default_controller"]

    desc_pkg = get_package_share_directory("estun_description")
    estun_hw_share = get_package_share_directory("estun_hardware")
    estun_libs_share = get_package_share_directory("estun_libs")
    sdk_runtime_dir = os.path.join(estun_libs_share, "config")
    xacro_file = os.path.join(estun_hw_share, "robot", f"{robot_model_val}.urdf.xacro")

    print(
        "[INFO] 控制节点实现: "
        f"requested={control_node_impl_val} selected={selected_control_node_impl}",
        flush=True,
    )
    print(f"[INFO] SDK 运行目录: {sdk_runtime_dir}", flush=True)
    if selected_control_node_impl == "estun":
        control_time_source = (
            "logical(t0 + tick*period)" if use_logical_control_time else "wall(now)"
        )
        print(f"[INFO] 控制循环时间源: {control_time_source}", flush=True)

    actual_path = os.path.join(desc_pkg, "config", robot_model_val, "calibration.yaml")
    nominal_path = os.path.join(desc_pkg, "config", robot_model_val, "default_kinematics.yaml")
    if os.path.exists(actual_path):
        target_calib_file = actual_path
    elif os.path.exists(nominal_path):
        target_calib_file = nominal_path
    else:
        target_calib_file = ""
        print("[WARN] 未找到任何参数文件，Xacro 解析可能失败。")

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

    try:
        controllers_yaml_path = resolve_controllers_yaml_path(controllers_file_val)
        raw_controllers_yaml = load_yaml_from_path(controllers_yaml_path)
        if raw_controllers_yaml is None:
            raise RuntimeError(f"加载控制器配置失败: {controllers_yaml_path}")
        required_manager_controllers = {
            "joint_state_broadcaster",
            "estun_arm_controller",
            "forward_position_controller",
            "cartesian_forward_controller",
        }
        if not use_fake_hardware:
            required_manager_controllers.update(
                {"estun_state_broadcaster", "estun_do_controller"}
            )

        document, ros2_controllers_yaml, ros2_control_overrides_yaml = (
            build_runtime_controller_params(
                raw_controllers_yaml,
                normalized_namespace,
                prefix_val,
                motion_period_ms,
            )
        )
        document.validate_required_controller_definitions(required_manager_controllers)

        # 运行时参数 YAML 只在当前 launch 生命周期内有效，因此要覆盖异常路径和关闭路径的清理。
        ros2_controllers_params_file = dump_yaml_to_tempfile(
            ros2_controllers_yaml, "estun_ros2_controllers_"
        )
        ros2_control_overrides_file = dump_yaml_to_tempfile(
            ros2_control_overrides_yaml, "estun_ros2_control_overrides_"
        )
        tempfiles_to_cleanup.extend(
            [ros2_controllers_params_file, ros2_control_overrides_file]
        )

        converger_arguments = [
            "--controller-manager",
            controller_manager_ns,
            "--joint-state-broadcaster",
            "joint_state_broadcaster",
            "--target-controller",
            default_controller_to_spawn,
            "--business-controller",
            "estun_arm_controller",
            "--business-controller",
            "forward_position_controller",
            "--business-controller",
            "cartesian_forward_controller",
            "--timeout-sec",
            "30.0",
            "--switch-timeout-sec",
            "30.0",
        ]
        if not use_fake_hardware:
            converger_arguments.extend(
                [
                    "--always-active-controller",
                    "estun_state_broadcaster",
                    "--always-active-controller",
                    "estun_do_controller",
                ]
            )

        reconcile_controller_state = Node(
            package="estun_hardware",
            executable="controller_state_converger.py",
            namespace=normalized_namespace,
            output="screen",
            arguments=converger_arguments,
        )

        control_node_kwargs = {
            "parameters": [robot_description],
            "arguments": [
                "--ros-args",
                "--params-file",
                ros2_controllers_params_file,
                "--params-file",
                ros2_control_overrides_file,
            ],
            "output": "both",
            "cwd": sdk_runtime_dir,
        }
        if selected_control_node_impl == "estun":
            control_node_kwargs["parameters"] = [
                robot_description,
                {"use_logical_control_time": use_logical_control_time},
                {"enable_control_loop_diag_log": enable_control_loop_diag_log},
            ]
            control_node = Node(
                package="estun_hardware",
                executable="estun_control_node",
                namespace=normalized_namespace,
                **control_node_kwargs,
            )
        else:
            control_node = Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=normalized_namespace,
                **control_node_kwargs,
            )

        shutdown_on_control_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="control node exited")),
                ],
            )
        )
        cleanup_runtime_params_on_shutdown = RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: cleanup_tempfiles_on_shutdown(
                    tempfiles_to_cleanup
                )
            )
        )

        nodes_to_launch = [
            control_node,
            shutdown_on_control_exit,
            cleanup_runtime_params_on_shutdown,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=normalized_namespace,
                parameters=[robot_description],
            ),
            reconcile_controller_state,
        ]
        return nodes_to_launch
    except Exception:
        cleanup_tempfiles(tempfiles_to_cleanup)
        raise


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("namespace", default_value="", description="ROS 命名空间"),
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
            description=(
                "控制器配置文件（支持相对节点 key、绝对节点 key、'/**' 通配 key；"
                "路径可为 estun_hardware share 相对路径或绝对路径）"
            ),
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

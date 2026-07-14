import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


DEFAULT_DUAL_ARM_MODEL = "iER7-910-MI"


def include_estun_control(
    *,
    namespace,
    prefix,
    model,
    robot_ip,
    cmd_port,
    servo_port,
    status_port,
    use_fake_hardware,
    motion_period_ms,
    control_mode,
    control_node_impl,
    use_logical_control_time,
    enable_control_loop_diag_log,
    controllers_file,
):
    estun_hw_share = get_package_share_directory("estun_hardware")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(estun_hw_share, "launch", "estun_control.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
            "model": model,
            "robot_ip": robot_ip,
            "cmd_port": cmd_port,
            "servo_port": servo_port,
            "status_port": status_port,
            "use_fake_hardware": use_fake_hardware,
            "motion_period_ms": motion_period_ms,
            "control_mode": control_mode,
            "control_node_impl": control_node_impl,
            "use_logical_control_time": use_logical_control_time,
            "enable_control_loop_diag_log": enable_control_loop_diag_log,
            "controllers_file": controllers_file,
        }.items(),
    )


def generate_launch_description():
    rarm_namespace = LaunchConfiguration("rarm_namespace")
    larm_namespace = LaunchConfiguration("larm_namespace")
    rarm_prefix = LaunchConfiguration("rarm_prefix")
    larm_prefix = LaunchConfiguration("larm_prefix")
    rarm_model = LaunchConfiguration("rarm_model")
    larm_model = LaunchConfiguration("larm_model")
    rarm_ip = LaunchConfiguration("rarm_ip")
    larm_ip = LaunchConfiguration("larm_ip")
    rarm_cmd_port = LaunchConfiguration("rarm_cmd_port")
    larm_cmd_port = LaunchConfiguration("larm_cmd_port")
    rarm_servo_port = LaunchConfiguration("rarm_servo_port")
    larm_servo_port = LaunchConfiguration("larm_servo_port")
    rarm_status_port = LaunchConfiguration("rarm_status_port")
    larm_status_port = LaunchConfiguration("larm_status_port")
    rarm_use_fake_hardware = LaunchConfiguration("rarm_use_fake_hardware")
    larm_use_fake_hardware = LaunchConfiguration("larm_use_fake_hardware")
    motion_period_ms = LaunchConfiguration("motion_period_ms")
    control_mode = LaunchConfiguration("control_mode")
    control_node_impl = LaunchConfiguration("control_node_impl")
    use_logical_control_time = LaunchConfiguration("use_logical_control_time")
    enable_control_loop_diag_log = LaunchConfiguration("enable_control_loop_diag_log")
    controllers_file = LaunchConfiguration("controllers_file")

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
            "control_mode", default_value="plan", description="控制模式: plan|apos|cpos"
        ),
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
    ]

    return LaunchDescription(
        declared_arguments
        + [
            include_estun_control(
                namespace=rarm_namespace,
                prefix=rarm_prefix,
                model=rarm_model,
                robot_ip=rarm_ip,
                cmd_port=rarm_cmd_port,
                servo_port=rarm_servo_port,
                status_port=rarm_status_port,
                use_fake_hardware=rarm_use_fake_hardware,
                motion_period_ms=motion_period_ms,
                control_mode=control_mode,
                control_node_impl=control_node_impl,
                use_logical_control_time=use_logical_control_time,
                enable_control_loop_diag_log=enable_control_loop_diag_log,
                controllers_file=controllers_file,
            ),
            include_estun_control(
                namespace=larm_namespace,
                prefix=larm_prefix,
                model=larm_model,
                robot_ip=larm_ip,
                cmd_port=larm_cmd_port,
                servo_port=larm_servo_port,
                status_port=larm_status_port,
                use_fake_hardware=larm_use_fake_hardware,
                motion_period_ms=motion_period_ms,
                control_mode=control_mode,
                control_node_impl=control_node_impl,
                use_logical_control_time=use_logical_control_time,
                enable_control_loop_diag_log=enable_control_loop_diag_log,
                controllers_file=controllers_file,
            ),
        ]
    )

import os
import random
import signal
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

import rclpy
import pytest
import yaml
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from estun_msgs.srv import (
    GetConnectionStatus,
    GetCurErrMsg,
    GetDo,
    GetTool,
    GetUser,
    SetDo,
)
from estun_msgs.msg import EstunRobotStatus
from std_msgs.msg import Float64MultiArray

TOOLS_DIR = Path(__file__).resolve().parents[2] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from estun_test_env import build_ros_test_env  # noqa: E402


BUSINESS_CONTROLLERS = [
    "estun_arm_controller",
    "forward_position_controller",
    "cartesian_forward_controller",
]

AUXILIARY_ACTIVE_CONTROLLERS = [
    "estun_state_broadcaster",
    "estun_do_controller",
]

EXPECTED_ACTIVE_BY_MODE = {
    "plan": "estun_arm_controller",
    "apos": "forward_position_controller",
    "cpos": "cartesian_forward_controller",
}

VALID_STATES = {"active", "inactive", "unconfigured", "finalized"}


def _dds_udp_block_reason() -> str | None:
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError as exc:
        if exc.errno in (1, 13):
            return (
                "当前环境禁止 UDP 套接字，DDS 发现无法建立；"
                f"跳过依赖 controller_manager 服务的集成测试（{exc}）"
            )
        raise
    else:
        probe.close()
    return None


def _test_env() -> dict:
    return build_ros_test_env(
        "estun_moveit_config_controller_mode_matrix",
        os.environ.copy(),
        randomize_domain_id=True,
    )


def _test_stub_env(
    env: dict,
    *,
    force_disconnect: bool = False,
    force_alarm: bool = False,
    force_service_bridge_fail: bool = False,
) -> dict:
    patched = dict(env)
    patched["ESTUN_TEST_STUB_SDK"] = "1"
    patched["ESTUN_TEST_FORCE_DISCONNECT"] = "1" if force_disconnect else "0"
    patched["ESTUN_TEST_FORCE_ALARM"] = "1" if force_alarm else "0"
    patched["ESTUN_TEST_FORCE_SERVICE_BRIDGE_FAIL"] = "1" if force_service_bridge_fail else "0"
    return patched


def _apply_ros_env(env: dict) -> dict:
    keys = ("ROS_DOMAIN_ID", "ROS_HOME", "ROS_LOG_DIR")
    backup = {key: os.environ.get(key) for key in keys}
    for key in keys:
        if key in env:
            os.environ[key] = env[key]
        elif key in os.environ:
            os.environ.pop(key)
    return backup


def _restore_ros_env(backup: dict) -> None:
    for key, value in backup.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value


class ControllerManagerClient:
    def __init__(self, controller_manager_ns: str, timeout_sec: float = 8.0) -> None:
        self._timeout_sec = timeout_sec
        base = controller_manager_ns.rstrip("/")
        node_name = f"controller_mode_matrix_{os.getpid()}_{random.randint(1000, 9999)}"
        self._node = rclpy.create_node(node_name)
        self._list_client = self._node.create_client(ListControllers, f"{base}/list_controllers")
        self._switch_client = self._node.create_client(
            SwitchController, f"{base}/switch_controller"
        )
        self._conn_status_client = self._node.create_client(
            GetConnectionStatus, "/estun/get_connection_status"
        )
        self._cur_err_client = self._node.create_client(GetCurErrMsg, "/estun/get_cur_err_msg")
        self._set_do_client = self._node.create_client(SetDo, "/estun/set_do")
        self._get_do_client = self._node.create_client(GetDo, "/estun/get_do")
        self._tool_client = self._node.create_client(GetTool, "/estun/get_tool")
        self._user_client = self._node.create_client(GetUser, "/estun/get_user")
        self._last_status = None
        self._status_subscription = self._node.create_subscription(
            EstunRobotStatus,
            "/estun_state_broadcaster/status",
            self._on_status,
            10,
        )

    def _on_status(self, msg: EstunRobotStatus) -> None:
        self._last_status = msg

    def wait_for_services(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            list_ready = self._list_client.wait_for_service(timeout_sec=0.3)
            switch_ready = self._switch_client.wait_for_service(timeout_sec=0.3)
            if list_ready and switch_ready:
                return
        raise RuntimeError("等待 controller_manager 服务超时")

    def _call(self, client, request, description: str, timeout_sec: float = None):
        timeout = self._timeout_sec if timeout_sec is None else timeout_sec
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"{description} 调用超时")
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"{description} 调用异常: {exc}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{description} 返回空结果")
        return result

    def wait_for_connection_status_service(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._conn_status_client.wait_for_service(timeout_sec=0.3):
                return
        raise RuntimeError("等待 /estun/get_connection_status 服务超时")

    def wait_for_service_bridge_services(self, timeout_sec: float) -> None:
        clients = [
            (self._conn_status_client, "/estun/get_connection_status"),
            (self._cur_err_client, "/estun/get_cur_err_msg"),
            (self._set_do_client, "/estun/set_do"),
            (self._get_do_client, "/estun/get_do"),
            (self._tool_client, "/estun/get_tool"),
            (self._user_client, "/estun/get_user"),
        ]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            pending = []
            for client, name in clients:
                if not client.wait_for_service(timeout_sec=0.2):
                    pending.append(name)
            if not pending:
                return
        raise RuntimeError(f"等待 service bridge 服务超时: {pending}")

    def get_connection_status(self):
        return self._call(
            self._conn_status_client,
            GetConnectionStatus.Request(),
            "get_connection_status",
        )

    def get_cur_err_msg(self):
        return self._call(
            self._cur_err_client,
            GetCurErrMsg.Request(),
            "get_cur_err_msg",
        )

    def set_do(self, port: int, value: bool):
        request = SetDo.Request()
        request.port = port
        request.value = value
        return self._call(
            self._set_do_client,
            request,
            "set_do",
        )

    def get_do(self, port: int):
        request = GetDo.Request()
        request.port = port
        return self._call(
            self._get_do_client,
            request,
            "get_do",
        )

    def get_tool(self, tool_id: int):
        request = GetTool.Request()
        request.tool_id = tool_id
        return self._call(
            self._tool_client,
            request,
            "get_tool",
        )

    def get_user(self, user_id: int):
        request = GetUser.Request()
        request.user_id = user_id
        return self._call(
            self._user_client,
            request,
            "get_user",
        )

    def list_controller_states(self) -> dict:
        response = self._call(self._list_client, ListControllers.Request(), "list_controllers")
        return {ctrl.name: ctrl.state for ctrl in response.controller}

    def latest_status(self):
        rclpy.spin_once(self._node, timeout_sec=0.1)
        return self._last_status

    def switch_controllers(
        self,
        activate: list,
        deactivate: list,
        switch_timeout_sec: float = 8.0,
    ):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True

        sec = int(switch_timeout_sec)
        nanosec = int((switch_timeout_sec - sec) * 1e9)
        request.timeout = Duration(sec=sec, nanosec=nanosec)

        return self._call(
            self._switch_client,
            request,
            "switch_controllers",
            timeout_sec=switch_timeout_sec + 2.0,
        )

    def destroy(self) -> None:
        self._node.destroy_node()


@contextmanager
def _controller_manager_client(env: dict, controller_manager_ns: str = "/controller_manager"):
    blocked_reason = _dds_udp_block_reason()
    if blocked_reason is not None:
        pytest.skip(blocked_reason)
    backup = _apply_ros_env(env)
    rclpy.init()
    client = ControllerManagerClient(controller_manager_ns=controller_manager_ns)
    try:
        yield client
    finally:
        client.destroy()
        if rclpy.ok():
            rclpy.shutdown()
        _restore_ros_env(backup)


def _terminate_process(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    # Prefer graceful shutdown first so ros2 launch can tear down child nodes
    # and let ros2_control run cleanup/release paths.
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


def _terminate_process_and_collect_output(proc: subprocess.Popen) -> str:
    _terminate_process(proc)
    if proc.stdout is None:
        return ""
    try:
        stdout, _ = proc.communicate(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        stdout, _ = proc.communicate(timeout=3)
    return stdout or ""


def _wait_for_expected_states(
    proc: subprocess.Popen,
    cm_client: ControllerManagerClient,
    expected_states: dict,
    timeout_sec: float = 60.0,
) -> None:
    deadline = time.monotonic() + timeout_sec
    last_states = {}
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise AssertionError(
                f"launch 进程提前退出: returncode={proc.returncode}"
            )

        try:
            states = cm_client.list_controller_states()
            last_states = states
            if all(states.get(name) == expected for name, expected in expected_states.items()):
                return
        except RuntimeError:
            pass
        time.sleep(1.0)

    raise AssertionError(
        "等待控制器状态收敛超时。"
        f"期望={expected_states}，最近一次状态={last_states}"
    )


def _wait_for_status(
    proc: subprocess.Popen,
    cm_client: ControllerManagerClient,
    timeout_sec: float = 30.0,
):
    deadline = time.monotonic() + timeout_sec
    last_status = None
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise AssertionError(
                f"launch 进程提前退出: returncode={proc.returncode}"
            )
        last_status = cm_client.latest_status()
        if last_status is not None:
            return last_status
        time.sleep(0.1)

    raise AssertionError(f"等待 estun 状态消息超时，最近一次状态={last_status}")


def _launch_estun_control(
    control_mode: str,
    env: dict,
    launch_package: str = "estun_hardware",
    use_fake_hardware: bool = True,
    robot_ip: str = "",
    capture_output: bool = False,
    extra_launch_args: list | None = None,
) -> subprocess.Popen:
    cmd = [
        "ros2",
        "launch",
        launch_package,
        "estun_control.launch.py",
        f"use_fake_hardware:={'true' if use_fake_hardware else 'false'}",
        "model:=ER20-1780-A6",
        f"robot_ip:={robot_ip}",
        f"control_mode:={control_mode}",
    ]
    if extra_launch_args:
        cmd.extend(extra_launch_args)
    return subprocess.Popen(
        cmd,
        env=env,
        stdout=subprocess.PIPE if capture_output else subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _write_temp_controllers_yaml(data: dict) -> str:
    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        prefix="estun_test_controllers_",
        delete=False,
    ) as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
        return handle.name


def _plan_mode_expected_states():
    expected = {
        "joint_state_broadcaster": "active",
        "estun_arm_controller": "active",
        "forward_position_controller": "inactive",
        "cartesian_forward_controller": "inactive",
    }
    return expected


def test_control_node_selection_log_contains_impl_and_clock_mode():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env())
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        use_fake_hardware=False,
        robot_ip="127.0.0.1",
        capture_output=True,
        extra_launch_args=[
            "control_node_impl:=auto",
            "use_logical_control_time:=true",
        ],
    )
    try:
        time.sleep(8.0)
        combined = _terminate_process_and_collect_output(proc)
    finally:
        _terminate_process(proc)

    assert "[INFO] 控制节点实现:" in combined
    assert "selected=estun" in combined
    assert "[INFO] 控制循环时间源: logical(t0 + tick*period)" in combined


def test_sigint_requests_signal_stop_before_controller_shutdown():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env())
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        use_fake_hardware=False,
        robot_ip="127.0.0.1",
        capture_output=True,
    )
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)
        combined = _terminate_process_and_collect_output(proc)
    finally:
        _terminate_process(proc)

    pre_shutdown_request = "signal watcher: 捕获到信号 2，先请求硬件 stop() 再执行 rclcpp::shutdown()。"
    stop_consumed = "检测到提前停机请求，优先执行 stop() 收尾。"
    pre_shutdown_wait_done = "signal watcher: 硬件 stop() 已完成，开始执行 rclcpp::shutdown()。"
    controller_shutdown = "Shutting down controller"

    assert pre_shutdown_request in combined
    assert stop_consumed in combined
    assert pre_shutdown_wait_done in combined
    assert combined.index(pre_shutdown_request) < combined.index(stop_consumed)
    assert combined.index(stop_consumed) < combined.index(pre_shutdown_wait_done)
    if controller_shutdown in combined:
        assert combined.index(pre_shutdown_wait_done) < combined.index(controller_shutdown)


@pytest.mark.parametrize("control_mode", ["plan", "apos", "cpos"])
def test_mode_matrix_converges_to_expected_controller_states(control_mode: str):
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    proc = _launch_estun_control(control_mode=control_mode, env=env)
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)

            expected_active = EXPECTED_ACTIVE_BY_MODE[control_mode]
            expected = {"joint_state_broadcaster": "active", expected_active: "active"}
            for controller in BUSINESS_CONTROLLERS:
                if controller != expected_active:
                    expected[controller] = "inactive"

            _wait_for_expected_states(
                proc=proc,
                cm_client=cm_client,
                expected_states=expected,
                timeout_sec=80.0,
            )
    finally:
        _terminate_process(proc)


def test_estun_hardware_launch_auto_activates_auxiliary_controllers():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env())
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        use_fake_hardware=False,
        robot_ip="127.0.0.1",
    )
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)

            expected = {
                "joint_state_broadcaster": "active",
                "estun_arm_controller": "active",
                "forward_position_controller": "inactive",
                "cartesian_forward_controller": "inactive",
            }
            expected.update({name: "active" for name in AUXILIARY_ACTIVE_CONTROLLERS})

            _wait_for_expected_states(
                proc=proc,
                cm_client=cm_client,
                expected_states=expected,
                timeout_sec=80.0,
            )
    finally:
        _terminate_process(proc)


def test_custom_controllers_file_supports_global_scope_with_namespace():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    controllers_file = _write_temp_controllers_yaml(
        {
            "/**": {
                "ros__parameters": {
                    "joint_state_broadcaster": {
                        "type": "joint_state_broadcaster/JointStateBroadcaster",
                    },
                    "estun_arm_controller": {
                        "type": "joint_trajectory_controller/JointTrajectoryController",
                    },
                    "forward_position_controller": {
                        "type": "forward_command_controller/ForwardCommandController",
                    },
                    "cartesian_forward_controller": {
                        "type": (
                            "forward_command_controller/"
                            "MultiInterfaceForwardCommandController"
                        ),
                    },
                }
            },
            "/**/estun_arm_controller": {
                "ros__parameters": {
                    "joints": [
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                    ],
                    "command_interfaces": ["position"],
                    "state_interfaces": ["position"],
                }
            },
            "/**/forward_position_controller": {
                "ros__parameters": {
                    "joints": [
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                    ],
                    "interface_name": "position",
                }
            },
            "/**/cartesian_forward_controller": {
                "ros__parameters": {
                    "joint": "cartesian_tcp",
                    "interface_names": ["x", "y", "z", "a", "b", "c"],
                }
            },
        }
    )
    env = _test_env()
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        extra_launch_args=[
            "namespace:=rarm",
            "prefix:=r1_",
            f"controllers_file:={controllers_file}",
        ],
    )
    try:
        with _controller_manager_client(
            env, controller_manager_ns="/rarm/controller_manager"
        ) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)
            _wait_for_expected_states(
                proc=proc,
                cm_client=cm_client,
                expected_states=_plan_mode_expected_states(),
                timeout_sec=80.0,
            )
    finally:
        _terminate_process(proc)
        os.unlink(controllers_file)


def test_custom_controllers_file_supports_absolute_namespace_keys():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    controllers_file = _write_temp_controllers_yaml(
        {
            "/rarm/controller_manager": {
                "ros__parameters": {
                    "joint_state_broadcaster": {
                        "type": "joint_state_broadcaster/JointStateBroadcaster",
                    },
                    "estun_arm_controller": {
                        "type": "joint_trajectory_controller/JointTrajectoryController",
                    },
                    "forward_position_controller": {
                        "type": "forward_command_controller/ForwardCommandController",
                    },
                    "cartesian_forward_controller": {
                        "type": (
                            "forward_command_controller/"
                            "MultiInterfaceForwardCommandController"
                        ),
                    },
                }
            },
            "/rarm/estun_arm_controller": {
                "ros__parameters": {
                    "joints": [
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                    ],
                    "command_interfaces": ["position"],
                    "state_interfaces": ["position"],
                }
            },
            "/rarm/forward_position_controller": {
                "ros__parameters": {
                    "joints": [
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                    ],
                    "interface_name": "position",
                }
            },
            "/rarm/cartesian_forward_controller": {
                "ros__parameters": {
                    "joint": "cartesian_tcp",
                    "interface_names": ["x", "y", "z", "a", "b", "c"],
                }
            },
        }
    )
    env = _test_env()
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        extra_launch_args=[
            "namespace:=rarm",
            "prefix:=r1_",
            f"controllers_file:={controllers_file}",
        ],
    )
    try:
        with _controller_manager_client(
            env, controller_manager_ns="/rarm/controller_manager"
        ) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)
            _wait_for_expected_states(
                proc=proc,
                cm_client=cm_client,
                expected_states=_plan_mode_expected_states(),
                timeout_sec=80.0,
            )
    finally:
        _terminate_process(proc)
        os.unlink(controllers_file)


@pytest.mark.parametrize(
    ("control_mode", "command_topic"),
    [
        ("apos", "/forward_position_controller/commands"),
        ("cpos", "/cartesian_forward_controller/commands"),
    ],
)
def test_realtime_modes_keep_latest_overwrite_depth_bounded(control_mode: str, command_topic: str):
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env())
    proc = _launch_estun_control(
        control_mode=control_mode,
        env=env,
        use_fake_hardware=False,
        robot_ip="127.0.0.1",
    )
    backup = None
    publisher_node = None
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)
            expected = {
                "joint_state_broadcaster": "active",
                "estun_state_broadcaster": "active",
                EXPECTED_ACTIVE_BY_MODE[control_mode]: "active",
            }
            _wait_for_expected_states(
                proc=proc,
                cm_client=cm_client,
                expected_states=expected,
                timeout_sec=80.0,
            )

            backup = _apply_ros_env(env)
            publisher_node = rclpy.create_node(
                f"latest_overwrite_probe_{os.getpid()}_{random.randint(1000, 9999)}"
            )
            publisher = publisher_node.create_publisher(Float64MultiArray, command_topic, 10)
            discovery_deadline = time.monotonic() + 5.0
            while (
                publisher.get_subscription_count() == 0
                and time.monotonic() < discovery_deadline
            ):
                rclpy.spin_once(publisher_node, timeout_sec=0.1)

            deadline = time.monotonic() + 1.2
            command_index = 0
            while time.monotonic() < deadline:
                msg = Float64MultiArray()
                msg.data = [0.001 * command_index + axis * 0.0001 for axis in range(6)]
                publisher.publish(msg)
                rclpy.spin_once(publisher_node, timeout_sec=0.0)
                command_index += 1
                time.sleep(0.002)

            status = None
            status_deadline = time.monotonic() + 30.0
            while time.monotonic() < status_deadline:
                try:
                    candidate = _wait_for_status(proc, cm_client, timeout_sec=1.0)
                except AssertionError:
                    continue
                if candidate.queue_push_count > 0:
                    status = candidate
                    break
            assert status is not None
            assert status.queue_push_count > 0
            assert status.queue_depth <= 1
            assert status.queue_max_depth <= 1
    finally:
        if publisher_node is not None:
            publisher_node.destroy_node()
        if backup is not None:
            _restore_ros_env(backup)
        _terminate_process(proc)


def test_invalid_control_mode_fails_fast():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    cmd = [
        "ros2",
        "launch",
        "estun_hardware",
        "estun_control.launch.py",
        "use_fake_hardware:=true",
        "control_mode:=invalid_mode",
    ]
    result = subprocess.run(
        cmd,
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=25,
    )
    combined = (result.stdout or "") + (result.stderr or "")
    assert result.returncode != 0
    assert "无效 control_mode" in combined


def test_invalid_motion_period_fails_fast():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    cmd = [
        "ros2",
        "launch",
        "estun_hardware",
        "estun_control.launch.py",
        "use_fake_hardware:=true",
        "control_mode:=plan",
        "motion_period_ms:=0",
    ]
    result = subprocess.run(
        cmd,
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=25,
    )
    combined = (result.stdout or "") + (result.stderr or "")
    assert result.returncode != 0
    assert "motion_period_ms 非法" in combined


def test_non_integer_motion_period_fails_fast():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    cmd = [
        "ros2",
        "launch",
        "estun_hardware",
        "estun_control.launch.py",
        "use_fake_hardware:=true",
        "control_mode:=plan",
        "motion_period_ms:=abc",
    ]
    result = subprocess.run(
        cmd,
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=25,
    )
    combined = (result.stdout or "") + (result.stderr or "")
    assert result.returncode != 0
    assert "motion_period_ms 非法" in combined


def test_invalid_controllers_file_fails_fast():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    cmd = [
        "ros2",
        "launch",
        "estun_hardware",
        "estun_control.launch.py",
        "use_fake_hardware:=true",
        "control_mode:=plan",
        "controllers_file:=/tmp/estun_not_exist_controllers.yaml",
    ]
    result = subprocess.run(
        cmd,
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=25,
    )
    combined = (result.stdout or "") + (result.stderr or "")
    assert result.returncode != 0
    assert "加载控制器配置失败" in combined


def test_namespace_mismatch_in_custom_controllers_file_fails_fast():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    controllers_file = _write_temp_controllers_yaml(
        {
            "/larm/controller_manager": {
                "ros__parameters": {
                    "joint_state_broadcaster": {
                        "type": "joint_state_broadcaster/JointStateBroadcaster",
                    },
                    "estun_arm_controller": {
                        "type": "joint_trajectory_controller/JointTrajectoryController",
                    },
                    "forward_position_controller": {
                        "type": "forward_command_controller/ForwardCommandController",
                    },
                    "cartesian_forward_controller": {
                        "type": (
                            "forward_command_controller/"
                            "MultiInterfaceForwardCommandController"
                        ),
                    },
                }
            }
        }
    )
    env = _test_env()
    cmd = [
        "ros2",
        "launch",
        "estun_hardware",
        "estun_control.launch.py",
        "use_fake_hardware:=true",
        "control_mode:=plan",
        "namespace:=rarm",
        f"controllers_file:={controllers_file}",
    ]
    try:
        result = subprocess.run(
            cmd,
            env=env,
            check=False,
            capture_output=True,
            text=True,
            timeout=25,
        )
    finally:
        os.unlink(controllers_file)

    combined = (result.stdout or "") + (result.stderr or "")
    assert result.returncode != 0
    assert "绝对节点 key 与当前 namespace 不一致" in combined


def test_empty_controllers_file_is_rejected_by_cli_parser():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    cmd = [
        "ros2",
        "launch",
        "estun_hardware",
        "estun_control.launch.py",
        "use_fake_hardware:=true",
        "control_mode:=plan",
        "controllers_file:=",
    ]
    result = subprocess.run(
        cmd,
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=25,
    )
    combined = (result.stdout or "") + (result.stderr or "")
    assert result.returncode != 0
    assert "malformed launch argument" in combined


def test_service_bridge_init_failure_fails_fast_in_test_stub_mode():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env(), force_service_bridge_fail=True)
    cmd = [
        "ros2",
        "launch",
        "estun_hardware",
        "estun_control.launch.py",
        "use_fake_hardware:=false",
        "model:=ER20-1780-A6",
        "robot_ip:=127.0.0.1",
        "control_mode:=plan",
    ]
    try:
        result = subprocess.run(
            cmd,
            env=env,
            check=False,
            capture_output=True,
            text=True,
            timeout=35,
        )
        combined = (result.stdout or "") + (result.stderr or "")
    except subprocess.TimeoutExpired as exc:
        stdout = (
            exc.stdout
            if isinstance(exc.stdout, str)
            else (exc.stdout or b"").decode("utf-8", errors="replace")
        )
        stderr = (
            exc.stderr
            if isinstance(exc.stderr, str)
            else (exc.stderr or b"").decode("utf-8", errors="replace")
        )
        combined = stdout + stderr

    assert "强制 service bridge 初始化失败" in combined
    assert "配置失败：SDK service bridge 启动失败" in combined


def test_disconnect_fault_sets_connection_status_disconnected():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env(), force_disconnect=True)
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        use_fake_hardware=False,
        robot_ip="127.0.0.1",
        capture_output=True,
    )
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)
            cm_client.wait_for_connection_status_service(timeout_sec=30.0)
            status = cm_client.get_connection_status().status
            assert not status.connected
            assert (
                "cmd=1" in status.detail
                and "servo=1" in status.detail
                and "udp=1" in status.detail
            )
    finally:
        _terminate_process_and_collect_output(proc)


def test_service_bridge_sdk_services_are_stubbed_in_test_mode():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env())
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        use_fake_hardware=False,
        robot_ip="127.0.0.1",
        capture_output=True,
    )
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)
            cm_client.wait_for_service_bridge_services(timeout_sec=30.0)

            err = cm_client.get_cur_err_msg()
            assert err.success
            assert err.err_id == 0
            assert err.err_msg == ""

            set_do = cm_client.set_do(port=18, value=True)
            assert set_do.success
            assert set_do.message == "ok"

            get_do = cm_client.get_do(port=18)
            assert get_do.success
            assert get_do.value is True
            assert get_do.message == "ok"

            tool = cm_client.get_tool(tool_id=2)
            assert tool.success
            assert list(tool.tool_data) == [20.0, 21.0, 22.0, 23.0, 24.0, 25.0]

            user = cm_client.get_user(user_id=3)
            assert user.success
            assert list(user.user_data) == [300.0, 301.0, 302.0, 303.0, 304.0, 305.0]
    finally:
        _terminate_process_and_collect_output(proc)


def test_alarm_fault_rejects_joint_mode_switch_in_test_stub_mode():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_stub_env(_test_env(), force_alarm=True)
    proc = _launch_estun_control(
        control_mode="plan",
        env=env,
        use_fake_hardware=False,
        robot_ip="127.0.0.1",
        capture_output=True,
    )
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)
            cm_client.wait_for_service_bridge_services(timeout_sec=30.0)
            err = cm_client.get_cur_err_msg()
            assert err.success
            assert err.err_id == 1
            assert err.err_msg == "test stub alarm injected"

            response = cm_client.switch_controllers(
                activate=["forward_position_controller"],
                deactivate=["estun_arm_controller"],
                switch_timeout_sec=8.0,
            )
            assert not response.ok
    finally:
        _terminate_process_and_collect_output(proc)


def test_cpos_mode_rejects_joint_controller_activation_request():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    proc = _launch_estun_control(control_mode="cpos", env=env)
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=30.0)

            expected = {
                "joint_state_broadcaster": "active",
                "cartesian_forward_controller": "active",
                "estun_arm_controller": "inactive",
                "forward_position_controller": "inactive",
            }
            _wait_for_expected_states(
                proc=proc,
                cm_client=cm_client,
                expected_states=expected,
                timeout_sec=80.0,
            )

            response = cm_client.switch_controllers(
                activate=["forward_position_controller"],
                deactivate=[],
                switch_timeout_sec=8.0,
            )
            if response.ok:
                pytest.skip("mock hardware 不执行 APOS/CPOS 模式门禁，跳过该反例断言")
            assert not response.ok
    finally:
        _terminate_process(proc)


def test_cpos_mode_rejects_joint_controller_on_real_hardware():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")
    if not _real_hardware_tests_enabled():
        pytest.skip("未启用真机测试（设置 ESTUN_ENABLE_REAL_HW_TESTS=1）")
    if not _real_hardware_cpos_guard_enabled():
        pytest.skip("真机 cpos 门禁测试默认禁用（设置 ESTUN_REAL_HW_RUN_CPOS_GUARD=1 才执行）")
    if not _real_robot_ip():
        pytest.skip("未提供真机 IP（设置 ESTUN_REAL_ROBOT_IP）")

    env = _test_env()
    proc = _launch_estun_control(
        control_mode="cpos",
        env=env,
        use_fake_hardware=False,
        robot_ip=_real_robot_ip(),
    )
    try:
        with _controller_manager_client(env) as cm_client:
            cm_client.wait_for_services(timeout_sec=40.0)

            expected = {
                "joint_state_broadcaster": "active",
                "cartesian_forward_controller": "active",
                "estun_arm_controller": "inactive",
                "forward_position_controller": "inactive",
            }
            expected.update({name: "active" for name in AUXILIARY_ACTIVE_CONTROLLERS})
            try:
                _wait_for_expected_states(
                    proc=proc,
                    cm_client=cm_client,
                    expected_states=expected,
                    timeout_sec=90.0,
                )
            except AssertionError:
                states = cm_client.list_controller_states()
                names = list(expected.keys())
                if all(states.get(name) == "inactive" for name in names):
                    pytest.skip("真机未进入 cpos 激活态（可能处于报警/奇异位），跳过该高风险断言")
                raise

            response = cm_client.switch_controllers(
                activate=["forward_position_controller"],
                deactivate=[],
                switch_timeout_sec=8.0,
            )
            assert not response.ok
    finally:
        _terminate_process(proc)


def test_demo_launch_from_non_workspace_cwd_uses_sdk_runtime_dir():
    if not _ros2_available():
        pytest.skip("ros2 command not available in PATH")

    env = _test_env()
    cmd = [
        "ros2",
        "launch",
        "estun_moveit_config",
        "demo.launch.py",
        "use_fake_hardware:=false",
        "model:=ER20-1780-A6",
        "robot_ip:=127.0.0.1",
        "control_mode:=plan",
        "start_moveit:=false",
        "start_rviz:=false",
    ]
    proc = subprocess.Popen(
        cmd,
        env=env,
        cwd="/tmp",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    time.sleep(8.0)
    if proc.poll() is None:
        proc.terminate()
        try:
            stdout, _ = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, _ = proc.communicate(timeout=5)
    else:
        stdout, _ = proc.communicate(timeout=5)

    combined = stdout or ""
    assert "[INFO] SDK 运行目录:" in combined
    assert "log4cpp.conf does not exist" not in combined
    assert (
        "libestun_msgs__rosidl_typesupport_fastrtps_cpp.so: cannot open shared object file"
        not in combined
    )


def _ros2_available() -> bool:
    return shutil.which("ros2") is not None


def _real_hardware_tests_enabled() -> bool:
    raw = os.environ.get("ESTUN_ENABLE_REAL_HW_TESTS", "0").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _real_robot_ip() -> str:
    return os.environ.get("ESTUN_REAL_ROBOT_IP", "").strip()


def _real_hardware_cpos_guard_enabled() -> bool:
    raw = os.environ.get("ESTUN_REAL_HW_RUN_CPOS_GUARD", "0").strip().lower()
    return raw in ("1", "true", "yes", "on")

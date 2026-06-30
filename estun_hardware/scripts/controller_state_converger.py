#!/usr/bin/env python3

import argparse
import sys
from typing import Dict, List, Set

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.node import Node


def safe_log(node: Node, level: str, message: str) -> None:
    context = getattr(node, "context", None)
    context_ok = context is not None and context.ok()
    if context_ok:
        log_fn = getattr(node.get_logger(), level, None)
        if callable(log_fn):
            log_fn(message)
            return
    print(f"[{level.upper()}] {message}", file=sys.stderr)


class ControllerStateConverger(Node):
    def __init__(
        self,
        controller_manager_ns: str,
        joint_state_broadcaster: str,
        target_controller: str,
        business_controllers: List[str],
        always_active_controllers: List[str],
        timeout_sec: float,
        switch_timeout_sec: float,
    ) -> None:
        super().__init__("controller_state_converger")
        self._timeout_sec = timeout_sec
        self._switch_timeout_sec = switch_timeout_sec

        base = controller_manager_ns.rstrip("/")
        self._srv_list = f"{base}/list_controllers"
        self._srv_load = f"{base}/load_controller"
        self._srv_configure = f"{base}/configure_controller"
        self._srv_switch = f"{base}/switch_controller"

        self._joint_state_broadcaster = joint_state_broadcaster
        self._target_controller = target_controller
        self._business_controllers = list(dict.fromkeys(business_controllers))
        self._always_active_controllers = list(dict.fromkeys(always_active_controllers))

        self._required_controllers: Set[str] = set(self._business_controllers)
        self._required_controllers.add(self._joint_state_broadcaster)
        self._required_controllers.add(self._target_controller)
        self._required_controllers.update(self._always_active_controllers)

        self._target_active: Set[str] = {
            self._joint_state_broadcaster,
            self._target_controller,
        }
        self._target_active.update(self._always_active_controllers)

        self._target_inactive: Set[str] = (
            set(self._business_controllers) - self._target_active
        )

        self._list_client = self.create_client(ListControllers, self._srv_list)
        self._load_client = self.create_client(LoadController, self._srv_load)
        self._configure_client = self.create_client(ConfigureController, self._srv_configure)
        self._switch_client = self.create_client(SwitchController, self._srv_switch)

    def _log_step(self, message: str) -> None:
        self.get_logger().debug(message)

    def _wait_for_services(self) -> None:
        for client, name in (
            (self._list_client, self._srv_list),
            (self._load_client, self._srv_load),
            (self._configure_client, self._srv_configure),
            (self._switch_client, self._srv_switch),
        ):
            if not client.wait_for_service(timeout_sec=self._timeout_sec):
                raise RuntimeError(f"等待服务超时: {name}")

    def _call(self, client, request, description: str):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout_sec)
        if not future.done():
            raise RuntimeError(f"{description} 超时")
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"{description} 异常: {exc}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{description} 返回空结果")
        return result

    def _list_controller_states(self) -> Dict[str, str]:
        response = self._call(self._list_client, ListControllers.Request(), "list_controllers")
        return {ctrl.name: ctrl.state for ctrl in response.controller}

    def _load_controller(self, name: str) -> None:
        req = LoadController.Request()
        req.name = name
        resp = self._call(self._load_client, req, f"load_controller({name})")
        if not resp.ok:
            raise RuntimeError(f"加载控制器失败: {name}")

    def _configure_controller(self, name: str) -> None:
        req = ConfigureController.Request()
        req.name = name
        resp = self._call(self._configure_client, req, f"configure_controller({name})")
        if not resp.ok:
            raise RuntimeError(f"配置控制器失败: {name}")

    def _switch_controllers(self, activate: List[str], deactivate: List[str]) -> None:
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True

        sec = int(self._switch_timeout_sec)
        nanosec = int((self._switch_timeout_sec - sec) * 1e9)
        req.timeout = Duration(sec=sec, nanosec=nanosec)

        resp = self._call(
            self._switch_client,
            req,
            f"switch_controller(activate={activate}, deactivate={deactivate})",
        )
        if not resp.ok:
            raise RuntimeError(
                "控制器切换失败: "
                f"activate={activate}, deactivate={deactivate}"
            )

    def converge(self) -> None:
        self.get_logger().debug(
            "开始收敛控制器状态: "
            f"target_active={sorted(self._target_active)}, "
            f"target_inactive={sorted(self._target_inactive)}"
        )

        self._wait_for_services()

        states = self._list_controller_states()
        self._log_step(f"当前控制器状态: {states}")

        for name in sorted(self._required_controllers):
            if name in states:
                continue
            self._log_step(f"控制器未加载，准备加载: {name}")
            self._load_controller(name)

        states = self._list_controller_states()
        for name in sorted(self._required_controllers):
            state = states.get(name, "")
            if state == "unconfigured":
                self._log_step(f"控制器未配置，准备配置: {name}")
                self._configure_controller(name)

        states = self._list_controller_states()
        for name in sorted(self._required_controllers):
            state = states.get(name, "<missing>")
            if state == "finalized":
                raise RuntimeError(f"控制器处于 finalized，无法继续收敛: {name}")

        deactivate = sorted(
            name
            for name, state in states.items()
            if state == "active" and name in self._target_inactive
        )
        activate = sorted(
            name for name in self._target_active if states.get(name, "") != "active"
        )

        if activate or deactivate:
            self._log_step(
                "执行切换: "
                f"activate={activate}, deactivate={deactivate}"
            )
            self._switch_controllers(activate=activate, deactivate=deactivate)
        else:
            self._log_step("当前状态已满足目标状态，无需切换")

        final_states = self._list_controller_states()
        mismatches = []
        for name in sorted(self._target_active):
            actual = final_states.get(name, "<missing>")
            if actual != "active":
                mismatches.append(f"{name}: expect=active, actual={actual}")
        for name in sorted(self._target_inactive):
            actual = final_states.get(name, "<missing>")
            if actual != "inactive":
                mismatches.append(f"{name}: expect=inactive, actual={actual}")

        if mismatches:
            raise RuntimeError("收敛后状态校验失败: " + "; ".join(mismatches))

        self.get_logger().debug(
            "控制器状态收敛完成: "
            f"active={sorted(self._target_active)}, "
            f"inactive={sorted(self._target_inactive)}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Converge ros2_control controller states.")
    parser.add_argument(
        "--controller-manager",
        default="/controller_manager",
        help="controller_manager namespace, e.g. /controller_manager",
    )
    parser.add_argument(
        "--joint-state-broadcaster",
        default="joint_state_broadcaster",
        help="joint state broadcaster controller name",
    )
    parser.add_argument(
        "--target-controller",
        required=True,
        help="controller that must be active for this mode",
    )
    parser.add_argument(
        "--business-controller",
        action="append",
        default=[],
        help="business controller name, can be passed multiple times",
    )
    parser.add_argument(
        "--always-active-controller",
        action="append",
        default=[],
        help="auxiliary controller that must stay active, can be passed multiple times",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=30.0,
        help="service wait/call timeout in seconds",
    )
    parser.add_argument(
        "--switch-timeout-sec",
        type=float,
        default=30.0,
        help="switch_controller timeout in seconds",
    )
    args, _ = parser.parse_known_args()
    return args


def main() -> int:
    args = parse_args()
    business_controllers = list(args.business_controller)
    if not business_controllers:
        raise RuntimeError("至少需要一个 --business-controller")

    rclpy.init()
    node = None
    exit_code = 0

    try:
        node = ControllerStateConverger(
            controller_manager_ns=args.controller_manager,
            joint_state_broadcaster=args.joint_state_broadcaster,
            target_controller=args.target_controller,
            business_controllers=business_controllers,
            always_active_controllers=list(args.always_active_controller),
            timeout_sec=args.timeout_sec,
            switch_timeout_sec=args.switch_timeout_sec,
        )
        node.converge()
    except KeyboardInterrupt:
        if node is not None:
            print("[INFO] 收到中断信号，停止控制器收敛。", file=sys.stderr)
        exit_code = 130
    except Exception as exc:  # pylint: disable=broad-except
        if node is not None:
            safe_log(node, "error", f"控制器收敛失败: {exc}")
        else:
            print(f"[ERROR] 控制器收敛失败: {exc}")
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

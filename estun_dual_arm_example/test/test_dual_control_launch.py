import importlib.util
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch_ros.actions import Node

TOOLS_DIR = Path(__file__).resolve().parents[2] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from estun_test_env import ensure_process_ros_test_env  # noqa: E402


ensure_process_ros_test_env("estun_dual_arm_example_test")


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class FakeLaunchConfiguration:
    defaults = {}

    def __init__(self, name):
        self.name = name

    def perform(self, _context):
        return self.defaults[self.name]


def _include_launch_arguments(include_action):
    return {
        key: _flatten_launch_value(value)
        for key, value in include_action.launch_arguments
    }


def _flatten_to_text(value):
    flattened = _flatten_launch_value(value)
    if isinstance(flattened, list):
        return "".join(str(item) for item in flattened)
    return str(flattened)


def _flatten_launch_value(value):
    if isinstance(value, tuple):
        flattened_items = [_flatten_launch_value(item) for item in value]
        if all(isinstance(item, str) for item in flattened_items):
            return "".join(flattened_items)
        if len(flattened_items) == 1:
            return flattened_items[0]
        return tuple(flattened_items)
    if isinstance(value, list):
        return [_flatten_launch_value(item) for item in value]
    text = getattr(value, "text", None)
    if text is not None:
        return text.replace("\n...\n", "")
    defaults = getattr(value.__class__, "defaults", None)
    name = getattr(value, "name", None)
    if isinstance(defaults, dict) and name in defaults:
        return defaults[name]
    describe = getattr(value, "describe", None)
    if callable(describe):
        return describe()
    return value


def _node_parameter_dicts(node):
    parameter_groups = getattr(node, "_Node__parameters")
    rewritten = []
    for group in parameter_groups:
        rewritten.append(
            {
                _flatten_launch_value(key): _flatten_launch_value(value)
                for key, value in group.items()
            }
        )
    return rewritten


def _load_launch_module():
    launch_path = PACKAGE_ROOT / "launch" / "dual_control.launch.py"
    spec = importlib.util.spec_from_file_location("dual_control_launch", launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 launch 文件: {launch_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_named_launch_module(file_name):
    launch_path = PACKAGE_ROOT / "launch" / file_name
    spec = importlib.util.spec_from_file_location(file_name.replace(".", "_"), launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 launch 文件: {launch_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_dual_control_launch_declares_two_independent_arms():
    dual_launch = _load_launch_module()
    dual_launch.get_package_share_directory = lambda package_name: f"/tmp/{package_name}"

    description = dual_launch.generate_launch_description()
    declared_names = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    include_actions = [
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]

    assert len(include_actions) == 2
    assert {
        "rarm_namespace",
        "larm_namespace",
        "rarm_prefix",
        "larm_prefix",
        "rarm_model",
        "larm_model",
        "rarm_ip",
        "larm_ip",
    }.issubset(declared_names)

    declared_arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert _flatten_to_text(declared_arguments["rarm_model"].default_value) == "iER7-910-MI"
    assert _flatten_to_text(declared_arguments["larm_model"].default_value) == "iER7-910-MI"
    assert _flatten_to_text(declared_arguments["rarm_ip"].default_value) == ""
    assert _flatten_to_text(declared_arguments["larm_ip"].default_value) == ""


def test_dual_arm_srdf_defines_expected_groups():
    srdf_root = ET.parse(PACKAGE_ROOT / "srdf" / "dual_arm.srdf").getroot()
    groups = {group.get("name"): group for group in srdf_root.findall("group")}

    assert set(groups) == {"rarm", "larm", "dual_arm"}
    assert groups["rarm"].find("chain").get("base_link") == "rarm_base_link"
    assert groups["rarm"].find("chain").get("tip_link") == "rarm_flange"
    assert groups["larm"].find("chain").get("base_link") == "larm_base_link"
    assert groups["larm"].find("chain").get("tip_link") == "larm_flange"
    assert {
        subgroup.get("name")
        for subgroup in groups["dual_arm"].findall("group")
    } == {"rarm", "larm"}


def test_dual_arm_urdf_expands_without_duplicate_names():
    urdf_path = PACKAGE_ROOT / "urdf" / "dual_arm.urdf.xacro"
    xacro_root = ET.parse(urdf_path).getroot()
    xacro_args = {
        arg.get("name"): arg.get("default")
        for arg in xacro_root.findall("{http://wiki.ros.org/xacro}arg")
    }
    result = subprocess.run(
        ["xacro", str(urdf_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    urdf_root = ET.fromstring(result.stdout)

    link_names = [link.get("name") for link in urdf_root.findall("link")]
    joint_names = [joint.get("name") for joint in urdf_root.findall("joint")]

    assert len(link_names) == len(set(link_names))
    assert len(joint_names) == len(set(joint_names))
    assert "rarm_ee_link" in link_names
    assert "larm_ee_link" in link_names
    assert "rarm_joint_6" in joint_names
    assert "larm_joint_6" in joint_names
    assert len([name for name in joint_names if name.endswith("joint_6")]) == 2

    joints = {joint.get("name"): joint for joint in urdf_root.findall("joint")}
    rarm_origin = joints["rarm_base_joint"].find("origin")
    larm_origin = joints["larm_base_joint"].find("origin")
    assert joints["rarm_base_joint"].find("parent").get("link") == "world"
    assert joints["larm_base_joint"].find("parent").get("link") == "world"
    assert rarm_origin.get("xyz") == " ".join(
        xacro_args[name]
        for name in ("rarm_origin_x", "rarm_origin_y", "rarm_origin_z")
    )
    assert rarm_origin.get("rpy") == " ".join(
        xacro_args[name]
        for name in ("rarm_origin_rr", "rarm_origin_rp", "rarm_origin_ry")
    )
    assert larm_origin.get("xyz") == " ".join(
        xacro_args[name]
        for name in ("larm_origin_x", "larm_origin_y", "larm_origin_z")
    )
    assert larm_origin.get("rpy") == " ".join(
        xacro_args[name]
        for name in ("larm_origin_rr", "larm_origin_rp", "larm_origin_ry")
    )
    assert xacro_args["rarm_model"] == "iER7-910-MI"
    assert xacro_args["larm_model"] == "iER7-910-MI"


def test_dual_arm_urdf_supports_mixed_models():
    urdf_path = PACKAGE_ROOT / "urdf" / "dual_arm.urdf.xacro"
    result = subprocess.run(
        [
            "xacro",
            str(urdf_path),
            "rarm_model:=ER20-1780-A6",
            "larm_model:=iER7-910-MI",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "package://estun_description/meshes/ER20-1780-A6/visual/base_link.dae" in result.stdout
    assert "package://estun_description/meshes/iER7-910-MI/visual/base_link.dae" in result.stdout


def test_dual_moveit_launch_declares_control_and_rviz_switches():
    dual_moveit_launch = _load_named_launch_module("dual_moveit.launch.py")

    description = dual_moveit_launch.generate_launch_description()
    declared_names = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    opaque_actions = [
        action for action in description.entities if isinstance(action, OpaqueFunction)
    ]

    assert len(opaque_actions) == 1
    assert {
        "rarm_namespace",
        "larm_namespace",
        "rarm_prefix",
        "larm_prefix",
        "rarm_model",
        "larm_model",
        "rarm_ip",
        "larm_ip",
        "start_control",
        "start_rviz",
    }.issubset(declared_names)

    declared_arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert _flatten_to_text(declared_arguments["rarm_model"].default_value) == "iER7-910-MI"
    assert _flatten_to_text(declared_arguments["larm_model"].default_value) == "iER7-910-MI"
    assert _flatten_to_text(declared_arguments["rarm_ip"].default_value) == ""
    assert _flatten_to_text(declared_arguments["larm_ip"].default_value) == ""


def test_dual_moveit_config_uses_ros2_control_multi_manager():
    moveit_controllers = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "moveit_controllers.yaml").read_text()
    )
    kinematics = yaml.safe_load((PACKAGE_ROOT / "config" / "kinematics.yaml").read_text())
    ompl = yaml.safe_load((PACKAGE_ROOT / "config" / "ompl_planning.yaml").read_text())

    assert (
        moveit_controllers["moveit_controller_manager"]
        == "moveit_ros_control_interface/Ros2ControlMultiManager"
    )
    assert moveit_controllers["moveit_manage_controllers"] is False
    assert set(kinematics) == {"rarm", "larm"}
    assert {"rarm", "larm", "dual_arm"}.issubset(ompl["ompl"])


def test_dual_moveit_launch_rewrites_prefixed_moveit_config():
    dual_moveit_launch = _load_named_launch_module("dual_moveit.launch.py")

    srdf_text = dual_moveit_launch.rewrite_prefixed_text(
        (PACKAGE_ROOT / "srdf" / "dual_arm.srdf").read_text(),
        "right_",
        "left_",
    )
    srdf_root = ET.fromstring(srdf_text)
    groups = {group.get("name"): group for group in srdf_root.findall("group")}

    assert groups["rarm"].find("chain").get("base_link") == "right_base_link"
    assert groups["rarm"].find("chain").get("tip_link") == "right_flange"
    assert groups["larm"].find("chain").get("base_link") == "left_base_link"
    assert groups["larm"].find("chain").get("tip_link") == "left_flange"

    joint_limits = dual_moveit_launch.merge_dual_arm_joint_limits(
        yaml.safe_load(
            (
                PACKAGE_ROOT.parents[1]
                / "estun_description"
                / "config"
                / "ER20-1780-A6"
                / "joint_limits.yaml"
            ).read_text()
        ),
        yaml.safe_load(
            (
                PACKAGE_ROOT.parents[1]
                / "estun_description"
                / "config"
                / "iER7-910-MI"
                / "joint_limits.yaml"
            ).read_text()
        ),
        "right_",
        "left_",
    )
    assert "right_joint_6" in joint_limits["joint_limits"]
    assert "left_joint_6" in joint_limits["joint_limits"]
    assert "rarm_joint_6" not in joint_limits["joint_limits"]
    assert "larm_joint_6" not in joint_limits["joint_limits"]
    assert joint_limits["joint_limits"]["right_joint_1"]["max_velocity"] == 3.2288591161895095
    assert joint_limits["joint_limits"]["left_joint_1"]["max_velocity"] == 5.84685299418


def test_dual_moveit_launch_setup_passes_custom_prefix_and_joint_state_topics(monkeypatch):
    dual_moveit_launch = _load_named_launch_module("dual_moveit.launch.py")
    monkeypatch.setattr(
        dual_moveit_launch,
        "get_package_share_directory",
        lambda package_name: str(PACKAGE_ROOT)
        if package_name == "estun_dual_arm_example"
        else str(PACKAGE_ROOT.parents[1] / "estun_description"),
    )

    FakeLaunchConfiguration.defaults = {
        "rarm_namespace": "right_arm_ns",
        "larm_namespace": "left_arm_ns",
        "rarm_prefix": "right_",
        "larm_prefix": "left_",
        "rarm_model": "ER20-1780-A6",
        "larm_model": "iER7-910-MI",
        "rarm_ip": "",
        "larm_ip": "",
        "rarm_cmd_port": "61210",
        "larm_cmd_port": "61210",
        "rarm_servo_port": "61211",
        "larm_servo_port": "61211",
        "rarm_status_port": "61212",
        "larm_status_port": "61212",
        "rarm_use_fake_hardware": "true",
        "larm_use_fake_hardware": "true",
        "motion_period_ms": "4",
        "control_node_impl": "auto",
        "use_logical_control_time": "true",
        "enable_control_loop_diag_log": "false",
        "controllers_file": "config/estun_6dof_controllers.yaml",
        "start_control": "true",
        "start_rviz": "false",
    }
    monkeypatch.setattr(dual_moveit_launch, "LaunchConfiguration", FakeLaunchConfiguration)

    actions = dual_moveit_launch.launch_setup(object())
    node_actions = [action for action in actions if isinstance(action, Node)]
    nodes_by_package = {node.node_package: node for node in node_actions}

    assert not any(isinstance(action, RegisterEventHandler) for action in actions)
    merger_node = nodes_by_package["estun_dual_arm_example"]

    assert _node_parameter_dicts(merger_node) == [
        {
            "rarm_joint_states_topic": "/right_arm_ns/joint_states",
            "larm_joint_states_topic": "/left_arm_ns/joint_states",
        }
    ]
    include_action = next(
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    )
    include_args = _include_launch_arguments(include_action)
    assert include_args["rarm_model"] == "ER20-1780-A6"
    assert include_args["larm_model"] == "iER7-910-MI"
    assert set(nodes_by_package) == {
        "estun_dual_arm_example",
        "moveit_ros_move_group",
    }


def test_dual_moveit_launch_setup_builds_expected_nodes(monkeypatch):
    dual_moveit_launch = _load_named_launch_module("dual_moveit.launch.py")
    monkeypatch.setattr(
        dual_moveit_launch,
        "get_package_share_directory",
        lambda package_name: str(PACKAGE_ROOT)
        if package_name == "estun_dual_arm_example"
        else str(PACKAGE_ROOT.parents[1] / "estun_description"),
    )

    context = object()
    FakeLaunchConfiguration.defaults = {
        "rarm_namespace": "rarm",
        "larm_namespace": "larm",
        "rarm_prefix": "rarm_",
        "larm_prefix": "larm_",
        "rarm_model": "ER20-1780-A6",
        "larm_model": "iER7-910-MI",
        "rarm_ip": "",
        "larm_ip": "",
        "rarm_cmd_port": "61210",
        "larm_cmd_port": "61210",
        "rarm_servo_port": "61211",
        "larm_servo_port": "61211",
        "rarm_status_port": "61212",
        "larm_status_port": "61212",
        "rarm_use_fake_hardware": "true",
        "larm_use_fake_hardware": "true",
        "motion_period_ms": "4",
        "control_node_impl": "auto",
        "use_logical_control_time": "true",
        "enable_control_loop_diag_log": "false",
        "controllers_file": "config/estun_6dof_controllers.yaml",
        "start_control": "true",
        "start_rviz": "true",
    }

    monkeypatch.setattr(dual_moveit_launch, "LaunchConfiguration", FakeLaunchConfiguration)

    actions = dual_moveit_launch.launch_setup(context)
    include_actions = [
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    ]
    node_actions = [action for action in actions if isinstance(action, Node)]
    event_handlers = [
        action for action in actions if isinstance(action, RegisterEventHandler)
    ]

    assert len(include_actions) == 1
    assert len(node_actions) == 3
    assert len(event_handlers) == 1
    assert {node.node_package for node in node_actions} == {
        "estun_dual_arm_example",
        "moveit_ros_move_group",
        "rviz2",
    }
    rviz_node = next(node for node in node_actions if node.node_package == "rviz2")
    event_handler = event_handlers[0].event_handler
    assert isinstance(event_handler, OnProcessExit)
    rviz_exit = ProcessExited(
        action=rviz_node,
        name="rviz2_dual_arm",
        cmd=[],
        cwd=None,
        env=None,
        pid=1,
        returncode=0,
    )
    assert event_handler.matches(rviz_exit)
    on_exit_actions = event_handler.describe()[1]
    assert len(on_exit_actions) == 1
    assert isinstance(on_exit_actions[0], EmitEvent)
    assert isinstance(on_exit_actions[0].event, Shutdown)

    merger_node = next(
        node for node in node_actions if node.node_package == "estun_dual_arm_example"
    )
    move_group_node = next(
        node for node in node_actions if node.node_package == "moveit_ros_move_group"
    )
    assert _node_parameter_dicts(merger_node) == [
        {
            "rarm_joint_states_topic": "/rarm/joint_states",
            "larm_joint_states_topic": "/larm/joint_states",
        }
    ]
    include_args = _include_launch_arguments(include_actions[0])
    assert include_args["rarm_model"] == "ER20-1780-A6"
    assert include_args["larm_model"] == "iER7-910-MI"
    move_group_params = _node_parameter_dicts(move_group_node)
    robot_description = _flatten_to_text(move_group_params[0]["robot_description"])
    assert "rarm_model:=" in robot_description
    assert "ER20-1780-A6" in robot_description
    assert "larm_model:=" in robot_description
    assert "iER7-910-MI" in robot_description
    planning = move_group_params[3]
    assert (
        planning["robot_description_planning.joint_limits.rarm_joint_1.max_velocity"]
        == 3.2288591161895095
    )
    assert (
        planning["robot_description_planning.joint_limits.larm_joint_1.max_velocity"]
        == 5.84685299418
    )

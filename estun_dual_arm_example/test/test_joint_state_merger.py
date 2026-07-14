import importlib.util
from pathlib import Path
import sys

import pytest
import rclpy
from sensor_msgs.msg import JointState

TOOLS_DIR = Path(__file__).resolve().parents[2] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from estun_test_env import ensure_process_ros_test_env  # noqa: E402


ensure_process_ros_test_env("estun_dual_arm_joint_state_merger_test")

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "joint_state_merger.py"


def _load_joint_state_merger_module():
    spec = importlib.util.spec_from_file_location(
        "joint_state_merger", SCRIPT_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载脚本: {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MERGER_MODULE = _load_joint_state_merger_module()


def _make_joint_state(names, positions, velocities=None, efforts=None):
    msg = JointState()
    msg.name = list(names)
    msg.position = list(positions)
    msg.velocity = [] if velocities is None else list(velocities)
    msg.effort = [] if efforts is None else list(efforts)
    return msg


class _PublisherStub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True
    else:
        initialized_here = False
    yield
    if initialized_here and rclpy.ok():
        rclpy.shutdown()


def test_valid_joint_state_accepts_empty_optional_fields():
    msg = _make_joint_state(
        names=["rarm_joint_1", "rarm_joint_2"],
        positions=[0.1, 0.2],
        velocities=[],
        efforts=[1.0, 2.0],
    )

    assert MERGER_MODULE.is_valid_joint_state(msg) is True


def test_is_valid_joint_state_rejects_mismatched_lengths():
    invalid_position = _make_joint_state(
        names=["joint_1", "joint_2"],
        positions=[0.1],
        velocities=[0.0, 0.0],
        efforts=[0.0, 0.0],
    )
    invalid_velocity = _make_joint_state(
        names=["joint_1", "joint_2"],
        positions=[0.1, 0.2],
        velocities=[0.0],
        efforts=[0.0, 0.0],
    )
    empty_state = _make_joint_state(names=[], positions=[])

    assert MERGER_MODULE.is_valid_joint_state(invalid_position) is False
    assert MERGER_MODULE.is_valid_joint_state(invalid_velocity) is False
    assert MERGER_MODULE.is_valid_joint_state(empty_state) is False


def test_merge_joint_states_drops_incomplete_optional_fields():
    rarm_msg = _make_joint_state(
        names=["rarm_joint_1", "rarm_joint_2"],
        positions=[0.1, 0.2],
        velocities=[1.0, 2.0],
        efforts=[3.0, 4.0],
    )
    larm_msg = _make_joint_state(
        names=["larm_joint_1", "larm_joint_2"],
        positions=[0.3, 0.4],
        velocities=[],
        efforts=[5.0, 6.0],
    )

    merged = MERGER_MODULE.merge_joint_states([rarm_msg, larm_msg])

    assert merged is not None
    assert merged.name == [
        "rarm_joint_1",
        "rarm_joint_2",
        "larm_joint_1",
        "larm_joint_2",
    ]
    assert list(merged.position) == [0.1, 0.2, 0.3, 0.4]
    assert list(merged.velocity) == []
    assert list(merged.effort) == [3.0, 4.0, 5.0, 6.0]


def test_merger_waits_until_both_arms_ready():
    node = MERGER_MODULE.JointStateMerger()
    node._publisher = _PublisherStub()
    try:
        node._store_rarm(
            _make_joint_state(
                names=["rarm_joint_1", "rarm_joint_2"],
                positions=[0.1, 0.2],
                velocities=[1.0, 2.0],
                efforts=[3.0, 4.0],
            )
        )

        node._publish_merged()

        assert node._publisher.messages == []
    finally:
        node.destroy_node()


def test_merger_keeps_last_valid_arm_cache():
    node = MERGER_MODULE.JointStateMerger()
    node._publisher = _PublisherStub()
    try:
        node._store_rarm(
            _make_joint_state(
                names=["rarm_joint_1", "rarm_joint_2"],
                positions=[0.1, 0.2],
                velocities=[1.0, 2.0],
                efforts=[3.0, 4.0],
            )
        )
        node._store_larm(
            _make_joint_state(
                names=["larm_joint_1", "larm_joint_2"],
                positions=[0.3, 0.4],
                velocities=[],
                efforts=[5.0, 6.0],
            )
        )
        node._publish_merged()

        first_msg = node._publisher.messages[-1]
        assert first_msg.name == [
            "rarm_joint_1",
            "rarm_joint_2",
            "larm_joint_1",
            "larm_joint_2",
        ]
        assert list(first_msg.position) == [0.1, 0.2, 0.3, 0.4]
        assert list(first_msg.velocity) == []

        node._store_larm(
            _make_joint_state(
                names=["larm_joint_1", "larm_joint_2"],
                positions=[0.3],
                velocities=[7.0, 8.0],
                efforts=[5.0, 6.0],
            )
        )
        node._publish_merged()

        second_msg = node._publisher.messages[-1]
        assert second_msg.name == [
            "rarm_joint_1",
            "rarm_joint_2",
            "larm_joint_1",
            "larm_joint_2",
        ]
        assert list(second_msg.position) == [0.1, 0.2, 0.3, 0.4]
        assert list(second_msg.velocity) == []
        assert list(second_msg.effort) == [3.0, 4.0, 5.0, 6.0]
    finally:
        node.destroy_node()

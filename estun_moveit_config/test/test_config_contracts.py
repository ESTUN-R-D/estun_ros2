from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
JOINTS = [f"joint_{index}" for index in range(1, 7)]
CARTESIAN_INTERFACES = ["x", "y", "z", "a", "b", "c"]


def _load_yaml(relative_path: str):
    with (REPO_ROOT / relative_path).open("r") as file:
        return yaml.safe_load(file)


def test_ros2_controllers_expose_expected_control_modes():
    controllers = _load_yaml("estun_hardware/config/estun_6dof_controllers.yaml")
    manager_params = controllers["controller_manager"]["ros__parameters"]

    assert manager_params["estun_arm_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert manager_params["estun_state_broadcaster"]["type"] == (
        "estun_controllers/EstunStateBroadcaster"
    )
    assert manager_params["estun_do_controller"]["type"] == (
        "estun_controllers/EstunDOController"
    )
    assert manager_params["forward_position_controller"]["type"] == (
        "forward_command_controller/ForwardCommandController"
    )
    assert manager_params["cartesian_forward_controller"]["type"] == (
        "forward_command_controller/MultiInterfaceForwardCommandController"
    )

    arm_params = controllers["estun_arm_controller"]["ros__parameters"]
    assert arm_params["joints"] == JOINTS
    assert arm_params["command_interfaces"] == ["position"]
    assert arm_params["state_interfaces"] == ["position"]
    assert arm_params["allow_partial_joints_goal"] is False

    forward_params = controllers["forward_position_controller"]["ros__parameters"]
    assert forward_params["joints"] == JOINTS
    assert forward_params["interface_name"] == "position"

    cart_params = controllers["cartesian_forward_controller"]["ros__parameters"]
    assert cart_params["joint"] == "cartesian_tcp"
    assert cart_params["interface_names"] == CARTESIAN_INTERFACES

    status_params = controllers["estun_state_broadcaster"]["ros__parameters"]
    assert status_params["prefix"] == ""
    assert status_params["state_publish_rate"] > 0.0

    do_params = controllers["estun_do_controller"]["ros__parameters"]
    assert do_params["prefix"] == ""
    assert do_params["sdk_namespace"] == "/estun"
    assert do_params["service_timeout_ms"] >= 3000


def test_moveit_controller_matches_ros2_trajectory_controller():
    controllers = _load_yaml("estun_hardware/config/estun_6dof_controllers.yaml")
    moveit_controllers = _load_yaml("estun_moveit_config/config/moveit_controllers.yaml")

    moveit_manager = moveit_controllers["moveit_simple_controller_manager"]
    assert moveit_manager["controller_names"] == ["estun_arm_controller"]

    moveit_arm = moveit_manager["estun_arm_controller"]
    ros2_arm = controllers["estun_arm_controller"]["ros__parameters"]
    assert moveit_arm["type"] == "FollowJointTrajectory"
    assert moveit_arm["action_ns"] == "follow_joint_trajectory"
    assert moveit_arm["default"] is True
    assert moveit_arm["joints"] == ros2_arm["joints"]


def test_servo_config_outputs_only_to_apos_forward_controller():
    servo = _load_yaml("estun_moveit_config/config/servo.yaml")

    assert servo["move_group_name"] == "estun_arm"
    assert servo["planning_frame"] == "base_link"
    assert servo["ee_frame_name"] == "flange"
    assert servo["robot_link_command_frame"] == "flange"
    assert servo["command_out_type"] == "std_msgs/Float64MultiArray"
    assert servo["command_out_topic"] == "/forward_position_controller/commands"
    assert servo["publish_joint_positions"] is True
    assert servo["publish_joint_velocities"] is False
    assert servo["publish_joint_accelerations"] is False
    assert servo["publish_period"] > 0.0


def test_planning_configs_keep_estun_arm_group_consistent():
    ompl = _load_yaml("estun_moveit_config/config/ompl_planning.yaml")
    pilz = _load_yaml("estun_moveit_config/config/pilz_planning.yaml")

    assert "estun_arm" in ompl["ompl"]
    assert "arm" not in ompl["ompl"]
    assert ompl["ompl"]["estun_arm"]["default_planner_config"] == "RRTConnectkConfigDefault"
    assert "RRTConnectkConfigDefault" in ompl["ompl"]["estun_arm"]["planner_configs"]

    assert (
        pilz["pilz_industrial_motion_planner"]["planning_plugin"]
        == "pilz_industrial_motion_planner/CommandPlanner"
    )
    assert pilz["pilz_industrial_motion_planner"]["start_state_max_bounds_error"] > 0.0


def test_pilz_cartesian_limits_are_directionally_valid_and_complete():
    limits = _load_yaml("estun_moveit_config/config/pilz_cartesian_limits.yaml")
    cart = limits["cartesian_limits"]

    required_positive_fields = [
        "max_trans_vel",
        "max_trans_acc",
        "max_rot_vel",
    ]
    for field in required_positive_fields:
        assert cart[field] > 0.0

    assert cart["max_trans_dec"] < 0.0

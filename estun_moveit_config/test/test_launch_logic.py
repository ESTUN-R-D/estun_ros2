import importlib.util
from pathlib import Path

import pytest


def _load_launch_module(package_name: str, file_name: str):
    launch_path = Path(__file__).resolve().parents[2] / package_name / "launch" / file_name
    spec = importlib.util.spec_from_file_location(file_name.replace(".", "_"), launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 launch 文件: {launch_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_control_mode_is_strict_three_values():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    for raw in ("plan", "apos", "cpos", "PLAN", " Apos "):
        assert control_launch.resolve_control_mode(raw)["control_mode"] in ("plan", "apos", "cpos")
        assert demo_launch.resolve_control_mode(raw)["control_mode"] in ("plan", "apos", "cpos")

    assert control_launch.resolve_control_mode("plan")["stream_policy"] == "fifo"
    assert control_launch.resolve_control_mode("apos")["stream_policy"] == "latest_overwrite"
    assert control_launch.resolve_control_mode("cpos")["stream_policy"] == "latest_overwrite"
    assert demo_launch.resolve_control_mode("plan")["stream_policy"] == "fifo"
    assert demo_launch.resolve_control_mode("apos")["stream_policy"] == "latest_overwrite"
    assert demo_launch.resolve_control_mode("cpos")["stream_policy"] == "latest_overwrite"

    with pytest.raises(RuntimeError):
        control_launch.resolve_control_mode("invalid")

    with pytest.raises(RuntimeError):
        demo_launch.resolve_control_mode("servo_mode")


def test_demo_launch_keeps_three_control_modes():
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")
    for mode in ("plan", "apos", "cpos"):
        mode_cfg = demo_launch.resolve_control_mode(mode)
        assert mode_cfg["control_mode"] == mode
    assert demo_launch.parse_auto_bool("true", False) is True
    assert demo_launch.parse_auto_bool("false", True) is False
    assert demo_launch.parse_non_negative_float("4.5", "delay") == 4.5

    with pytest.raises(RuntimeError):
        demo_launch.parse_auto_bool("not_bool", True)

    with pytest.raises(RuntimeError):
        demo_launch.parse_non_negative_float("-1", "delay")


def test_demo_launch_delays_moveit_only_for_real_estun_control_node():
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    assert demo_launch.should_delay_moveit_start("false", "auto") is True
    assert demo_launch.should_delay_moveit_start("false", "estun") is True
    assert demo_launch.should_delay_moveit_start("true", "auto") is False
    assert demo_launch.should_delay_moveit_start("false", "official") is False


def test_robot_ip_validation_requires_explicit_value_for_real_hardware():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    assert control_launch.validate_robot_ip("", True) == ""
    assert demo_launch.validate_robot_ip("", "true") == ""
    assert control_launch.validate_robot_ip(" 127.0.0.1 ", False) == "127.0.0.1"
    assert demo_launch.validate_robot_ip(" 127.0.0.1 ", "false") == "127.0.0.1"

    with pytest.raises(RuntimeError):
        control_launch.validate_robot_ip("", False)

    with pytest.raises(RuntimeError):
        demo_launch.validate_robot_ip(" ", "false")


def test_planning_pipeline_parser_supports_alias_and_validation():
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    assert demo_launch.normalize_planning_pipeline("ompl") == "ompl"
    assert demo_launch.normalize_planning_pipeline("pilz") == "pilz_industrial_motion_planner"
    assert (
        demo_launch.normalize_planning_pipeline("pilz_industrial_motion_planner")
        == "pilz_industrial_motion_planner"
    )
    assert demo_launch.parse_planning_pipelines("ompl,pilz,ompl") == [
        "ompl",
        "pilz_industrial_motion_planner",
    ]

    with pytest.raises(RuntimeError):
        demo_launch.normalize_planning_pipeline("chomp")

    with pytest.raises(RuntimeError):
        demo_launch.parse_planning_pipelines(",,,")


def test_pilz_cartesian_limits_merge_and_validation():
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    base = {"joint_limits": {"joint_1": {"has_velocity_limits": True}}}
    pilz_limits = {"cartesian_limits": {"max_trans_vel": 1.0}}
    merged = demo_launch.with_pilz_cartesian_limits(base, pilz_limits)

    assert merged["cartesian_limits"]["max_trans_vel"] == 1.0
    assert "cartesian_limits" not in base

    with pytest.raises(RuntimeError):
        demo_launch.with_pilz_cartesian_limits(base, None)


def test_robot_cartesian_limits_loader_prefers_model_file_and_falls_back_to_global():
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    def fake_load_yaml(package_name, file_path):
        assert package_name in ("estun_description", "estun_moveit_config")
        if (
            package_name == "estun_description"
            and file_path == "config/ER20-1780-A6/cartesian_limits.yaml"
        ):
            return {"cartesian_limits": {"max_trans_vel": 4.0}}
        if file_path == "config/pilz_cartesian_limits.yaml":
            return {"cartesian_limits": {"max_trans_vel": 1.0}}
        return None

    demo_launch.load_yaml = fake_load_yaml

    model_limits = demo_launch.load_robot_cartesian_limits_yaml("ER20-1780-A6")
    fallback_limits = demo_launch.load_robot_cartesian_limits_yaml("unknown_model")

    assert model_limits["cartesian_limits"]["max_trans_vel"] == 4.0
    assert fallback_limits["cartesian_limits"]["max_trans_vel"] == 1.0


def test_default_pipeline_fallback_for_rviz_when_pilz_is_selected():
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    assert (
        demo_launch.resolve_effective_default_pipeline(
            "pilz_industrial_motion_planner",
            ["ompl", "pilz_industrial_motion_planner"],
            True,
        )
        == "ompl"
    )
    assert (
        demo_launch.resolve_effective_default_pipeline(
            "pilz_industrial_motion_planner",
            ["pilz_industrial_motion_planner"],
            False,
        )
        == "pilz_industrial_motion_planner"
    )
    assert (
        demo_launch.resolve_effective_default_pipeline(
            "ompl",
            ["ompl", "pilz_industrial_motion_planner"],
            True,
        )
        == "ompl"
    )

    with pytest.raises(RuntimeError):
        demo_launch.resolve_effective_default_pipeline(
            "pilz_industrial_motion_planner",
            ["pilz_industrial_motion_planner"],
            True,
        )


def test_servo_helpers_cover_runtime_override_and_srdf_chain_extraction():
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    assert demo_launch.parse_positive_int("8", "motion_period_ms") == 8
    with pytest.raises(RuntimeError):
        demo_launch.parse_positive_int("0", "motion_period_ms")

    srdf_text = """
<robot name="estun_test">
  <group name="estun_arm">
    <chain base_link="base_link" tip_link="flange"/>
  </group>
</robot>
"""
    base_link, tip_link = demo_launch.extract_group_chain_links(srdf_text, "estun_arm")
    assert base_link == "base_link"
    assert tip_link == "flange"

    raw_servo = {
        "move_group_name": "arm",
        "planning_frame": "world",
        "ee_frame_name": "tool0",
        "robot_link_command_frame": "tool0",
        "command_out_topic": "/old",
        "publish_period": 0.01,
    }
    patched = demo_launch.with_servo_runtime_overrides(
        raw_yaml=raw_servo,
        move_group_name="estun_arm",
        planning_frame="base_link",
        ee_frame="flange",
        command_out_topic="/forward_position_controller/commands",
        publish_period_sec=0.008,
    )
    assert patched["move_group_name"] == "estun_arm"
    assert patched["planning_frame"] == "base_link"
    assert patched["ee_frame_name"] == "flange"
    assert patched["robot_link_command_frame"] == "flange"
    assert patched["command_out_topic"] == "/forward_position_controller/commands"
    assert patched["publish_period"] == 0.008
    assert raw_servo["move_group_name"] == "arm"

    with pytest.raises(RuntimeError):
        demo_launch.with_servo_runtime_overrides(
            raw_yaml=None,
            move_group_name="estun_arm",
            planning_frame="base_link",
            ee_frame="flange",
            command_out_topic="/forward_position_controller/commands",
            publish_period_sec=0.008,
        )


def test_controllers_file_path_resolution_supports_relative_and_absolute():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    fake_share = "/tmp/fake_estun_hardware_share"
    control_launch.get_package_share_directory = lambda package_name: fake_share

    assert (
        control_launch.resolve_controllers_yaml_path("config/estun_6dof_controllers.yaml")
        == f"{fake_share}/config/estun_6dof_controllers.yaml"
    )

    assert (
        control_launch.resolve_controllers_yaml_path("/tmp/custom_controllers.yaml")
        == "/tmp/custom_controllers.yaml"
    )


def test_control_launch_namespace_helpers_keep_root_default():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    assert control_launch.normalize_namespace("") == ""
    assert control_launch.normalize_namespace(" /rarm/ ") == "rarm"
    assert control_launch.scoped_controller_manager("") == "/controller_manager"
    assert control_launch.scoped_controller_manager("rarm") == "/rarm/controller_manager"
    assert control_launch.scoped_controller_manager("/larm/") == "/larm/controller_manager"


def test_runtime_tempfile_cleanup_helpers_remove_generated_yaml():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    temp_a = control_launch.dump_yaml_to_tempfile({"demo": {"value": 1}}, "estun_test_a_")
    temp_b = control_launch.dump_yaml_to_tempfile({"demo": {"value": 2}}, "estun_test_b_")

    assert Path(temp_a).is_file()
    assert Path(temp_b).is_file()

    control_launch.cleanup_tempfiles_on_shutdown([temp_a, temp_b])

    assert not Path(temp_a).exists()
    assert not Path(temp_b).exists()
    control_launch.cleanup_tempfiles([temp_a, temp_b])


def test_motion_period_parsing_and_update_rate_mapping():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    assert control_launch.parse_motion_period_ms("8") == 8
    assert control_launch.parse_motion_period_ms(" 4 ") == 4

    for raw in ("0", "-1", "abc", "", None):
        with pytest.raises(RuntimeError):
            control_launch.parse_motion_period_ms(raw)

    base_yaml = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 125,
            }
        }
    }
    patched_4ms = control_launch.with_motion_period_update_rate(base_yaml, 4)
    patched_7ms = control_launch.with_motion_period_update_rate(base_yaml, 7)
    assert patched_4ms["controller_manager"]["ros__parameters"]["update_rate"] == 250
    assert patched_7ms["controller_manager"]["ros__parameters"]["update_rate"] == 143
    assert base_yaml["controller_manager"]["ros__parameters"]["update_rate"] == 125
    assert control_launch.with_motion_period_update_rate(None, 8) is None

    wildcard_yaml = {
        "/**": {
            "ros__parameters": {
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster",
                }
            }
        }
    }
    patched_namespaced = control_launch.with_motion_period_update_rate(
        wildcard_yaml, 4, "rarm"
    )
    assert patched_namespaced["/**"]["ros__parameters"]["joint_state_broadcaster"][
        "type"
    ] == "joint_state_broadcaster/JointStateBroadcaster"
    assert (
        patched_namespaced["/rarm/controller_manager"]["ros__parameters"]["update_rate"]
        == 250
    )


def test_controller_yaml_namespace_rewrite_targets_scoped_nodes():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    raw_yaml = {
        "controller_manager": {
            "ros__parameters": {
                "cartesian_forward_controller": {
                    "type": "forward_command_controller/MultiInterfaceForwardCommandController",
                },
            },
        },
        "cartesian_forward_controller": {
            "ros__parameters": {
                "joint": "rarm_cartesian_tcp",
            },
        },
    }

    patched = control_launch.with_node_namespace(raw_yaml, " /rarm/ ")

    assert set(patched) == {
        "/rarm/controller_manager",
        "/rarm/cartesian_forward_controller",
    }
    assert (
        patched["/rarm/controller_manager"]["ros__parameters"][
            "cartesian_forward_controller"
        ]["type"]
        == "forward_command_controller/MultiInterfaceForwardCommandController"
    )
    assert patched["/rarm/cartesian_forward_controller"]["ros__parameters"]["joint"] == (
        "rarm_cartesian_tcp"
    )
    assert "controller_manager" in raw_yaml
    assert control_launch.with_node_namespace(raw_yaml, "") == raw_yaml


def test_controller_yaml_namespace_rewrite_preserves_absolute_and_global_keys():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    raw_yaml = {
        "/**": {
            "ros__parameters": {
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster",
                }
            }
        },
        "/**/forward_position_controller": {
            "ros__parameters": {
                "joints": ["joint_1"],
            }
        },
        "/rarm/controller_manager": {
            "ros__parameters": {
                "forward_position_controller": {
                    "type": "forward_command_controller/ForwardCommandController",
                }
            }
        },
    }

    patched = control_launch.with_node_namespace(raw_yaml, "rarm")

    assert set(patched) == {
        "/**",
        "/**/forward_position_controller",
        "/rarm/controller_manager",
    }
    assert patched["/**/forward_position_controller"]["ros__parameters"]["joints"] == [
        "joint_1"
    ]


def test_controller_yaml_namespace_conflict_and_mismatch_fail_fast():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    with pytest.raises(RuntimeError):
        control_launch.with_node_namespace(
            {
                "controller_manager": {"ros__parameters": {}},
                "/rarm/controller_manager": {"ros__parameters": {}},
            },
            "rarm",
        )

    with pytest.raises(RuntimeError):
        control_launch.with_node_namespace(
            {
                "/larm/controller_manager": {"ros__parameters": {}},
            },
            "rarm",
        )


def test_control_node_impl_resolution_and_logical_time_bool():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    assert control_launch.resolve_control_node_impl("auto", True) == "official"
    assert control_launch.resolve_control_node_impl("auto", False) == "estun"
    assert control_launch.resolve_control_node_impl("official", False) == "official"
    assert control_launch.resolve_control_node_impl("estun", True) == "estun"
    assert control_launch.resolve_control_node_impl(" ESTUN ", True) == "estun"

    with pytest.raises(RuntimeError):
        control_launch.resolve_control_node_impl("invalid_impl", False)

    assert control_launch.parse_bool("true", "use_logical_control_time") is True
    assert control_launch.parse_bool("False", "use_logical_control_time") is False
    assert control_launch.parse_bool("1", "use_fake_hardware") is True
    assert control_launch.parse_bool("0", "use_fake_hardware") is False
    with pytest.raises(RuntimeError):
        control_launch.parse_bool("not_bool", "use_logical_control_time")


def test_prefix_rewrite_covers_joint_and_cartesian_interfaces():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    raw_yaml = {
        "estun_arm_controller": {
            "ros__parameters": {
                "joints": ["joint_1", "r1_joint_2"],
            }
        },
        "forward_position_controller": {
            "ros__parameters": {
                "joints": ["joint_1", "joint_2"],
            }
        },
        "cartesian_forward_controller": {
            "ros__parameters": {
                "joint": "cartesian_tcp",
            }
        },
    }

    patched = control_launch.with_prefixed_controllers(raw_yaml, "r1_")
    assert patched["estun_arm_controller"]["ros__parameters"]["joints"] == [
        "r1_joint_1",
        "r1_joint_2",
    ]
    assert patched["forward_position_controller"]["ros__parameters"]["joints"] == [
        "r1_joint_1",
        "r1_joint_2",
    ]
    assert (
        patched["cartesian_forward_controller"]["ros__parameters"]["joint"]
        == "r1_cartesian_tcp"
    )
    # Ensure input YAML remains unchanged (function must deep-copy).
    assert raw_yaml["estun_arm_controller"]["ros__parameters"]["joints"] == [
        "joint_1",
        "r1_joint_2",
    ]


def test_prefix_rewrite_supports_global_and_absolute_controller_sections():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    raw_yaml = {
        "/**/forward_position_controller": {
            "ros__parameters": {
                "joints": ["joint_1", "joint_2"],
            }
        },
        "/rarm/cartesian_forward_controller": {
            "ros__parameters": {
                "joint": "cartesian_tcp",
            }
        },
        "/rarm/estun_do_controller": {
            "ros__parameters": {
                "prefix": "",
                "sdk_namespace": "/estun",
            }
        },
    }

    patched = control_launch.with_prefixed_controllers(raw_yaml, "r1_")
    assert patched["/**/forward_position_controller"]["ros__parameters"]["joints"] == [
        "r1_joint_1",
        "r1_joint_2",
    ]
    assert (
        patched["/rarm/cartesian_forward_controller"]["ros__parameters"]["joint"]
        == "r1_cartesian_tcp"
    )
    assert (
        patched["/rarm/estun_do_controller"]["ros__parameters"]["sdk_namespace"]
        == "/r1_estun"
    )


def test_runtime_controller_params_builds_compiled_yaml_and_update_rate_overlay():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")

    raw_yaml = {
        "controller_manager": {
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
                    "type": "forward_command_controller/MultiInterfaceForwardCommandController",
                },
            }
        },
        "forward_position_controller": {
            "ros__parameters": {
                "joints": ["joint_1", "joint_2"],
            }
        },
    }

    document, compiled_yaml, overrides_yaml = control_launch.build_runtime_controller_params(
        raw_yaml, "rarm", "r1_", 4
    )
    document.validate_required_controller_definitions(
        {
            "joint_state_broadcaster",
            "estun_arm_controller",
            "forward_position_controller",
            "cartesian_forward_controller",
        }
    )

    assert set(compiled_yaml) == {
        "/rarm/controller_manager",
        "/rarm/forward_position_controller",
    }
    assert compiled_yaml["/rarm/forward_position_controller"]["ros__parameters"][
        "joints"
    ] == ["r1_joint_1", "r1_joint_2"]
    assert overrides_yaml == {
        "/rarm/controller_manager": {
            "ros__parameters": {
                "update_rate": 250,
            }
        }
    }

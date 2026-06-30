import importlib.util
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]


def _load_launch_module(package_name: str, file_name: str):
    launch_path = REPO_ROOT / package_name / "launch" / file_name
    spec = importlib.util.spec_from_file_location(file_name.replace(".", "_"), launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 launch 文件: {launch_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_control_modes_keep_expected_runtime_policies():
    control_launch = _load_launch_module("estun_hardware", "estun_control.launch.py")
    demo_launch = _load_launch_module("estun_moveit_config", "demo.launch.py")

    expected = {
        "plan": {
            "hardware_servo_mode": "apos",
            "default_controller": "estun_arm_controller",
            "stream_policy": "fifo",
        },
        "apos": {
            "hardware_servo_mode": "apos",
            "default_controller": "forward_position_controller",
            "stream_policy": "latest_overwrite",
        },
        "cpos": {
            "hardware_servo_mode": "cpos",
            "default_controller": "cartesian_forward_controller",
            "stream_policy": "latest_overwrite",
        },
    }

    for mode, expected_cfg in expected.items():
        control_cfg = control_launch.resolve_control_mode(mode)
        assert control_cfg["hardware_servo_mode"] == expected_cfg["hardware_servo_mode"]
        assert control_cfg["default_controller"] == expected_cfg["default_controller"]
        assert control_cfg["stream_policy"] == expected_cfg["stream_policy"]

        demo_cfg = demo_launch.resolve_control_mode(mode)
        assert demo_cfg["hardware_servo_mode"] == expected_cfg["hardware_servo_mode"]
        assert demo_cfg["stream_policy"] == expected_cfg["stream_policy"]

    for raw_true in ("true", "1", "yes", "on"):
        assert demo_launch.parse_auto_bool(raw_true, False) is True

    for raw_false in ("false", "0", "no", "off"):
        assert demo_launch.parse_auto_bool(raw_false, True) is False

    with pytest.raises(RuntimeError):
        demo_launch.parse_auto_bool("maybe", True)


def test_ros2_control_xacro_exposes_expected_control_params():
    control_xacro = (
        REPO_ROOT
        / "estun_hardware"
        / "config"
        / "estun_6dof_control.ros2_control.xacro"
    ).read_text()

    required_params = {
        '<param name="servo_mode">${resolved_servo_mode}</param>',
        '<param name="stream_policy">${resolved_stream_policy}</param>',
        '<param name="stream_target_depth">5</param>',
    }
    for param in required_params:
        assert param in control_xacro
    assert 'stream_smoothing_enable' not in control_xacro
    assert 'stream_low_watermark' not in control_xacro
    assert 'stream_max_queue_depth' not in control_xacro
    assert 'stream_smoothing_depth_filter_alpha' not in control_xacro
    assert 'stream_smoothing_depth_gain' not in control_xacro
    assert 'stream_smoothing_min_phase_step' not in control_xacro
    assert 'stream_smoothing_max_phase_step' not in control_xacro
    assert (
        'params="name prefix robot_ip cmd_port servo_port status_port control_mode '
        'motion_period_ms robot_model use_mock"' in control_xacro
    )


def test_hardware_robot_xacros_require_explicit_control_mode():
    hardware_robot_dir = REPO_ROOT / "estun_hardware" / "robot"
    xacro_files = sorted(hardware_robot_dir.glob("*.urdf.xacro"))
    assert xacro_files, "未找到 estun_hardware 顶层机器人 xacro 文件"

    env = os.environ.copy()
    existing_pythonpath = env.get("PYTHONPATH", "")
    repo_src_path = str(REPO_ROOT)
    env["PYTHONPATH"] = (
        repo_src_path
        if not existing_pythonpath
        else f"{repo_src_path}:{existing_pythonpath}"
    )

    for xacro_file in xacro_files:
        result = subprocess.run(
            [
                "xacro",
                str(xacro_file),
                "use_mock:=true",
            ],
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode != 0, f"{xacro_file.name} 未传 control_mode 时不应解析成功"
        combined_output = result.stdout + result.stderr
        assert "control_mode 不能为空" in combined_output


def test_hardware_robot_xacros_do_not_embed_real_robot_ip_default():
    hardware_robot_dir = REPO_ROOT / "estun_hardware" / "robot"
    xacro_files = sorted(hardware_robot_dir.glob("*.urdf.xacro"))
    assert xacro_files, "未找到 estun_hardware 顶层机器人 xacro 文件"

    for xacro_file in xacro_files:
        text = xacro_file.read_text()
        assert '<xacro:arg name="robot_ip" default=""/>' in text
        assert "172.31.16.227" not in text


def test_hardware_robot_xacros_keep_control_mode_as_top_level_contract():
    hardware_robot_dir = REPO_ROOT / "estun_hardware" / "robot"
    xacro_files = sorted(hardware_robot_dir.glob("*.urdf.xacro"))
    assert xacro_files, "未找到 estun_hardware 顶层机器人 xacro 文件"

    for xacro_file in xacro_files:
        text = xacro_file.read_text()
        assert '<xacro:arg name="control_mode" default=""/>' in text
        assert 'arg name="servo_mode"' not in text
        assert 'arg name="stream_policy"' not in text
        assert 'control_mode="$(arg control_mode)"' in text
        assert 'servo_mode="$(arg servo_mode)"' not in text
        assert 'stream_policy="$(arg stream_policy)"' not in text


def test_hardware_robot_xacros_map_control_mode_to_expected_runtime_params():
    hardware_robot_dir = REPO_ROOT / "estun_hardware" / "robot"
    env = os.environ.copy()
    existing_pythonpath = env.get("PYTHONPATH", "")
    repo_src_path = str(REPO_ROOT)
    env["PYTHONPATH"] = (
        repo_src_path
        if not existing_pythonpath
        else f"{repo_src_path}:{existing_pythonpath}"
    )

    expected_by_mode = {
        "plan": ("apos", "fifo"),
        "apos": ("apos", "latest_overwrite"),
        "cpos": ("cpos", "latest_overwrite"),
    }

    for control_mode, expected in expected_by_mode.items():
        result = subprocess.run(
            [
                "xacro",
                str(hardware_robot_dir / "ER20-1780-A6.urdf.xacro"),
                "use_mock:=false",
                "control_mode:=" + control_mode,
            ],
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 0, (
            f"control_mode={control_mode} 解析失败。\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )

        root = ET.fromstring(result.stdout)
        ros2_control = root.find("ros2_control")
        assert ros2_control is not None
        hardware = ros2_control.find("hardware")
        assert hardware is not None

        runtime_params = {
            param.attrib["name"]: (param.text or "").strip()
            for param in hardware.findall("param")
        }
        assert runtime_params["servo_mode"] == expected[0]
        assert runtime_params["stream_policy"] == expected[1]

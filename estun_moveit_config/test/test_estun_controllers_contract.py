from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_estun_controllers_package_exports_expected_plugins():
    plugin_xml = (REPO_ROOT / "estun_controllers" / "estun_controllers.xml").read_text()
    cmake_text = (REPO_ROOT / "estun_controllers" / "CMakeLists.txt").read_text()

    export_call = (
        "pluginlib_export_plugin_description_file("
        "controller_interface estun_controllers.xml)"
    )
    assert "estun_controllers/EstunStateBroadcaster" in plugin_xml
    assert "estun_controllers/EstunDOController" in plugin_xml
    assert export_call in cmake_text


def test_hardware_exports_estun_status_state_interfaces():
    hardware_dir = REPO_ROOT / "estun_hardware"
    header_text = (
        hardware_dir / "include" / "estun_hardware" / "estun_hardware_interface.hpp"
    ).read_text()
    source_text = (hardware_dir / "src" / "estun_hardware_interface.cpp").read_text()

    for symbol in [
        "EstunStatusStateIndex",
        "estun_status_states_",
        "estun_status_interface_name_",
        "update_estun_status_state_interfaces",
    ]:
        assert symbol in header_text or symbol in source_text

    for interface_name in [
        "queue_underflow_count",
        "repeated_send_count",
        "servo_sdk_fail_count",
        "active_command_mode",
    ]:
        assert interface_name in source_text

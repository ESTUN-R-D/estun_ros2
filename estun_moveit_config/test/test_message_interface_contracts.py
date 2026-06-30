from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_estun_msgs_cmake_lists_all_public_interfaces():
    cmake_text = (REPO_ROOT / "estun_msgs" / "CMakeLists.txt").read_text()

    expected_interfaces = [
        "msg/ConnectionStatus.msg",
        "msg/EstunRobotStatus.msg",
        "srv/GetConnectionStatus.srv",
        "srv/GetCurErrMsg.srv",
        "srv/GetDo.srv",
        "srv/GetJointValue.srv",
        "srv/GetRobotConnStatus.srv",
        "srv/GetTool.srv",
        "srv/GetUser.srv",
        "srv/GetWorldCpos.srv",
        "srv/SetDo.srv",
    ]
    for interface in expected_interfaces:
        assert f'"{interface}"' in cmake_text


def test_connection_status_message_has_stable_fields():
    msg_text = (REPO_ROOT / "estun_msgs" / "msg" / "ConnectionStatus.msg").read_text()

    assert "builtin_interfaces/Time stamp" in msg_text
    assert "bool connected" in msg_text
    assert "string robot_ip" in msg_text
    assert "string detail" in msg_text


def test_estun_robot_status_message_has_current_sdk_fields():
    msg_text = (REPO_ROOT / "estun_msgs" / "msg" / "EstunRobotStatus.msg").read_text()

    expected_fields = [
        "builtin_interfaces/Time stamp",
        "bool connected",
        "bool robot_error",
        "bool disconnected",
        "bool first_packet_received",
        "uint64 status_packet_count",
        "uint8 active_command_mode",
        "uint8 configured_servo_mode",
        "uint32 queue_depth",
        "uint64 queue_underflow_count",
        "uint64 repeated_send_count",
        "uint64 servo_sdk_fail_count",
    ]
    for field in expected_fields:
        assert field in msg_text


def test_service_bridge_interfaces_keep_expected_shapes():
    expected_snippets = {
        "GetConnectionStatus.srv": [
            "---",
            "estun_msgs/ConnectionStatus status",
        ],
        "SetDo.srv": [
            "uint16 port",
            "bool value",
            "---",
            "bool success",
            "string message",
        ],
        "GetDo.srv": [
            "uint16 port",
            "---",
            "bool success",
            "bool value",
            "string message",
        ],
        "GetRobotConnStatus.srv": [
            "---",
            "uint8 cmd_status",
            "uint8 servo_status",
            "uint8 udp_status",
        ],
        "GetWorldCpos.srv": [
            "---",
            "bool success",
            "float64[16] pos",
        ],
        "GetJointValue.srv": [
            "---",
            "bool success",
            "float64[16] pos",
        ],
        "GetTool.srv": [
            "int32 tool_id",
            "---",
            "bool success",
            "float64[6] tool_data",
        ],
        "GetUser.srv": [
            "int32 user_id",
            "---",
            "bool success",
            "float64[6] user_data",
        ],
        "GetCurErrMsg.srv": [
            "---",
            "bool success",
            "int32 err_id",
            "string err_msg",
        ],
    }

    for srv_name, snippets in expected_snippets.items():
        text = (REPO_ROOT / "estun_msgs" / "srv" / srv_name).read_text()
        for snippet in snippets:
            assert snippet in text

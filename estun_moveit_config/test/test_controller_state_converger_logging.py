import importlib.util
import sys
from pathlib import Path


def _load_controller_state_converger_module():
    script_path = (
        Path(__file__).resolve().parents[2]
        / "estun_hardware"
        / "scripts"
        / "controller_state_converger.py"
    )
    spec = importlib.util.spec_from_file_location(
        "controller_state_converger_script",
        script_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载脚本: {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _FakeContext:
    def __init__(self, ok_value):
        self._ok_value = ok_value

    def ok(self):
        return self._ok_value


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))


class _FakeNode:
    def __init__(self, ok_value):
        self.context = _FakeContext(ok_value)
        self.logger = _FakeLogger()

    def get_logger(self):
        return self.logger


def test_safe_log_uses_ros_logger_when_context_is_valid(capsys):
    module = _load_controller_state_converger_module()
    node = _FakeNode(True)

    module.safe_log(node, "info", "hello")

    assert node.logger.messages == [("info", "hello")]
    captured = capsys.readouterr()
    assert captured.err == ""


def test_safe_log_falls_back_to_stderr_when_context_is_invalid(capsys):
    module = _load_controller_state_converger_module()
    node = _FakeNode(False)

    module.safe_log(node, "info", "bye")

    assert node.logger.messages == []
    captured = capsys.readouterr()
    assert "[INFO] bye" in captured.err


def test_keyboard_interrupt_log_uses_plain_stderr(capsys):
    print("[INFO] 收到中断信号，停止控制器收敛。", file=sys.stderr)

    captured = capsys.readouterr()
    assert "收到中断信号，停止控制器收敛。" in captured.err

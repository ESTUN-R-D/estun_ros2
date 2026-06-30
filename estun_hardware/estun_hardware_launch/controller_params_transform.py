import copy
from dataclasses import dataclass


CONTROLLER_MANAGER_NAME = "controller_manager"
GLOBAL_ALL_NODES_KEY = "/**"
GLOBAL_NODE_PREFIX = "/**/"


def normalize_namespace(namespace):
    return str(namespace).strip().strip("/")


def _update_rate_hz_from_motion_period_ms(motion_period_ms):
    return max(1, int(round(1000.0 / float(motion_period_ms))))


def _manager_overlay_key(namespace, manager_sections):
    normalized_namespace = normalize_namespace(namespace)
    if normalized_namespace:
        return f"/{normalized_namespace}/{CONTROLLER_MANAGER_NAME}"
    for section in manager_sections:
        if section.key_kind == "absolute":
            return section.raw_key
        if section.key_kind == "relative":
            return CONTROLLER_MANAGER_NAME
    return CONTROLLER_MANAGER_NAME


def _merge_dict(base, overlay):
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_dict(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _add_prefix_if_needed(name, prefix):
    if not prefix:
        return name
    if name.startswith(prefix):
        return name
    return f"{prefix}{name}"


def _extract_logical_name_from_absolute_key(raw_key):
    normalized_key = raw_key.rstrip("/") or raw_key
    logical_name = normalized_key.rsplit("/", 1)[-1]
    if not logical_name:
        raise RuntimeError(f"controllers_file 顶层 key 非法: '{raw_key}'")
    return logical_name, normalized_key


def _validate_relative_key(raw_key):
    if "/" in raw_key:
        raise RuntimeError(
            "controllers_file 相对节点 key 只支持裸节点名；"
            f"收到 '{raw_key}'，请改用 '/<namespace>/<node>' 或 '/**/<node>'"
        )


def _validate_global_node_key(logical_name, raw_key):
    if not logical_name or "/" in logical_name:
        raise RuntimeError(
            "controllers_file 通配节点 key 只支持 '/**/<node>' 形式；"
            f"收到 '{raw_key}'"
        )


def _get_ros_parameters(section_data):
    ros_parameters = section_data.get("ros__parameters")
    return ros_parameters if isinstance(ros_parameters, dict) else None


@dataclass
class ControllerParamsSection:
    raw_key: str
    key_kind: str
    logical_name: str | None
    effective_key: str | None
    data: dict

    @property
    def is_global_root(self):
        return self.raw_key == GLOBAL_ALL_NODES_KEY


class ControllerParamsDocument:
    def __init__(self, sections, namespace):
        self.sections = sections
        self.namespace = normalize_namespace(namespace)

    @classmethod
    def from_raw_yaml(cls, raw_yaml, namespace):
        if not isinstance(raw_yaml, dict):
            raise RuntimeError("controllers_file 顶层必须是 YAML 映射")

        sections = []
        normalized_namespace = normalize_namespace(namespace)
        for raw_key, raw_value in raw_yaml.items():
            if not isinstance(raw_key, str) or not raw_key.strip():
                raise RuntimeError(f"controllers_file 顶层 key 非法: '{raw_key}'")
            if not isinstance(raw_value, dict):
                raise RuntimeError(
                    f"controllers_file 顶层节点 '{raw_key}' 必须对应 YAML 映射"
                )

            key = raw_key.strip()
            if key == GLOBAL_ALL_NODES_KEY:
                sections.append(
                    ControllerParamsSection(
                        raw_key=key,
                        key_kind="global",
                        logical_name=None,
                        effective_key=key,
                        data=copy.deepcopy(raw_value),
                    )
                )
                continue

            if key.startswith(GLOBAL_NODE_PREFIX):
                logical_name = key[len(GLOBAL_NODE_PREFIX):]
                _validate_global_node_key(logical_name, key)
                sections.append(
                    ControllerParamsSection(
                        raw_key=key,
                        key_kind="global",
                        logical_name=logical_name,
                        effective_key=key,
                        data=copy.deepcopy(raw_value),
                    )
                )
                continue

            if key.startswith("/"):
                logical_name, normalized_key = _extract_logical_name_from_absolute_key(key)
                if normalized_namespace and not normalized_key.startswith(
                    f"/{normalized_namespace}/"
                ):
                    raise RuntimeError(
                        "controllers_file 中绝对节点 key 与当前 namespace 不一致："
                        f"key='{key}', namespace='{normalized_namespace}'"
                    )
                sections.append(
                    ControllerParamsSection(
                        raw_key=normalized_key,
                        key_kind="absolute",
                        logical_name=logical_name,
                        effective_key=normalized_key,
                        data=copy.deepcopy(raw_value),
                    )
                )
                continue

            _validate_relative_key(key)
            effective_key = (
                f"/{normalized_namespace}/{key}" if normalized_namespace else f"/{key}"
            )
            sections.append(
                ControllerParamsSection(
                    raw_key=key,
                    key_kind="relative",
                    logical_name=key,
                    effective_key=effective_key,
                    data=copy.deepcopy(raw_value),
                )
            )

        document = cls(sections=sections, namespace=normalized_namespace)
        document.validate_concrete_key_collisions()
        return document

    def _concrete_manager_sections(self):
        return [
            section
            for section in self.sections
            if section.logical_name == CONTROLLER_MANAGER_NAME
            and section.key_kind in ("relative", "absolute")
        ]

    def _manager_param_sections(self):
        return [
            section
            for section in self.sections
            if section.logical_name == CONTROLLER_MANAGER_NAME or section.is_global_root
        ]

    def validate_concrete_key_collisions(self):
        seen = {}
        for section in self.sections:
            if section.key_kind == "global":
                continue
            previous = seen.get(section.effective_key)
            if previous is not None:
                raise RuntimeError(
                    "controllers_file 顶层节点冲突："
                    f"'{previous.raw_key}' 与 '{section.raw_key}' 最终都指向 '{section.effective_key}'"
                )
            seen[section.effective_key] = section

    def validate_required_controller_definitions(self, required_controller_names):
        manager_sections = self._manager_param_sections()
        if not manager_sections:
            raise RuntimeError(
                "controllers_file 缺少 controller_manager 配置段；"
                "请提供 'controller_manager'、'/<namespace>/controller_manager' 或 '/**'"
            )

        defined = set()
        for section in manager_sections:
            ros_parameters = _get_ros_parameters(section.data)
            if ros_parameters is None:
                continue
            for controller_name, controller_cfg in ros_parameters.items():
                if isinstance(controller_cfg, dict) and "type" in controller_cfg:
                    defined.add(controller_name)

        missing = sorted(
            controller_name
            for controller_name in required_controller_names
            if controller_name not in defined
        )
        if missing:
            raise RuntimeError(
                "controllers_file 缺少 controller_manager 控制器定义："
                + ", ".join(missing)
            )

    def apply_prefix_overrides(self, prefix):
        for controller_name in ("estun_arm_controller", "forward_position_controller"):
            for section in self.sections:
                if section.logical_name != controller_name:
                    continue
                ros_parameters = _get_ros_parameters(section.data)
                joints = ros_parameters.get("joints") if ros_parameters else None
                if isinstance(joints, list):
                    ros_parameters["joints"] = [
                        _add_prefix_if_needed(joint_name, prefix) for joint_name in joints
                    ]

        for section in self.sections:
            if section.logical_name == "cartesian_forward_controller":
                ros_parameters = _get_ros_parameters(section.data)
                cartesian_joint = ros_parameters.get("joint") if ros_parameters else None
                if isinstance(cartesian_joint, str) and cartesian_joint:
                    ros_parameters["joint"] = _add_prefix_if_needed(cartesian_joint, prefix)

            if section.logical_name == "estun_state_broadcaster":
                ros_parameters = _get_ros_parameters(section.data)
                if ros_parameters is not None:
                    ros_parameters["prefix"] = prefix

            if section.logical_name == "estun_do_controller":
                ros_parameters = _get_ros_parameters(section.data)
                if ros_parameters is not None:
                    ros_parameters["prefix"] = prefix
                    ros_parameters["sdk_namespace"] = (
                        "/estun" if not prefix else f"/{prefix}estun"
                    )

    def export_raw_yaml(self):
        exported = {}
        for section in self.sections:
            exported[section.raw_key] = copy.deepcopy(section.data)
        return exported

    def compile_runtime_yaml(self):
        compiled = {}
        for section in self.sections:
            if section.key_kind == "relative" and self.namespace:
                target_key = f"/{self.namespace}/{section.logical_name}"
            else:
                target_key = section.raw_key
            compiled[target_key] = copy.deepcopy(section.data)
        return compiled

    def build_update_rate_overlay(self, motion_period_ms):
        manager_key = _manager_overlay_key(self.namespace, self._concrete_manager_sections())
        return {
            manager_key: {
                "ros__parameters": {
                    "update_rate": _update_rate_hz_from_motion_period_ms(motion_period_ms)
                }
            }
        }


def with_prefixed_controllers(raw_yaml, prefix):
    if not raw_yaml:
        return raw_yaml
    document = ControllerParamsDocument.from_raw_yaml(raw_yaml, "")
    document.apply_prefix_overrides(prefix)
    return document.export_raw_yaml()


def with_node_namespace(raw_yaml, namespace):
    normalized_namespace = normalize_namespace(namespace)
    if not raw_yaml or not normalized_namespace:
        return raw_yaml
    document = ControllerParamsDocument.from_raw_yaml(raw_yaml, normalized_namespace)
    return document.compile_runtime_yaml()


def with_motion_period_update_rate(raw_yaml, motion_period_ms, namespace=""):
    if not raw_yaml:
        return raw_yaml
    document = ControllerParamsDocument.from_raw_yaml(raw_yaml, namespace)
    compiled = document.compile_runtime_yaml()
    overlay = document.build_update_rate_overlay(motion_period_ms)
    return _merge_dict(compiled, overlay)


def build_runtime_controller_params(raw_yaml, namespace, prefix, motion_period_ms):
    document = ControllerParamsDocument.from_raw_yaml(raw_yaml, namespace)
    document.apply_prefix_overrides(prefix)
    return document, document.compile_runtime_yaml(), document.build_update_rate_overlay(
        motion_period_ms
    )

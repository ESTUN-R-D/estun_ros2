from __future__ import annotations

import os
import random
import re
from pathlib import Path
from typing import Mapping, MutableMapping


_SCOPE_PATTERN = re.compile(r"[^A-Za-z0-9_.-]+")


def _normalize_scope(scope: str) -> str:
    normalized = _SCOPE_PATTERN.sub("_", scope).strip("._-")
    return normalized or "estun_test"


def build_ros_test_env(
    scope: str,
    env: Mapping[str, str] | None = None,
    *,
    domain_id: int | None = None,
    randomize_domain_id: bool = False,
) -> dict[str, str]:
    resolved_env = dict(os.environ if env is None else env)

    ros_home = resolved_env.get("ROS_HOME") or resolved_env.get("ESTUN_TEST_ROS_HOME")
    if ros_home is None:
        ros_home = str(Path("/tmp") / f"{_normalize_scope(scope)}_{os.getpid()}")

    ros_log_dir = resolved_env.get("ROS_LOG_DIR") or resolved_env.get("ESTUN_TEST_ROS_LOG_DIR")
    if ros_log_dir is None:
        ros_log_dir = str(Path(ros_home) / "log")

    Path(ros_log_dir).mkdir(parents=True, exist_ok=True)
    Path(ros_home).mkdir(parents=True, exist_ok=True)

    resolved_env["ROS_HOME"] = ros_home
    resolved_env["ROS_LOG_DIR"] = ros_log_dir

    if domain_id is not None:
        resolved_env["ROS_DOMAIN_ID"] = str(domain_id)
    elif randomize_domain_id and "ROS_DOMAIN_ID" not in resolved_env:
        resolved_env["ROS_DOMAIN_ID"] = str(40 + random.randint(0, 80))

    return resolved_env


def ensure_process_ros_test_env(
    scope: str,
    *,
    domain_id: int | None = None,
    randomize_domain_id: bool = False,
) -> dict[str, str]:
    resolved_env = build_ros_test_env(
        scope,
        os.environ,
        domain_id=domain_id,
        randomize_domain_id=randomize_domain_id,
    )
    for key in ("ROS_HOME", "ROS_LOG_DIR", "ROS_DOMAIN_ID"):
        if key in resolved_env:
            os.environ[key] = resolved_env[key]
    return resolved_env


def apply_ros_test_env(
    target_env: MutableMapping[str, str],
    scope: str,
    *,
    domain_id: int | None = None,
    randomize_domain_id: bool = False,
) -> dict[str, str]:
    resolved_env = build_ros_test_env(
        scope,
        target_env,
        domain_id=domain_id,
        randomize_domain_id=randomize_domain_id,
    )
    target_env.update(
        {key: resolved_env[key] for key in ("ROS_HOME", "ROS_LOG_DIR") if key in resolved_env}
    )
    if "ROS_DOMAIN_ID" in resolved_env:
        target_env["ROS_DOMAIN_ID"] = resolved_env["ROS_DOMAIN_ID"]
    return resolved_env

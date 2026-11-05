import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def load_pad_speed_limits(pad_model, robot_type):
    pad_model = str(pad_model).strip()
    if not pad_model or Path(pad_model).name != pad_model or pad_model in (".", ".."):
        raise ValueError(f"invalid pad model name: {pad_model!r}")

    if robot_type == "mock":
        config_path = (
            Path(get_package_share_directory("centauro_hmi_backend"))
            / "config"
            / "mock"
            / "pad"
            / f"{pad_model}.yaml"
        )
    else:
        config_path = (
            Path(get_package_share_directory("robot_bringup"))
            / "config"
            / "base"
            / "pad"
            / f"{pad_model}.yaml"
        )
    try:
        with config_path.open(encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except OSError as exc:
        raise RuntimeError(f"could not load pad configuration {config_path}: {exc}") from exc

    try:
        movement = config["/**"]["robotnik_pad"]["ros__parameters"]["Movement"]
        linear_speed = float(movement["max_linear_speed"])
        angular_speed = float(movement["max_angular_speed"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid Movement speed limits in {config_path}") from exc

    if not math.isfinite(linear_speed) or linear_speed <= 0.0:
        raise ValueError(f"Movement.max_linear_speed must be finite and positive in {config_path}")
    if not math.isfinite(angular_speed) or angular_speed <= 0.0:
        raise ValueError(f"Movement.max_angular_speed must be finite and positive in {config_path}")
    return linear_speed, angular_speed
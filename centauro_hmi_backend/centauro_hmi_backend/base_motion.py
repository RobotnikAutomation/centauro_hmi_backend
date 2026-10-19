import math


def parse_base_joystick(payload, max_linear_speed, max_angular_speed):
    """Convert normalized planar joystick axes to body-frame velocities."""
    twist = payload.get("twist", {})
    if not isinstance(twist, dict):
        raise ValueError("twist must be an object")
    linear, angular = twist.get("linear"), twist.get("angular")
    if not isinstance(linear, (list, tuple)) or len(linear) != 3:
        raise ValueError("twist.linear must contain three values")
    if not isinstance(angular, (list, tuple)) or len(angular) != 3:
        raise ValueError("twist.angular must contain three values")
    axes = [float(value) for value in (*linear, *angular)]
    if not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in axes):
        raise ValueError("joystick axes must be finite values between -1 and 1")
    if axes[2] != 0.0 or axes[3] != 0.0 or axes[4] != 0.0:
        raise ValueError("base joystick supports linear x/y and angular z only")
    return [
        axes[0] * max_linear_speed,
        axes[1] * max_linear_speed,
        axes[5] * max_angular_speed,
    ]

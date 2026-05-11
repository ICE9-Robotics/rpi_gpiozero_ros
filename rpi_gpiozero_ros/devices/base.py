"""Base class and shared helpers for gpiozero-backed ROS device wrappers."""

from typing import Any, Dict, Optional, Union

from gpiozero import Device as _GZDevice
from gpiozero.pins.mock import MockFactory, MockPWMPin
from rclpy.node import Node


PinType = Union[int, str]


def configure_pin_factory(pin_factory_mode: str) -> None:
    """Configure gpiozero global pin factory."""
    if pin_factory_mode == "auto":
        return
    if pin_factory_mode == "mock":
        # PWM-capable mock pins so PWMOutputDevice works in off-target tests.
        _GZDevice.pin_factory = MockFactory(pin_class=MockPWMPin)
        return
    raise ValueError("Unsupported pin_factory mode. Use 'auto' or 'mock'.")


def resolve_pin(pin_numbering: str, pin: int) -> PinType:
    """Resolve configured pin number to gpiozero pin identifier."""
    normalized = pin_numbering.strip().lower()
    if not normalized in ["bcm", "gpio", "board", "wpi"]:
        raise ValueError(f"Unsupported 'pin_numbering'. Use 'bcm', 'gpio', 'board', or 'wpi'.")
    if normalized == "bcm":
        return int(pin)
    else:
        return f"{normalized.upper()}{int(pin)}"


def bool_from_spec(value: Any, field_name: str) -> bool:
    """Validate scalar bool field from a gpio device spec."""
    if not isinstance(value, bool):
        raise ValueError(f"'{field_name}' must be a bool.")
    return value


def optional_bool_from_spec(value: Any, field_name: str) -> Optional[bool]:
    """Validate optional bool field from a gpio device spec."""
    if value is None:
        return None
    return bool_from_spec(value, field_name)


def int_from_spec(value: Any, field_name: str) -> int:
    """Validate scalar integer field from a gpio device spec."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"'{field_name}' must be an integer.")
    return int(value)


def float_from_spec(value: Any, field_name: str) -> float:
    """Validate scalar numeric field from a gpio device spec."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"'{field_name}' must be numeric.")
    return float(value)


def optional_non_negative_float_from_spec(value: Any, field_name: str) -> Optional[float]:
    """Validate optional non-negative float field from a gpio device spec."""
    if value is None:
        return None
    parsed = float_from_spec(value, field_name)
    validate_non_negative_float(parsed, field_name)
    return parsed


def validate_non_negative_int(value: int, param_name: str) -> None:
    """Validate integer values that cannot be negative."""
    if value < 0:
        raise ValueError(f"'{param_name}' (={value:.3f}) must be >= 0.")


def validate_positive_int(value: int, param_name: str) -> None:
    """Validate integer values that must be positive."""
    if value <= 0:
        raise ValueError(f"'{param_name}' (={value:.3f}) must be > 0.")


def validate_non_negative_float(value: float, param_name: str) -> None:
    """Validate float values that cannot be negative."""
    if value < 0.0:
        raise ValueError(f"'{param_name}' (={value:.3f}) must be >= 0.0.")


def validate_positive_float(value: float, param_name: str) -> None:
    """Validate float values that must be positive."""
    if value <= 0.0:
        raise ValueError(f"'{param_name}' (={value:.3f}) must be > 0.0.")


def validate_unit_interval(value: float, param_name: str) -> None:
    """Validate float values in closed unit interval."""
    if value < 0.0 or value > 1.0:
        raise ValueError(f"'{param_name}' (={value:.3f}) must be in range [0.0, 1.0].")


class GPIODevice:
    """Base class for one gpiozero-backed ROS device.

    Subclasses must set `TYPE_NAME` (matching `gpio.<name>.type`) and implement
    `_build`, which is responsible for constructing the underlying gpiozero
    device and any ROS publishers/services. Subclasses may override `tick` for
    periodic publishing and `close` for custom teardown.
    """

    TYPE_NAME: str = ""

    def __init__(
        self,
        node: Node,
        logical_name: str,
        spec: Dict[str, Any],
        pin_numbering: str,
    ) -> None:
        self.node = node
        self.name = logical_name
        self.spec_prefix = f"gpio.{logical_name}"

        if "pin" not in spec:
            raise ValueError(f"'{self.spec_prefix}.pin' is required.")
        pin = int_from_spec(spec["pin"], f"{self.spec_prefix}.pin")
        validate_non_negative_int(pin, f"{self.spec_prefix}.pin")
        self.pin = resolve_pin(pin_numbering, pin)

        # Per-device publish_rate_hz is accepted and validated for backwards
        # compatibility but ignored at runtime; publishing uses the node-global
        # timer only.
        if "publish_rate_hz" in spec:
            value = float_from_spec(spec["publish_rate_hz"], f"{self.spec_prefix}.publish_rate_hz")
            validate_positive_float(value, f"{self.spec_prefix}.publish_rate_hz")

        self._device = None
        self._build(spec)

    def _build(self, spec: Dict[str, Any]) -> None:
        """Construct gpiozero device and ROS interfaces. Override in subclass."""
        raise NotImplementedError

    def tick(self) -> None:
        """Called periodically by the node timer. Default: no-op."""
        return

    def close(self) -> None:
        """Release the underlying gpiozero device."""
        if self._device is not None:
            self._device.close()

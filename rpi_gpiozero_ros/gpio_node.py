"""ROS 2 node integrating gpiozero base input/output devices."""

import re
from typing import Any, Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rpi_gpiozero_ros.devices import DEVICE_CLASSES, GPIODevice, configure_pin_factory


class GPIOZeroNode(Node):
    """Expose configured gpiozero devices through ROS 2 topics/services."""

    def __init__(self) -> None:
        super().__init__("gpiozero_node", automatically_declare_parameters_from_overrides=True)

        configure_pin_factory(self._string_param("pin_factory", "auto"))
        pin_numbering = self._string_param("pin_numbering", "bcm")

        self._devices: List[GPIODevice] = self._setup_devices(pin_numbering)

        publish_rate_hz = self._float_param("publish_rate_hz", 20.0)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0.0")
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._tick)

    def _setup_devices(self, pin_numbering: str) -> List[GPIODevice]:
        """Instantiate gpiozero device wrappers from ROS parameters."""
        specs = self._collect_named_device_specs()
        if not specs:
            raise ValueError(
                "No GPIO devices configured. Declare parameters under gpio.<logical_name>.<field> "
                "(for example gpio.button.type and gpio.button.pin)."
            )
        devices: List[GPIODevice] = []
        for logical_name in sorted(specs.keys()):
            spec = specs[logical_name]
            device_type = self._required_type(spec, logical_name)
            if device_type not in DEVICE_CLASSES:
                raise ValueError(
                    f"Unsupported gpio.{logical_name}.type '{device_type}'. "
                    f"Supported: {sorted(DEVICE_CLASSES.keys())}."
                )
            cls = DEVICE_CLASSES[device_type]
            devices.append(
                cls(node=self, logical_name=logical_name, spec=spec, pin_numbering=pin_numbering)
            )
        return devices

    def _required_type(self, spec: Dict[str, Any], logical_name: str) -> str:
        """Read and normalise the device `type` field from a spec."""
        if "type" not in spec:
            raise ValueError(f"'gpio.{logical_name}.type' is required.")
        value = spec["type"]
        if not isinstance(value, str):
            raise ValueError(f"'gpio.{logical_name}.type' must be a string.")
        if value.strip() == "":
            raise ValueError(f"'gpio.{logical_name}.type' cannot be empty.")
        return value.strip().lower()

    def _collect_named_device_specs(self) -> Dict[str, Dict[str, Any]]:
        """Collect device specs from parameter names like `gpio.button.type`."""
        specs: Dict[str, Dict[str, Any]] = {}
        for param_name, param in self._parameters.items():
            match = re.match(r"^gpio\.([^.]+)\.(\w+)$", param_name)
            if match is None:
                continue
            specs.setdefault(match.group(1), {})[match.group(2)] = param.value
        return specs

    def _tick(self) -> None:
        """Fan out the periodic tick to every device."""
        for device in self._devices:
            device.tick()

    def _get_parameter_or_none(self, param_name: str) -> Optional[Parameter]:
        """Fetch parameter by name when declared or auto-declared from overrides."""
        if not self.has_parameter(param_name):
            return None
        return self.get_parameter(param_name)

    def _string_param(self, param_name: str, default: str) -> str:
        """Read scalar string parameter with default fallback."""
        param = self._get_parameter_or_none(param_name)
        if param is None:
            return default
        if param.type_ != Parameter.Type.STRING:
            raise ValueError(f"'{param_name}' must be a string.")
        return str(param.value)

    def _float_param(self, param_name: str, default: float) -> float:
        """Read scalar float/int parameter with default fallback."""
        param = self._get_parameter_or_none(param_name)
        if param is None:
            return default
        if param.type_ == Parameter.Type.DOUBLE:
            return float(param.value)
        if param.type_ == Parameter.Type.INTEGER:
            return float(param.value)
        raise ValueError(f"'{param_name}' must be a float or integer.")

    def destroy_node(self) -> bool:
        for device in self._devices:
            device.close()
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the ROS 2 gpiozero node."""
    rclpy.init(args=args)
    node = GPIOZeroNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

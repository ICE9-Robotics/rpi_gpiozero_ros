"""ROS 2 node integrating gpiozero base input/output devices."""

import re
from functools import partial
from typing import Any, Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_common_srvs.srv import SetFloat32
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

from rpi_gpiozero_ros.gpio_devices import (
    GPIODeviceRegistry,
    PinType,
    DigitalInputConfig,
    DigitalOutputConfig,
    PWMOutputConfig,
    SmoothedInputConfig,
    configure_pin_factory,
)


class GPIOZeroNode(Node):
    """Expose configured gpiozero devices through ROS 2 topics/services."""

    def __init__(self) -> None:
        super().__init__("gpiozero_node", automatically_declare_parameters_from_overrides=True)

        pin_factory_mode = self._string_param("pin_factory", "auto")
        configure_pin_factory(pin_factory_mode)

        self._registry = GPIODeviceRegistry()
        self._digital_input_publishers = {}
        self._smoothed_input_publishers = {}
        self._services = []

        self._setup_devices()
        self._setup_interfaces()

        publish_rate_hz = self._float_param("publish_rate_hz", 20.0)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0.0")
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_inputs)

    def _setup_devices(self) -> None:
        """Instantiate gpiozero devices from ROS parameters."""
        pin_numbering = self._string_param("pin_numbering", "bcm")
        if self._setup_named_devices(pin_numbering):
            return
        if self._setup_indexed_devices(pin_numbering):
            return
        self._setup_digital_inputs(pin_numbering)
        self._setup_smoothed_inputs(pin_numbering)
        self._setup_digital_outputs(pin_numbering)
        self._setup_pwm_outputs(pin_numbering)

    def _setup_named_devices(self, pin_numbering: str) -> bool:
        """Instantiate devices from `gpio.<name>.<field>` style parameters."""
        specs = self._collect_named_device_specs()
        if not specs:
            return False
        for name in sorted(specs.keys()):
            self._setup_device_spec(specs[name], pin_numbering, f"gpio.{name}", explicit_name=name)
        return True

    def _setup_indexed_devices(self, pin_numbering: str) -> bool:
        """Instantiate devices from `devices.<index>.*` style parameters."""
        specs = self._collect_indexed_device_specs()
        if not specs:
            return False

        for index in sorted(specs.keys()):
            self._setup_device_spec(specs[index], pin_numbering, f"devices.{index}")

        return True

    def _setup_device_spec(
        self, spec: Dict[str, Any], pin_numbering: str, path_prefix: str, explicit_name: Optional[str] = None
    ) -> None:
        """Instantiate one device from a normalized spec dict."""
        name = explicit_name or self._required_string_field(spec, "name", path_prefix)
        device_type = self._required_string_field(spec, "type", path_prefix).strip().lower()
        pin = self._required_int_field(spec, "pin", path_prefix)
        self._validate_non_negative_int(pin, f"{path_prefix}.pin")
        resolved_pin = self._resolve_pin(pin_numbering, pin)

        publish_rate = spec.get("publish_rate_hz")
        if publish_rate is not None:
            publish_rate_value = self._float_from_spec(publish_rate, f"{path_prefix}.publish_rate_hz")
            self._validate_positive_float(publish_rate_value, f"{path_prefix}.publish_rate_hz")

        if device_type == "digital_input":
            pull_up = self._optional_bool_spec(spec.get("pull_up"), f"{path_prefix}.pull_up")
            if "pull_up" not in spec:
                pull_up = None if "active_state" in spec else True
            bounce_time = self._optional_non_negative_float_spec(spec.get("bounce_time"), f"{path_prefix}.bounce_time")
            active_state = self._optional_bool_spec(spec.get("active_state"), f"{path_prefix}.active_state")
            if pull_up is not None and active_state is not None:
                raise ValueError(
                    f"'{path_prefix}.active_state' must be omitted when '{path_prefix}.pull_up' is set. "
                    "gpiozero only allows active_state for floating inputs."
                )
            self._registry.add_digital_input(
                DigitalInputConfig(
                    name=name,
                    pin=resolved_pin,
                    pull_up=pull_up,
                    bounce_time=bounce_time,
                    active_state=active_state,
                )
            )
            return

        if device_type == "smoothed_input":
            queue_len = self._int_from_spec(spec.get("queue_len", 5), f"{path_prefix}.queue_len")
            self._validate_positive_int(queue_len, f"{path_prefix}.queue_len")
            sample_wait = self._float_from_spec(spec.get("sample_wait", 0.0), f"{path_prefix}.sample_wait")
            self._validate_non_negative_float(sample_wait, f"{path_prefix}.sample_wait")
            threshold = self._float_from_spec(spec.get("threshold", 0.5), f"{path_prefix}.threshold")
            self._validate_unit_interval(threshold, f"{path_prefix}.threshold")
            partial = self._bool_from_spec(spec.get("partial", False), f"{path_prefix}.partial")
            self._registry.add_smoothed_input(
                SmoothedInputConfig(
                    name=name,
                    pin=resolved_pin,
                    queue_len=queue_len,
                    sample_wait=sample_wait,
                    threshold=threshold,
                    partial=partial,
                    ignore=None,
                )
            )
            return

        if device_type == "digital_output":
            active_high = self._bool_from_spec(spec.get("active_high", True), f"{path_prefix}.active_high")
            initial_value = self._bool_from_spec(spec.get("initial_value", False), f"{path_prefix}.initial_value")
            self._registry.add_digital_output(
                DigitalOutputConfig(
                    name=name,
                    pin=resolved_pin,
                    active_high=active_high,
                    initial_value=initial_value,
                )
            )
            return

        if device_type == "pwm_output":
            active_high = self._bool_from_spec(spec.get("active_high", True), f"{path_prefix}.active_high")
            initial_value = self._float_from_spec(spec.get("initial_value", 0.0), f"{path_prefix}.initial_value")
            self._validate_unit_interval(initial_value, f"{path_prefix}.initial_value")
            frequency = self._float_from_spec(spec.get("frequency", 100.0), f"{path_prefix}.frequency")
            self._validate_positive_float(frequency, f"{path_prefix}.frequency")
            self._registry.add_pwm_output(
                PWMOutputConfig(
                    name=name,
                    pin=resolved_pin,
                    active_high=active_high,
                    initial_value=initial_value,
                    frequency=frequency,
                )
            )
            return

        raise ValueError(f"Unsupported {path_prefix}.type '{device_type}'.")

    def _collect_named_device_specs(self) -> Dict[str, Dict[str, Any]]:
        """Collect device specs from parameter names like `gpio.button.type`."""
        specs: Dict[str, Dict[str, Any]] = {}
        for param_name, param in self._parameters.items():
            match = re.match(r"^gpio\.([^.]+)\.(\w+)$", param_name)
            if match is None:
                continue
            name = match.group(1)
            field = match.group(2)
            specs.setdefault(name, {})[field] = param.value
        return specs

    def _collect_indexed_device_specs(self) -> Dict[int, Dict[str, Any]]:
        """Collect device specs from parameter names like `devices.0.name`."""
        specs: Dict[int, Dict[str, Any]] = {}
        for param_name, param in self._parameters.items():
            match = re.match(r"^devices(?:\.|\[)(\d+)(?:\])?\.(\w+)$", param_name)
            if match is None:
                continue
            index = int(match.group(1))
            field = match.group(2)
            specs.setdefault(index, {})[field] = param.value
        return specs

    def _required_string_field(self, spec: Dict[str, Any], field: str, path_prefix: str) -> str:
        """Read required string field from a device spec."""
        if field not in spec:
            raise ValueError(f"'{path_prefix}.{field}' is required.")
        value = spec[field]
        if not isinstance(value, str):
            raise ValueError(f"'{path_prefix}.{field}' must be a string.")
        if value.strip() == "":
            raise ValueError(f"'{path_prefix}.{field}' cannot be empty.")
        return value

    def _required_int_field(self, spec: Dict[str, Any], field: str, path_prefix: str) -> int:
        """Read required integer field from a device spec."""
        if field not in spec:
            raise ValueError(f"'{path_prefix}.{field}' is required.")
        return self._int_from_spec(spec[field], f"{path_prefix}.{field}")

    def _bool_from_spec(self, value: Any, field_name: str) -> bool:
        """Validate scalar bool field from indexed device spec."""
        if not isinstance(value, bool):
            raise ValueError(f"'{field_name}' must be a bool.")
        return value

    def _optional_bool_spec(self, value: Any, field_name: str) -> Optional[bool]:
        """Validate optional bool field from indexed device spec."""
        if value is None:
            return None
        return self._bool_from_spec(value, field_name)

    def _int_from_spec(self, value: Any, field_name: str) -> int:
        """Validate scalar integer field from indexed device spec."""
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"'{field_name}' must be an integer.")
        return int(value)

    def _float_from_spec(self, value: Any, field_name: str) -> float:
        """Validate scalar numeric field from indexed device spec."""
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"'{field_name}' must be numeric.")
        return float(value)

    def _optional_non_negative_float_spec(self, value: Any, field_name: str) -> Optional[float]:
        """Validate optional non-negative float field from indexed device spec."""
        if value is None:
            return None
        parsed = self._float_from_spec(value, field_name)
        self._validate_non_negative_float(parsed, field_name)
        return parsed

    def _setup_digital_inputs(self, pin_numbering: str) -> None:
        """Instantiate DigitalInputDevice objects from ROS parameters."""
        di_names = self._string_list("digital_input.names", [])
        di_pins = self._int_list("digital_input.pins", [])
        self._validate_lengths(di_names, di_pins, "digital_input.names", "digital_input.pins")

        pull_up_values = self._resolve_bool_param("digital_input.pull_up", len(di_names), True)
        bounce_values = self._resolve_float_param("digital_input.bounce_time", len(di_names), 0.0)
        active_state_values = self._resolve_int_param("digital_input.active_state", len(di_names), -1)

        for index, (name, pin) in enumerate(zip(di_names, di_pins)):
            self._validate_non_negative_int(pin, f"digital_input.pins[{index}]")
            self._validate_non_negative_float(bounce_values[index], f"digital_input.bounce_time[{index}]")
            self._validate_active_state(active_state_values[index], f"digital_input.active_state[{index}]")
            self._validate_input_mode(
                pull_up_values[index],
                active_state_values[index],
                f"digital_input.pull_up[{index}]",
                f"digital_input.active_state[{index}]",
            )
            bounce_time = bounce_values[index] if bounce_values[index] > 0.0 else None
            active_state: Optional[bool] = None if active_state_values[index] < 0 else bool(active_state_values[index])
            self._registry.add_digital_input(
                DigitalInputConfig(
                    name=name,
                    pin=self._resolve_pin(pin_numbering, pin),
                    pull_up=pull_up_values[index],
                    bounce_time=bounce_time,
                    active_state=active_state,
                )
            )

    def _setup_smoothed_inputs(self, pin_numbering: str) -> None:
        """Instantiate SmoothedInputDevice objects from ROS parameters."""
        si_names = self._string_list("smoothed_input.names", [])
        si_pins = self._int_list("smoothed_input.pins", [])
        self._validate_lengths(si_names, si_pins, "smoothed_input.names", "smoothed_input.pins")

        queue_len_values = self._resolve_int_param("smoothed_input.queue_len", len(si_names), 5)
        sample_wait_values = self._resolve_float_param("smoothed_input.sample_wait", len(si_names), 0.0)
        threshold_values = self._resolve_float_param("smoothed_input.threshold", len(si_names), 0.5)
        partial_values = self._resolve_bool_param("smoothed_input.partial", len(si_names), False)

        for index, (name, pin) in enumerate(zip(si_names, si_pins)):
            self._validate_non_negative_int(pin, f"smoothed_input.pins[{index}]")
            self._validate_positive_int(queue_len_values[index], f"smoothed_input.queue_len[{index}]")
            self._validate_non_negative_float(sample_wait_values[index], f"smoothed_input.sample_wait[{index}]")
            self._validate_unit_interval(threshold_values[index], f"smoothed_input.threshold[{index}]")
            self._registry.add_smoothed_input(
                SmoothedInputConfig(
                    name=name,
                    pin=self._resolve_pin(pin_numbering, pin),
                    queue_len=queue_len_values[index],
                    sample_wait=sample_wait_values[index],
                    threshold=threshold_values[index],
                    partial=partial_values[index],
                    ignore=None,
                )
            )

    def _setup_digital_outputs(self, pin_numbering: str) -> None:
        """Instantiate DigitalOutputDevice objects from ROS parameters."""
        do_names = self._string_list("digital_output.names", [])
        do_pins = self._int_list("digital_output.pins", [])
        self._validate_lengths(do_names, do_pins, "digital_output.names", "digital_output.pins")

        active_high_values = self._resolve_bool_param("digital_output.active_high", len(do_names), True)
        initial_value_values = self._resolve_bool_param("digital_output.initial_value", len(do_names), False)

        for index, (name, pin) in enumerate(zip(do_names, do_pins)):
            self._validate_non_negative_int(pin, f"digital_output.pins[{index}]")
            self._registry.add_digital_output(
                DigitalOutputConfig(
                    name=name,
                    pin=self._resolve_pin(pin_numbering, pin),
                    active_high=active_high_values[index],
                    initial_value=initial_value_values[index],
                )
            )

    def _setup_pwm_outputs(self, pin_numbering: str) -> None:
        """Instantiate PWMOutputDevice objects from ROS parameters."""
        pwm_names = self._string_list("pwm_output.names", [])
        pwm_pins = self._int_list("pwm_output.pins", [])
        self._validate_lengths(pwm_names, pwm_pins, "pwm_output.names", "pwm_output.pins")

        active_high_values = self._resolve_bool_param("pwm_output.active_high", len(pwm_names), True)
        initial_value_values = self._resolve_float_param("pwm_output.initial_value", len(pwm_names), 0.0)
        frequency_values = self._resolve_float_param("pwm_output.frequency", len(pwm_names), 100.0)

        for index, (name, pin) in enumerate(zip(pwm_names, pwm_pins)):
            self._validate_non_negative_int(pin, f"pwm_output.pins[{index}]")
            self._validate_unit_interval(initial_value_values[index], f"pwm_output.initial_value[{index}]")
            self._validate_positive_float(frequency_values[index], f"pwm_output.frequency[{index}]")
            self._registry.add_pwm_output(
                PWMOutputConfig(
                    name=name,
                    pin=self._resolve_pin(pin_numbering, pin),
                    active_high=active_high_values[index],
                    initial_value=initial_value_values[index],
                    frequency=frequency_values[index],
                )
            )

    def _setup_interfaces(self) -> None:
        """Create ROS publishers and services for all devices."""
        for name in self._registry.digital_inputs:
            topic = f"~/inputs/{name}/state"
            self._digital_input_publishers[name] = self.create_publisher(Bool, topic, 10)

        for name in self._registry.smoothed_inputs:
            topic = f"~/smoothed_inputs/{name}/value"
            self._smoothed_input_publishers[name] = self.create_publisher(Float32, topic, 10)

        for name in self._registry.digital_outputs:
            service_name = f"~/outputs/{name}/set"
            srv = self.create_service(
                SetBool,
                service_name,
                partial(self._set_digital_output, output_name=name),
            )
            self._services.append(srv)

        for name in self._registry.pwm_outputs:
            service_name = f"~/pwm_outputs/{name}/set"
            srv = self.create_service(
                SetFloat32,
                service_name,
                partial(self._set_pwm_output, output_name=name),
            )
            self._services.append(srv)

    def _publish_inputs(self) -> None:
        """Publish current state/value for input devices."""
        for name, device in self._registry.digital_inputs.items():
            msg = Bool()
            msg.data = bool(device.value)
            self._digital_input_publishers[name].publish(msg)

        for name, device in self._registry.smoothed_inputs.items():
            msg = Float32()
            msg.data = float(device.value)
            self._smoothed_input_publishers[name].publish(msg)

    def _set_digital_output(
        self, request: SetBool.Request, response: SetBool.Response, output_name: str
    ) -> SetBool.Response:
        """Handle set requests for DigitalOutputDevice instances."""
        device = self._registry.digital_outputs[output_name]
        if request.data:
            device.on()
        else:
            device.off()
        response.success = True
        response.message = f"Set digital output '{output_name}' to {request.data}"
        return response

    def _set_pwm_output(
        self, request: SetFloat32.Request, response: SetFloat32.Response, output_name: str
    ) -> SetFloat32.Response:
        """Handle set requests for PWMOutputDevice instances."""
        value = request.data
        if value < 0.0 or value > 1.0:
            response.success = False
            response.message = "PWM value must be in range [0.0, 1.0]"
            return response
        device = self._registry.pwm_outputs[output_name]
        device.value = value
        response.success = True
        response.message = f"Set pwm output '{output_name}' to {value:.3f}"
        return response

    def _validate_lengths(self, a: List[Any], b: List[Any], a_name: str, b_name: str) -> None:
        """Validate that paired parameter arrays are aligned."""
        if len(a) != len(b):
            raise ValueError(f"'{a_name}' and '{b_name}' must have the same length.")

    def _string_list(self, param_name: str, default: List[str]) -> List[str]:
        """Read a ROS string array parameter with default fallback."""
        param = self._get_parameter_or_none(param_name)
        if param is None:
            return default
        if param.type_ != Parameter.Type.STRING_ARRAY:
            raise ValueError(f"'{param_name}' must be a string array.")
        return list(param.value)

    def _int_list(self, param_name: str, default: List[int]) -> List[int]:
        """Read a ROS integer array parameter with default fallback."""
        param = self._get_parameter_or_none(param_name)
        if param is None:
            return default
        if param.type_ != Parameter.Type.INTEGER_ARRAY:
            raise ValueError(f"'{param_name}' must be an integer array.")
        return list(param.value)

    def _resolve_bool_param(self, param_name: str, count: int, default: bool) -> List[bool]:
        """Resolve scalar-or-array bool parameter into per-device list."""
        param = self._get_parameter_or_none(param_name)
        if param is None:
            return [default] * count
        if param.type_ == Parameter.Type.BOOL_ARRAY:
            values = list(param.value)
            self._validate_count(values, param_name, count)
            return values
        if param.type_ == Parameter.Type.BOOL:
            return [bool(param.value)] * count
        raise ValueError(f"'{param_name}' must be bool or bool array.")

    def _resolve_int_param(self, param_name: str, count: int, default: int) -> List[int]:
        """Resolve scalar-or-array int parameter into per-device list."""
        param = self._get_parameter_or_none(param_name)
        if param is None:
            return [default] * count
        if param.type_ == Parameter.Type.INTEGER_ARRAY:
            values = list(param.value)
            self._validate_count(values, param_name, count)
            return values
        if param.type_ == Parameter.Type.INTEGER:
            return [int(param.value)] * count
        raise ValueError(f"'{param_name}' must be integer or integer array.")

    def _resolve_float_param(self, param_name: str, count: int, default: float) -> List[float]:
        """Resolve scalar-or-array float parameter into per-device list."""
        param = self._get_parameter_or_none(param_name)
        if param is None:
            return [default] * count
        if param.type_ == Parameter.Type.DOUBLE_ARRAY:
            values = [float(value) for value in param.value]
            self._validate_count(values, param_name, count)
            return values
        if param.type_ == Parameter.Type.INTEGER_ARRAY:
            values = [float(value) for value in param.value]
            self._validate_count(values, param_name, count)
            return values
        if param.type_ == Parameter.Type.DOUBLE:
            return [float(param.value)] * count
        if param.type_ == Parameter.Type.INTEGER:
            return [float(param.value)] * count
        raise ValueError(f"'{param_name}' must be float/int scalar or array.")

    def _validate_count(self, values: List[Any], values_name: str, expected_count: int) -> None:
        """Validate optional value list count against number of devices."""
        if len(values) != expected_count:
            raise ValueError(f"'{values_name}' length must match corresponding device names length.")

    def _validate_non_negative_int(self, value: int, param_name: str) -> None:
        """Validate integer values that cannot be negative."""
        if value < 0:
            raise ValueError(f"'{param_name}' must be >= 0.")

    def _validate_positive_int(self, value: int, param_name: str) -> None:
        """Validate integer values that must be positive."""
        if value <= 0:
            raise ValueError(f"'{param_name}' must be > 0.")

    def _validate_non_negative_float(self, value: float, param_name: str) -> None:
        """Validate float values that cannot be negative."""
        if value < 0.0:
            raise ValueError(f"'{param_name}' must be >= 0.0.")

    def _validate_positive_float(self, value: float, param_name: str) -> None:
        """Validate float values that must be positive."""
        if value <= 0.0:
            raise ValueError(f"'{param_name}' must be > 0.0.")

    def _validate_unit_interval(self, value: float, param_name: str) -> None:
        """Validate float values in closed unit interval."""
        if value < 0.0 or value > 1.0:
            raise ValueError(f"'{param_name}' must be in range [0.0, 1.0].")

    def _validate_active_state(self, value: int, param_name: str) -> None:
        """Validate digital input active state enum values."""
        if value not in (-1, 0, 1):
            raise ValueError(f"'{param_name}' must be one of -1, 0, 1.")

    def _validate_input_mode(
        self,
        pull_up: bool,
        active_state: int,
        pull_up_name: str,
        active_state_name: str,
    ) -> None:
        """Validate gpiozero input mode compatibility."""
        if pull_up in (True, False) and active_state != -1:
            raise ValueError(
                f"'{active_state_name}' must be -1 when '{pull_up_name}' is true or false. "
                "gpiozero only supports custom active_state for floating inputs."
            )

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

    def _resolve_pin(self, pin_numbering: str, pin: int) -> PinType:
        """Resolve configured pin number to gpiozero pin identifier."""
        normalized = pin_numbering.strip().lower()
        if normalized == "bcm":
            return pin
        if normalized == "gpio":
            return f"GPIO{pin}"
        raise ValueError("Unsupported 'pin_numbering'. Use 'bcm' or 'gpio'.")

    def destroy_node(self) -> bool:
        self._registry.close()
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

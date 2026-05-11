"""Servo wrapper exposing `ros_common_srvs/SetFloat32` on `~/servo_outputs/<name>/set`."""

from typing import Any, Dict

from gpiozero import Servo as _GZServo
from ros_common_srvs.srv import SetFloat32

from rpi_gpiozero_ros.devices.base import (
    GPIODevice,
    float_from_spec,
    validate_positive_float,
)


def _map_range(value: float, from_range: tuple, to_range: tuple) -> float:
    """Remap a value from one range to another."""
    return (value - from_range[0]) * (to_range[1] - to_range[0]) / (from_range[1] - from_range[0]) + to_range[0]


class Servo(GPIODevice):
    """Servo output. Service input is in [0.0, 1.0] and remapped to gpiozero's [-1.0, 1.0]."""

    TYPE_NAME = "servo"

    def _build(self, spec: Dict[str, Any]) -> None:
        initial_value = float_from_spec(
            spec.get("initial_value", 0.0), f"{self.spec_prefix}.initial_value"
        )
        validate_positive_float(initial_value, f"{self.spec_prefix}.initial_value")
        initial_value = _map_range(initial_value, (0.0, 1.0), (-1.0, 1.0))

        min_pulse_width = float_from_spec(
            spec.get("min_pulse_width", 1.0 / 1000.0), f"{self.spec_prefix}.min_pulse_width"
        )
        max_pulse_width = float_from_spec(
            spec.get("max_pulse_width", 2.0 / 1000.0), f"{self.spec_prefix}.max_pulse_width"
        )
        frame_width = float_from_spec(
            spec.get("frame_width", 20.0 / 1000.0), f"{self.spec_prefix}.frame_width"
        )
        validate_positive_float(min_pulse_width, f"{self.spec_prefix}.min_pulse_width")
        validate_positive_float(max_pulse_width, f"{self.spec_prefix}.max_pulse_width")
        validate_positive_float(frame_width, f"{self.spec_prefix}.frame_width")
        if max_pulse_width <= min_pulse_width:
            raise ValueError(
                f"'{self.spec_prefix}.max_pulse_width' must be greater than "
                f"'{self.spec_prefix}.min_pulse_width'."
            )
        if frame_width <= max_pulse_width:
            raise ValueError(
                f"'{self.spec_prefix}.frame_width' must be greater than "
                f"'{self.spec_prefix}.max_pulse_width'."
            )

        self._device = _GZServo(
            pin=self.pin,
            initial_value=initial_value,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
            frame_width=frame_width,
        )
        self._service = self.node.create_service(
            SetFloat32, f"~/servo_outputs/{self.name}/set", self._handle_set
        )

    def _handle_set(
        self, request: SetFloat32.Request, response: SetFloat32.Response
    ) -> SetFloat32.Response:
        value = request.data
        if value < 0 or value > 1.0:
            response.success = False
            response.message = "Servo value must be in range [0.0, 1.0]"
            return response
        remapped_value = _map_range(value, (-1.0, 1.0), (0.0, 1.0))
        self._device.value = remapped_value
        response.success = True
        response.message = f"Set servo output '{self.name}' to {value:.3f} (remapped to {remapped_value:.3f})"
        return response

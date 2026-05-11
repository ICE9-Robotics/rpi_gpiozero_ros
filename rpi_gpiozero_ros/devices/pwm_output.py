"""PWMOutputDevice wrapper exposing `ros_common_srvs/SetFloat32` on `~/pwm_outputs/<name>/set`."""

from typing import Any, Dict

from gpiozero import PWMOutputDevice as _GZPWMOutputDevice
from ros_common_srvs.srv import SetFloat32

from rpi_gpiozero_ros.devices.base import (
    GPIODevice,
    bool_from_spec,
    float_from_spec,
    validate_positive_float,
    validate_unit_interval,
)


class PWMOutput(GPIODevice):
    """PWM GPIO output (duty cycle 0..1). Set via SetFloat32 service."""

    TYPE_NAME = "pwm_output"

    def _build(self, spec: Dict[str, Any]) -> None:
        active_high = bool_from_spec(spec.get("active_high", True), f"{self.spec_prefix}.active_high")
        initial_value = float_from_spec(
            spec.get("initial_value", 0.0), f"{self.spec_prefix}.initial_value"
        )
        validate_unit_interval(initial_value, f"{self.spec_prefix}.initial_value")
        frequency = float_from_spec(spec.get("frequency", 100.0), f"{self.spec_prefix}.frequency")
        validate_positive_float(frequency, f"{self.spec_prefix}.frequency")

        self._device = _GZPWMOutputDevice(
            pin=self.pin,
            active_high=active_high,
            initial_value=initial_value,
            frequency=frequency,
        )
        self._service = self.node.create_service(
            SetFloat32, f"~/pwm_outputs/{self.name}/set", self._handle_set
        )

    def _handle_set(
        self, request: SetFloat32.Request, response: SetFloat32.Response
    ) -> SetFloat32.Response:
        value = request.data
        if value < 0.0 or value > 1.0:
            response.success = False
            response.message = "PWM value must be in range [0.0, 1.0]"
            return response
        self._device.value = value
        response.success = True
        response.message = f"Set pwm output '{self.name}' to {value:.3f}"
        return response

"""DigitalOutputDevice wrapper exposing `std_srvs/SetBool` on `~/outputs/<name>/set`."""

from typing import Any, Dict

from gpiozero import DigitalOutputDevice as _GZDigitalOutputDevice
from std_srvs.srv import SetBool

from rpi_gpiozero_ros.devices.base import GPIODevice, bool_from_spec


class DigitalOutput(GPIODevice):
    """Boolean GPIO output. Set via SetBool service."""

    TYPE_NAME = "digital_output"

    def _build(self, spec: Dict[str, Any]) -> None:
        active_high = bool_from_spec(spec.get("active_high", True), f"{self.spec_prefix}.active_high")
        initial_value = bool_from_spec(
            spec.get("initial_value", False), f"{self.spec_prefix}.initial_value"
        )

        self._device = _GZDigitalOutputDevice(
            pin=self.pin,
            active_high=active_high,
            initial_value=initial_value,
        )
        self._service = self.node.create_service(
            SetBool, f"~/outputs/{self.name}/set", self._handle_set
        )

    def _handle_set(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            self._device.on()
        else:
            self._device.off()
        response.success = True
        response.message = f"Set digital output '{self.name}' to {request.data}"
        return response

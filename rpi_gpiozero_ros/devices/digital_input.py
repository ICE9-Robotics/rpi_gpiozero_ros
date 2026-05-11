"""DigitalInputDevice wrapper publishing `std_msgs/Bool` on `~/inputs/<name>/state`."""

from typing import Any, Dict

from gpiozero import DigitalInputDevice as _GZDigitalInputDevice
from std_msgs.msg import Bool

from rpi_gpiozero_ros.devices.base import (
    GPIODevice,
    optional_bool_from_spec,
    optional_non_negative_float_from_spec,
)


class DigitalInput(GPIODevice):
    """Boolean GPIO input. Publishes state at the node-global publish rate."""

    TYPE_NAME = "digital_input"

    def _build(self, spec: Dict[str, Any]) -> None:
        pull_up = optional_bool_from_spec(spec.get("pull_up"), f"{self.spec_prefix}.pull_up")
        if "pull_up" not in spec:
            pull_up = None if "active_state" in spec else True
        bounce_time = optional_non_negative_float_from_spec(
            spec.get("bounce_time"), f"{self.spec_prefix}.bounce_time"
        )
        active_state = optional_bool_from_spec(
            spec.get("active_state"), f"{self.spec_prefix}.active_state"
        )
        if pull_up is not None and active_state is not None:
            raise ValueError(
                f"'{self.spec_prefix}.active_state' must be omitted when "
                f"'{self.spec_prefix}.pull_up' is set. "
                "gpiozero only allows active_state for floating inputs."
            )

        self._device = _GZDigitalInputDevice(
            pin=self.pin,
            pull_up=pull_up,
            bounce_time=bounce_time,
            active_state=active_state,
        )
        self._publisher = self.node.create_publisher(Bool, f"~/inputs/{self.name}/state", 10)

    def tick(self) -> None:
        msg = Bool()
        msg.data = bool(self._device.value)
        self._publisher.publish(msg)

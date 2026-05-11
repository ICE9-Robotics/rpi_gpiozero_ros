"""SmoothedInputDevice wrapper publishing `std_msgs/Float32` on `~/smoothed_inputs/<name>/value`."""

from typing import Any, Dict

from gpiozero import SmoothedInputDevice as _GZSmoothedInputDevice
from std_msgs.msg import Float32

from rpi_gpiozero_ros.devices.base import (
    GPIODevice,
    bool_from_spec,
    float_from_spec,
    int_from_spec,
    validate_non_negative_float,
    validate_positive_int,
    validate_unit_interval,
)


class SmoothedInput(GPIODevice):
    """Smoothed analog-ish GPIO input. Publishes value at the node-global publish rate."""

    TYPE_NAME = "smoothed_input"

    def _build(self, spec: Dict[str, Any]) -> None:
        queue_len = int_from_spec(spec.get("queue_len", 5), f"{self.spec_prefix}.queue_len")
        validate_positive_int(queue_len, f"{self.spec_prefix}.queue_len")
        sample_wait = float_from_spec(spec.get("sample_wait", 0.0), f"{self.spec_prefix}.sample_wait")
        validate_non_negative_float(sample_wait, f"{self.spec_prefix}.sample_wait")
        threshold = float_from_spec(spec.get("threshold", 0.5), f"{self.spec_prefix}.threshold")
        validate_unit_interval(threshold, f"{self.spec_prefix}.threshold")
        partial = bool_from_spec(spec.get("partial", False), f"{self.spec_prefix}.partial")

        self._device = _GZSmoothedInputDevice(
            pin=self.pin,
            queue_len=queue_len,
            sample_wait=sample_wait,
            threshold=threshold,
            partial=partial,
            ignore=None,
        )
        self._publisher = self.node.create_publisher(
            Float32, f"~/smoothed_inputs/{self.name}/value", 10
        )

    def tick(self) -> None:
        msg = Float32()
        msg.data = float(self._device.value)
        self._publisher.publish(msg)

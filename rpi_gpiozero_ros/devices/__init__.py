"""gpiozero-backed ROS device wrappers.

Each device type lives in its own module and exposes a class that owns its
gpiozero device, parses its own spec fields, and creates its own ROS interface.
The `DEVICE_CLASSES` mapping is the single dispatch table used by the node.
"""

from typing import Dict, Type

from rpi_gpiozero_ros.devices.base import GPIODevice, PinType, configure_pin_factory
from rpi_gpiozero_ros.devices.digital_input import DigitalInput
from rpi_gpiozero_ros.devices.digital_output import DigitalOutput
from rpi_gpiozero_ros.devices.pwm_output import PWMOutput
from rpi_gpiozero_ros.devices.servo import Servo
from rpi_gpiozero_ros.devices.smoothed_input import SmoothedInput


DEVICE_CLASSES: Dict[str, Type[GPIODevice]] = {
    cls.TYPE_NAME: cls
    for cls in (DigitalInput, SmoothedInput, DigitalOutput, PWMOutput, Servo)
}


__all__ = [
    "DEVICE_CLASSES",
    "DigitalInput",
    "DigitalOutput",
    "GPIODevice",
    "PWMOutput",
    "PinType",
    "Servo",
    "SmoothedInput",
    "configure_pin_factory",
]

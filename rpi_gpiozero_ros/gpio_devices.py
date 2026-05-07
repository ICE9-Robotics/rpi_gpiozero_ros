"""gpiozero device registry and typed configuration models."""

from dataclasses import dataclass
from typing import Dict, Optional, Set, Union

from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice, Servo, SmoothedInputDevice
from gpiozero import Device
from gpiozero.pins.mock import MockFactory, MockPWMPin


PinType = Union[int, str]


@dataclass(frozen=True)
class DigitalInputConfig:
    """Configuration for one DigitalInputDevice."""

    name: str
    pin: PinType
    pull_up: Optional[bool]
    bounce_time: Optional[float]
    active_state: Optional[bool]


@dataclass(frozen=True)
class SmoothedInputConfig:
    """Configuration for one SmoothedInputDevice."""

    name: str
    pin: PinType
    queue_len: int
    sample_wait: float
    threshold: float
    partial: bool
    ignore: Optional[Set]


@dataclass(frozen=True)
class DigitalOutputConfig:
    """Configuration for one DigitalOutputDevice."""

    name: str
    pin: PinType
    active_high: bool
    initial_value: bool


@dataclass(frozen=True)
class PWMOutputConfig:
    """Configuration for one PWMOutputDevice."""

    name: str
    pin: PinType
    active_high: bool
    initial_value: float
    frequency: float


@dataclass(frozen=True)
class ServoOutputConfig:
    """Configuration for one Servo."""

    name: str
    pin: PinType
    initial_value: float
    min_pulse_width: float
    max_pulse_width: float
    frame_width: float


def configure_pin_factory(pin_factory_mode: str) -> None:
    """Configure gpiozero pin factory mode."""
    if pin_factory_mode == "auto":
        return
    if pin_factory_mode == "mock":
        # Use PWM-capable mock pins so PWMOutputDevice works in off-target tests.
        Device.pin_factory = MockFactory(pin_class=MockPWMPin)
        return
    raise ValueError("Unsupported pin_factory mode. Use 'auto' or 'mock'.")


class GPIODeviceRegistry:
    """Own and lifecycle-manage all gpiozero devices."""

    def __init__(self) -> None:
        self.digital_inputs: Dict[str, DigitalInputDevice] = {}
        self.smoothed_inputs: Dict[str, SmoothedInputDevice] = {}
        self.digital_outputs: Dict[str, DigitalOutputDevice] = {}
        self.pwm_outputs: Dict[str, PWMOutputDevice] = {}
        self.servo_outputs: Dict[str, Servo] = {}

    def add_digital_input(self, config: DigitalInputConfig) -> None:
        self.digital_inputs[config.name] = DigitalInputDevice(
            pin=config.pin,
            pull_up=config.pull_up,
            bounce_time=config.bounce_time,
            active_state=config.active_state,
        )

    def add_smoothed_input(self, config: SmoothedInputConfig) -> None:
        self.smoothed_inputs[config.name] = SmoothedInputDevice(
            pin=config.pin,
            queue_len=config.queue_len,
            sample_wait=config.sample_wait,
            threshold=config.threshold,
            partial=config.partial,
            ignore=config.ignore,
        )

    def add_digital_output(self, config: DigitalOutputConfig) -> None:
        self.digital_outputs[config.name] = DigitalOutputDevice(
            pin=config.pin,
            active_high=config.active_high,
            initial_value=config.initial_value,
        )

    def add_pwm_output(self, config: PWMOutputConfig) -> None:
        self.pwm_outputs[config.name] = PWMOutputDevice(
            pin=config.pin,
            active_high=config.active_high,
            initial_value=config.initial_value,
            frequency=config.frequency,
        )

    def add_servo_output(self, config: ServoOutputConfig) -> None:
        self.servo_outputs[config.name] = Servo(
            pin=config.pin,
            initial_value=config.initial_value,
            min_pulse_width=config.min_pulse_width,
            max_pulse_width=config.max_pulse_width,
            frame_width=config.frame_width,
        )

    def close(self) -> None:
        for device in self.digital_inputs.values():
            device.close()
        for device in self.smoothed_inputs.values():
            device.close()
        for device in self.digital_outputs.values():
            device.close()
        for device in self.pwm_outputs.values():
            device.close()
        for device in self.servo_outputs.values():
            device.close()

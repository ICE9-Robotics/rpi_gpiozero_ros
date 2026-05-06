# rpi_gpiozero_ros

ROS 2 Jazzy Python package for Raspberry Pi GPIO control using `gpiozero`.

## Supported gpiozero base classes

- `DigitalInputDevice`
- `SmoothedInputDevice`
- `DigitalOutputDevice`
- `PWMOutputDevice`

## Interfaces

- Publishes digital input state on `~/inputs/<name>/state` (`std_msgs/Bool`)
- Publishes smoothed input value on `~/smoothed_inputs/<name>/value` (`std_msgs/Float32`)
- Provides digital output service `~/outputs/<name>/set` (`std_srvs/SetBool`)
- Provides PWM output service `~/pwm_outputs/<name>/set` (`ros_common_srvs/SetFloat32`)

## Build

```bash
cd /path/to/workspace
colcon build --packages-select rpi5_gpiozero_ros
```

## Run

```bash
source install/setup.bash
ros2 launch rpi5_gpiozero_ros gpiozero_node.launch.py
```

## Parameter file

See `config/gpiozero_params.yaml`.

Use `pin_factory: mock` for off-target testing without Raspberry Pi GPIO hardware.

Use `pin_numbering` to select pin identifier format:

- `bcm` (default): pins are passed as BCM integers (for example `17`)
- `gpio`: pins are passed as explicit gpiozero GPIO labels (for example `GPIO17`)

## Per-device settings

Each setting can be either:

- a scalar (applied to all devices), or
- an array (per-device values aligned with `names`/`pins`).

Examples:

- `digital_input.pull_up: true` or `digital_input.pull_up: [true, false]`
- `digital_input.bounce_time: 0.01` or `digital_input.bounce_time: [0.01, 0.05]`
- `digital_output.initial_value: false` or `digital_output.initial_value: [false, true]`
- `pwm_output.frequency: 100.0` or `pwm_output.frequency: [100.0, 500.0]`

Validation rules:

- `digital_input.active_state` values must be `-1`, `0`, or `1`
- `smoothed_input.queue_len` must be `> 0`
- `smoothed_input.sample_wait` must be `>= 0.0`
- `smoothed_input.threshold` must be in `[0.0, 1.0]`
- `pwm_output.initial_value` must be in `[0.0, 1.0]`
- `pwm_output.frequency` must be `> 0.0`

# rpi_gpiozero_ros

ROS 2 Jazzy Python package for Raspberry Pi GPIO control using `gpiozero`.

## Dependencies
- [gpiozero](https://pypi.org/project/gpiozero/)
- [ros_common_srvs](https://github.com/ICE9-Robotics/ros_common_srvs)

## Supported gpiozero base classes

- `DigitalInputDevice`
- `SmoothedInputDevice`
- `DigitalOutputDevice`
- `PWMOutputDevice`
- `Servo`

## Interfaces

- Publishes digital input state on `~/inputs/<name>/state` (`std_msgs/Bool`)
- Publishes smoothed input value on `~/smoothed_inputs/<name>/value` (`std_msgs/Float32`)
- Provides digital output service `~/outputs/<name>/set` (`std_srvs/SetBool`)
- Provides PWM output service `~/pwm_outputs/<name>/set` (`ros_common_srvs/SetFloat32`)
- Provides servo output service `~/servo_outputs/<name>/set` (`ros_common_srvs/SetFloat32`)

## Build

```bash
cd /path/to/workspace
colcon build --packages-select rpi_gpiozero_ros
```

## Run

```bash
source install/setup.bash
ros2 launch rpi_gpiozero_ros gpiozero_node.launch.py
```

## Parameter file

Parameters live under the node name (for example `gpiozero_node`) in `ros__parameters:`. See [`config/gpiozero_params.yaml`](config/gpiozero_params.yaml).

### Device parameters

Declare each device under **`gpio.<logical_name>.<field>`**. In YAML this is normally a nested map `gpio:` with one key per `<logical_name>`. ROS stores these as flat dotted parameters (for example `gpio.button.pin`).

There must be at least one configured device (`type` + `pin` per logical name).

### Global parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pin_factory` | string | `auto` | `auto`: use gpiozero’s default backend. `mock`: [`MockFactory`](https://gpiozero.readthedocs.io/) with PWM-capable mock pins so `pwm_output` can run without a Pi. |
| `pin_numbering` | string | `bcm` | How each integer `pin` is turned into a [gpiozero pin specification](https://gpiozero.readthedocs.io/en/stable/recipes.html#pin-numbering). See next subsection. |
| `publish_rate_hz` | float | `20.0` | Period for publishing **all** digital and smoothed input topics (`1.0 / publish_rate_hz`). Must be `> 0`. |

If a device entry defines `publish_rate_hz`, it is **validated** (must be `> 0`) but publishing still uses the **global** `publish_rate_hz` timer only.

### `pin_numbering`

| Value | Example YAML `pin` | Passed to gpiozero |
|-------|---------------------|--------------------|
| `bcm` | `17` | `17` (integer BCM number) |
| `gpio` | `17` | `"GPIO17"` |
| `board` | `11` | `"BOARD11"` (header pin numbering) |
| `wpi` | `0` | `"WPI0"` ([WiringPi](https://wiringpi.com/pins/) style index; availability depends on backend) |

### `gpio.<logical_name>.*` — per-device fields

The map key `<logical_name>` becomes the device name in topics and services. Required for every entry: **`type`** and **`pin`**.

| `type` | Extra fields | Notes |
|--------|----------------|-------|
| `digital_input` | <ul><li><code>pull_up</code> (bool, optional)</li><li><code>bounce_time</code> (float, optional)</li><li><code>active_state</code> (bool, optional)</li><li><code>publish_rate_hz</code> (float, optional)</li></ul> | If `pull_up` is omitted and `active_state` is omitted, behaviour matches gpiozero pull-up default (`pull_up=true`). If you set `active_state`, you **must** omit `pull_up`; gpiozero does not allow both `pull_up=true/false` and a custom `active_state`. See [also](https://gpiozero.readthedocs.io/en/stable/api_input.html#inputdevice).|
| `smoothed_input` | <ul><li><code>queue_len</code> (int, default <code>5</code>)</li><li><code>sample_wait</code> (float, default <code>0.0</code>)</li><li><code>threshold</code> (float, default <code>0.5</code>)</li><li><code>partial</code> (bool, default <code>false</code>)</li><li><code>publish_rate_hz</code> (float, optional)</li></ul> | `queue_len` must be `> 0`; `threshold` in `[0.0, 1.0]`. |
| `digital_output` | <ul><li><code>active_high</code> (bool, default <code>true</code>)</li><li><code>initial_value</code> (bool, default <code>false</code>)</li></ul> | |
| `pwm_output` | <ul><li><code>active_high</code> (bool, default <code>true</code>)</li><li><code>initial_value</code> (float, default <code>0.0</code>)</li><li><code>frequency</code> (float, default <code>100.0</code>)</li></ul> | `initial_value` in `[0.0, 1.0]`; `frequency` must be `> 0`. On real Pi hardware, BCM **PWM-capable pins** restrictions still apply (same as gpiozero). |
| `servo` | <ul><li><code>initial_value</code> (float, default <code>0.0</code>)</li><li><code>min_pulse_width</code> (float, default <code>0.001</code>)</li><li><code>max_pulse_width</code> (float, default <code>0.002</code>)</li><li><code>frame_width</code> (float, default <code>0.02</code>)</li></ul> | `initial_value` and service setpoint in `[-1.0, 1.0]`; `min_pulse_width > 0`; `max_pulse_width > min_pulse_width`; `frame_width > max_pulse_width`. See [gpiozero Servo](https://gpiozero.readthedocs.io/en/stable/api_output.html#servo). |

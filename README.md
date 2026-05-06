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

## Interfaces

- Publishes digital input state on `~/inputs/<name>/state` (`std_msgs/Bool`)
- Publishes smoothed input value on `~/smoothed_inputs/<name>/value` (`std_msgs/Float32`)
- Provides digital output service `~/outputs/<name>/set` (`std_srvs/SetBool`)
- Provides PWM output service `~/pwm_outputs/<name>/set` (`ros_common_srvs/SetFloat32`)

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

See `config/gpiozero_params.yaml`.

Use `pin_factory: mock` for off-target testing without Raspberry Pi GPIO hardware.

Use `pin_numbering` to select pin identifier format:

- `bcm` (default): pins are passed as BCM integers (for example `17`)
- `gpio`: pins are passed as explicit gpiozero GPIO labels (for example `GPIO17`)

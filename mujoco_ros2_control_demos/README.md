# mujoco_ros2_control_demos

Demonstration examples for the `mujoco_ros2_control` package. This package provides ready-to-use examples that show how to use MuJoCo with ros2_control.

## Available Demos

### Basic Demo

Launch a simple robot with position controllers:

```bash
ros2 launch mujoco_ros2_control_demos demo.launch.py
```

This will start:
- MuJoCo simulation with visualization
- Robot state publisher
- Joint state broadcaster
- Position controllers for the robot

### PID Control Demo

Launch with PID control enabled (allows velocity and effort control modes):

```bash
ros2 launch mujoco_ros2_control_demos demo.launch.py use_pid:=true
```

### MJCF Generation Demo

Generate MJCF from URDF at runtime:

```bash
ros2 launch mujoco_ros2_control_demos demo.launch.py use_mjcf_from_topic:=true
```

### Transmission Demo

Test with transmission interfaces:

```bash
ros2 launch mujoco_ros2_control_demos demo.launch.py test_transmissions:=true
```

### Headless Mode

Run without visualization (useful for servers or CI):

```bash
ros2 launch mujoco_ros2_control_demos demo.launch.py headless:=true
```

## Launch Arguments

- `use_pid` (default: `false`) - Enable PID control for velocity/effort modes
- `headless` (default: `false`) - Run without visualization
- `use_mjcf_from_topic` (default: `false`) - Generate MJCF from URDF at runtime
- `test_transmissions` (default: `false`) - Enable transmission testing

## Package Contents

- `launch/` - Launch files for demonstrations
- `config/` - Controller and PID configuration files
- `demo_resources/` - Robot models (URDF, MJCF) and scenes

## Controlling the Robot

After launching a demo, you can control the robot using ros2 topic commands:

```bash
# Set joint positions
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5]"

# Monitor joint states
ros2 topic echo /joint_states
```

## See Also

- Main package: [mujoco_ros2_control](../mujoco_ros2_control/)
- Tests: [mujoco_ros2_control_tests](../mujoco_ros2_control_tests/)

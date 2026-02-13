# mujoco_ros2_control_demos

Demonstration tutorials for the `mujoco_ros2_control` package. This package provides ready-to-use tutorials that show how to use MuJoCo with ros2_control.

## Tutorials

This package includes 4 progressive tutorials demonstrating different aspects of MuJoCo ros2_control integration.

### Tutorial 1: Basic Robot

The simplest setup - launches a two-link arm with position controllers using a pre-defined MJCF model.

```bash
ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py
```

**Key concepts:** Pre-defined MJCF loading, basic ros2_control integration, position control

**Resources:** `demo_resources/scenes/scene.xml`, `demo_resources/robot/test_robot.xml`

> [!TIP]
> UI panels can be toggled with `Tab` or `Shift+Tab`.
> All standard MuJoCo keyboard shortcuts are available.
> To see a short list, press `F1`.

---

### Tutorial 2: MJCF Generation at Runtime

Demonstrates generating MJCF models from URDF at runtime using the conversion script.

```bash
# Using external input files
ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py

# Using mujoco_inputs embedded in URDF
ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py use_urdf_inputs:=true
```

**Key concepts:** Runtime URDF→MJCF conversion, `<mujoco_inputs>` tags, external input files

**Resources:** `demo_resources/mjcf_generation/test_inputs.xml`, `demo_resources/scenes/scene_info.xml`

---

### Tutorial 3: PID Control

Demonstrates PID controllers with motor actuators for velocity/effort control modes.

```bash
ros2 launch mujoco_ros2_control_demos 03_pid_control.launch.py
```

**Key concepts:** PID gain configuration, motor actuators, velocity/effort control

**Resources:** `demo_resources/pid_control/test_robot_pid.xml`, `config/mujoco_pid.yaml`

---

### Tutorial 4: Transmissions

Demonstrates ros2_control transmissions with mechanical reduction ratios.

```bash
ros2 launch mujoco_ros2_control_demos 04_transmissions.launch.py
```

**Key concepts:** SimpleTransmission interface, mechanical reduction, actuator-to-joint mapping

**Resources:** Uses `demo_resources/robot/test_robot.urdf` with `use_transmissions:=true`

---

## Combined Demo

For backward compatibility and combined feature testing:

```bash
ros2 launch mujoco_ros2_control_demos demo.launch.py
ros2 launch mujoco_ros2_control_demos demo.launch.py use_pid:=true
ros2 launch mujoco_ros2_control_demos demo.launch.py use_mjcf_from_topic:=true
ros2 launch mujoco_ros2_control_demos demo.launch.py test_transmissions:=true
```

## Headless Mode

All tutorials support headless mode:

```bash
ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py headless:=true
```

## Controlling the Robot

```bash
# Set joint positions (joint1, joint2)
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5]"

# Control the gripper
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.02]"

# Monitor joint states
ros2 topic echo /joint_states
```

## Package Structure

```
mujoco_ros2_control_demos/
├── launch/
│   ├── 01_basic_robot.launch.py      # Tutorial 1
│   ├── 02_mjcf_generation.launch.py  # Tutorial 2
│   ├── 03_pid_control.launch.py      # Tutorial 3
│   ├── 04_transmissions.launch.py    # Tutorial 4
│   └── demo.launch.py                # Combined demo
├── config/
│   ├── controllers.yaml              # Controller configuration
│   └── mujoco_pid.yaml               # PID gains (Tutorial 3)
└── demo_resources/
    ├── robot/
    │   ├── test_robot.urdf           # Shared URDF description
    │   └── test_robot.xml            # MJCF robot model
    ├── scenes/
    │   ├── scene.xml                 # Basic scene (Tutorial 1, 4)
    │   ├── scene_pid.xml             # PID scene (Tutorial 3)
    │   └── scene_info.xml            # Scene generation info
    ├── mjcf_generation/
    │   └── test_inputs.xml           # MJCF conversion inputs (Tutorial 2)
    └── pid_control/
        └── test_robot_pid.xml        # Robot with motor actuators
```

## See Also

- Main package: [mujoco_ros2_control](../mujoco_ros2_control/)
- Tests: [mujoco_ros2_control_tests](../mujoco_ros2_control_tests/)

# Drive Bringup Package

Launch files and configuration for Athena drive system (hardware and simulation).

## Launch Files

### athena_drive.launch.py
Main launch file for **hardware** operation. Orchestrates robot description, controllers, and optional teleop.

**Key Arguments:**
- `robot_controller` (default: `single_ackermann_controller`): Active controller to start
- `enable_teleop` (default: `true`): Enable joystick/teleop nodes
- `use_mock_hardware` (default: `false`): Use mock hardware instead of real CAN
- `use_sim` (default: `true`): Start RViz

**Usage:**
```bash
# Hardware with teleop
ros2 launch drive_bringup athena_drive.launch.py

# Hardware without teleop (autonomous)
ros2 launch drive_bringup athena_drive.launch.py enable_teleop:=false

# Mock hardware for testing
ros2 launch drive_bringup athena_drive.launch.py use_mock_hardware:=true
```

### athena_drive_sim.launch.py
Launch file for **Gazebo simulation**. Starts Gazebo, spawns robot, and manages controllers.

**Key Arguments:**
- `world` (default: `empty.sdf`): Gazebo world to load
- `robot_controller` (default: `ackermann_steering_controller`): Controller to use
- `enable_teleop` (default: `false`): Enable manual control in sim
- `rviz` (default: `false`): Start RViz

**Usage:**
```bash
# Basic simulation
ros2 launch drive_bringup athena_drive_sim.launch.py

# Simulation with RViz
ros2 launch drive_bringup athena_drive_sim.launch.py rviz:=true

# Simulation with teleop
ros2 launch drive_bringup athena_drive_sim.launch.py enable_teleop:=true

# Custom world
ros2 launch drive_bringup athena_drive_sim.launch.py world:=mars_terrain.sdf
```

### controllers.launch.py
**Modular** controller spawning. Can be included by other launch files or run standalone.

**Spawns:**
- `joint_state_broadcaster` (always)
- Active controller (specified by `robot_controller` arg)
- Inactive controllers for runtime switching (ackermann, velocity, position)
- Controller switcher service (optional)

**Usage:**
```bash
# Standalone testing
ros2 launch drive_bringup controllers.launch.py robot_controller:=ackermann_steering_controller

# Without controller switcher
ros2 launch drive_bringup controllers.launch.py start_controller_switcher:=false
```

### teleop.launch.py
**Modular** teleoperation nodes. Can be included by other launch files or run standalone.

**Spawns:**
- Joystick publisher node
- teleop_twist_joy node

**Usage:**
```bash
# Standalone teleop
ros2 launch drive_bringup teleop.launch.py

# Custom config
ros2 launch drive_bringup teleop.launch.py joystick_config:=my_joystick.yaml
```

## Architecture

```
drive_bringup/
├── launch/
│   ├── athena_drive.launch.py          # Hardware orchestrator
│   ├── athena_drive_sim.launch.py      # Simulation orchestrator
│   ├── controllers.launch.py           # Modular controller spawning
│   └── teleop.launch.py                # Modular teleop nodes
├── config/
│   ├── athena_drive_controllers.yaml   # Unified controller config (hw + sim)
│   ├── joystick.yaml
│   └── teleop_twist.yaml
└── README.md
```

## Configuration

### Unified Controllers
The package uses **one controller configuration** (`athena_drive_controllers.yaml`) for both hardware and simulation, eliminating config drift.

All controllers defined:
- `single_ackermann_controller`: Custom Ackermann with 4-wheel drive
- `ackermann_steering_controller`: Standard Ackermann (2-wheel rear drive)
- `drive_velocity_controller`: Direct velocity control
- `drive_position_controller`: Direct position control

### Controller Switching

Use the controller switcher service for runtime switching:

```bash
# Switch to velocity controller
ros2 service call /switch_controller drive_bringup/SwitchController "{target_controller: 'drive_velocity_controller'}"

# Switch to ackermann
ros2 service call /switch_controller drive_bringup/SwitchController "{target_controller: 'ackermann_steering_controller'}"
```

## Hardware vs Simulation

| Feature | Hardware | Simulation |
|---------|----------|------------|
| Launch file | `athena_drive.launch.py` | `athena_drive_sim.launch.py` |
| Default controller | `single_ackermann_controller` | `ackermann_steering_controller` |
| Teleop default | Enabled | Disabled |
| RViz default | Enabled | Disabled (use arg) |
| Controller config | `athena_drive_controllers.yaml` | `athena_drive_controllers.yaml` |
| Wheel drive | Rear (bl, br) | Rear (bl, br) ✓ unified |

## Dependencies

- `controller_manager`
- `ros2_control`
- `athena_drive_controllers` (custom controllers)
- `ros_gz_sim` (simulation only)
- `ros_gz_bridge` (simulation only)
- `simulation` package (simulation only)

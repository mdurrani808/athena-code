# Drive Bringup Package

Launch files and configuration for Athena drive system (hardware and simulation).

## Quick Start

**Simulation:**
```bash
ros2 launch drive_bringup athena_drive.launch.py sim:=true
```

**Hardware:**
```bash
ros2 launch drive_bringup athena_drive.launch.py sim:=false
```

## Launch Files

### athena_drive.launch.py
**Unified** launch file for drive subsystem. Supports both simulation and hardware via `sim` argument.

**Key Arguments:**
- `sim` (default: `false`): Simulation vs hardware mode
- `world` (default: `empty.sdf`): Gazebo world file (sim only)
- `robot_controller` (default: auto): Controller to use (sim=ackermann_steering, hw=single_ackermann)
- `enable_teleop` (default: auto): Enable teleop (sim=false, hw=true)
- `rviz` (default: `true`): Start RViz
- `use_mock_hardware` (default: `false`): Use mock hardware (hw mode only)

**Usage:**

```bash
# Simulation with RViz
ros2 launch drive_bringup athena_drive.launch.py sim:=true rviz:=true

# Simulation with custom world
ros2 launch drive_bringup athena_drive.launch.py sim:=true world:=mars_terrain.sdf

# Simulation with teleop
ros2 launch drive_bringup athena_drive.launch.py sim:=true enable_teleop:=true

# Hardware with teleop (default)
ros2 launch drive_bringup athena_drive.launch.py sim:=false

# Hardware without teleop (autonomous)
ros2 launch drive_bringup athena_drive.launch.py sim:=false enable_teleop:=false

# Mock hardware for testing
ros2 launch drive_bringup athena_drive.launch.py sim:=false use_mock_hardware:=true

# Different controller
ros2 launch drive_bringup athena_drive.launch.py robot_controller:=ackermann_steering_controller
```

### controllers.launch.py
**Modular** controller spawning. Can be included by other launch files or run standalone.

**Spawns:**
- `joint_state_broadcaster` (always)
- Active controller (specified by `robot_controller` arg)
- Inactive controllers for runtime switching
- Controller switcher service (optional)

**Usage:**
```bash
ros2 launch drive_bringup controllers.launch.py robot_controller:=ackermann_steering_controller
```

### teleop.launch.py
**Modular** teleoperation nodes. Can be included by other launch files or run standalone.

**Spawns:**
- Joystick publisher node
- teleop_twist_joy node

**Usage:**
```bash
ros2 launch drive_bringup teleop.launch.py
```

## Architecture

```
drive_bringup/
├── launch/
│   ├── athena_drive.launch.py          # Unified sim/hardware launcher
│   ├── controllers.launch.py           # Modular controller spawning
│   └── teleop.launch.py                # Modular teleop nodes
├── config/
│   ├── athena_drive_controllers.yaml   # Unified controller config
│   ├── joystick.yaml
│   └── teleop_twist.yaml
└── README.md
```

## How It Works

The unified `athena_drive.launch.py` conditionally includes different infrastructure based on `sim` argument:

**Simulation Mode (sim:=true):**
1. Includes `simulation/gz_sim.launch.py` (starts Gazebo)
2. Includes `simulation/bridge.launch.py` (ROS-Gazebo topic bridges)
3. Spawns robot in Gazebo via `robot_state_publisher` + `ros_gz_sim/create`
4. Includes `controllers.launch.py` (spawns controllers)
5. Optionally includes `teleop.launch.py`
6. Optionally starts RViz

**Hardware Mode (sim:=false):**
1. Starts `ros2_control_node` (manages hardware interfaces)
2. Starts `robot_state_publisher` (publishes robot state)
3. Delays controller spawning until hardware is ready
4. Includes `controllers.launch.py` (spawns controllers)
5. Optionally includes `teleop.launch.py` (default: enabled)
6. Optionally starts RViz

## Configuration

### Unified Controllers
The package uses **one controller configuration** (`athena_drive_controllers.yaml`) for both hardware and simulation, eliminating config drift.

All controllers defined:
- `single_ackermann_controller`: Custom Ackermann with 4-wheel drive (default for hardware)
- `ackermann_steering_controller`: Standard Ackermann with 2-wheel rear drive (default for sim)
- `drive_velocity_controller`: Direct velocity control
- `drive_position_controller`: Direct position control

### Conditional Defaults

The launch file automatically sets appropriate defaults based on mode:

| Argument | Simulation Default | Hardware Default |
|----------|-------------------|------------------|
| `robot_controller` | `ackermann_steering_controller` | `single_ackermann_controller` |
| `enable_teleop` | `false` | `true` |
| `use_sim_time` | `true` | `false` |

Override these by explicitly passing arguments.

### Controller Switching

Use the controller switcher service for runtime switching:

```bash
# Switch to velocity controller
ros2 service call /switch_controller drive_bringup/SwitchController "{target_controller: 'drive_velocity_controller'}"

# Switch to ackermann
ros2 service call /switch_controller drive_bringup/SwitchController "{target_controller: 'ackermann_steering_controller'}"
```

## Integration with Top-Level Launcher

This package can be launched independently or as part of the complete robot:

**Independent (just drive):**
```bash
ros2 launch drive_bringup athena_drive.launch.py sim:=true
```

**Integrated (full robot):**
```bash
ros2 launch bringup athena.launch.py sim:=true
```

The top-level `bringup/athena.launch.py` includes this package and passes through arguments.

## Subsystem Pattern

This package follows the standard subsystem pattern:

1. **Single Launch File**: `athena_<subsystem>.launch.py` with `sim:=true/false`
2. **Conditional Infrastructure**: Different nodes/includes based on `sim` argument
3. **Modular Components**: Reusable launch files for controllers, teleop, etc.
4. **Unified Config**: Single config file for both sim and hardware
5. **Conditional Defaults**: Smart defaults that change based on mode

This pattern is designed to be replicated for other subsystems (arm, perception, etc.).

## Dependencies

- `controller_manager`
- `ros2_control`
- `athena_drive_controllers` (custom controllers)
- `ros_gz_sim` (simulation only)
- `ros_gz_bridge` (simulation only)
- `simulation` package (simulation only)
- `description` package (robot URDF/xacro files)

# Athena Bringup Package

Top-level orchestration for the complete Athena rover system.

## Purpose

This package provides **system-level** launch files that orchestrate multiple subsystems (drive, arm, sensors, etc.) together. It serves as the single entry point for launching the complete robot in simulation or hardware mode.

## Launch Files

### athena.launch.py
Main entry point for the complete Athena rover system.

**Key Arguments:**
- `sim` (default: `false`): Launch in simulation (Gazebo) vs hardware mode
- `world` (default: `empty.sdf`): Gazebo world file (sim only)
- `rviz` (default: `true`): Start RViz for visualization
- `enable_drive` (default: `true`): Enable drive subsystem
- `enable_arm` (default: `false`): Enable arm subsystem (TODO: not yet implemented)
- `enable_teleop` (default: ""): Enable teleoperation (empty = subsystem defaults)
- `namespace` (default: ""): Robot namespace for multi-robot scenarios

**Usage:**

```bash
# Complete robot in simulation
ros2 launch bringup athena.launch.py sim:=true

# Complete robot in simulation with custom world
ros2 launch bringup athena.launch.py sim:=true world:=mars_terrain.sdf

# Complete robot on hardware
ros2 launch bringup athena.launch.py sim:=false

# Hardware without teleoperation (autonomous mode)
ros2 launch bringup athena.launch.py sim:=false enable_teleop:=false

# Just drive subsystem in sim
ros2 launch bringup athena.launch.py sim:=true enable_arm:=false

# Complete robot with RViz disabled
ros2 launch bringup athena.launch.py sim:=true rviz:=false
```

## Architecture

```
athena-code/
├── src/
│   ├── bringup/                        # TOP-LEVEL ORCHESTRATION
│   │   └── launch/
│   │       └── athena.launch.py        # Full robot launcher
│   │
│   ├── simulation/                     # Generic Gazebo infrastructure
│   │   └── launch/
│   │       ├── gz_sim.launch.py        # Gazebo launcher
│   │       └── bridge.launch.py        # ROS-Gazebo bridges
│   │
│   └── subsystems/                     # Individual subsystems
│       ├── drive/
│       │   └── drive_bringup/
│       │       └── launch/
│       │           ├── athena_drive.launch.py     # sim:=true/false
│       │           ├── controllers.launch.py
│       │           └── teleop.launch.py
│       │
│       └── arm/                        # TODO: Future subsystem
│           └── arm_bringup/
│               └── launch/
│                   └── athena_arm.launch.py       # sim:=true/false
```

## Design Principles

### 1. Single Entry Point
- `athena.launch.py` is the **only** launch file users need to know about
- One command launches the entire robot
- No confusion about which launch file to use

### 2. Subsystem Independence
- Each subsystem has its own `<subsystem>_bringup` package
- Subsystems can be launched independently via their own launch files
- Subsystems are included, not embedded, in the top-level launcher

### 3. Sim/Hardware Unification
- Every subsystem launch file supports `sim:=true/false`
- No separate `_sim.launch.py` files
- Reduces duplication and configuration drift

### 4. Composability
- Subsystems can be enabled/disabled individually
- Arguments flow from top-level to subsystems
- Multiple robots can run with different namespaces

### 5. Extensibility
- Adding new subsystems:
  1. Create `<subsystem>_bringup/launch/<subsystem>.launch.py` with `sim` argument
  2. Add include in `athena.launch.py`
  3. Add `enable_<subsystem>` argument
- No modification to simulation infrastructure needed

## Subsystem Integration

Each subsystem should follow this pattern:

**subsystem_bringup/launch/athena_<subsystem>.launch.py:**
```python
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("sim", default_value="false", ...),
        # Other subsystem-specific arguments
    ]

    sim = LaunchConfiguration("sim")

    # Simulation infrastructure (if sim=true)
    gazebo_launch = IncludeLaunchDescription(
        ..., # Include simulation/gz_sim.launch.py
        condition=IfCondition(sim)
    )

    # Hardware infrastructure (if sim=false)
    hardware_node = Node(
        ...,
        condition=UnlessCondition(sim)
    )

    # Common components (both modes)
    controllers_launch = IncludeLaunchDescription(...)

    return LaunchDescription([...])
```

## Multi-Robot Support

The architecture supports multiple robots via namespaces:

```bash
# Robot 1
ros2 launch bringup athena.launch.py sim:=true namespace:=robot1

# Robot 2
ros2 launch bringup athena.launch.py sim:=true namespace:=robot2
```

Each robot gets its own namespace for topics, services, and TF frames.

## Development Workflow

**Testing individual subsystem:**
```bash
ros2 launch drive_bringup athena_drive.launch.py sim:=true
```

**Testing full robot:**
```bash
ros2 launch bringup athena.launch.py sim:=true
```

**Deploying to hardware:**
```bash
ros2 launch bringup athena.launch.py sim:=false
```

## Future Subsystems

Planned subsystems following this pattern:
- `arm_bringup` - Manipulator arm
- `perception_bringup` - Cameras, LIDAR, perception stack
- `navigation_bringup` - Autonomous navigation
- `science_bringup` - Science instruments

Each will have:
- `athena_<subsystem>.launch.py` with `sim:=true/false`
- Modular component launches (controllers, sensors, etc.)
- Integration point in `athena.launch.py`

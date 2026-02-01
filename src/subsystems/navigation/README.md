# Navigation

## How To Launch Navigation Simulation

After building the relevant packages: 


```bash
ros2 launch simulation bringup.launch.py
```
With the following options:
* `rviz:={true,false}`: Launch rviz visualization
* `publish_ground_truth_tf:={true,false}`: Use the Gazebo odometry for the ground truth odom to base transform
* `world:={empty.sdf, terrain_world.sdf}: Use specific SDF file



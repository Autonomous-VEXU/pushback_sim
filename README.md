# pushback_sim
Simulation world of the VEX Push Back field + a few test envionments using ROS2 Jazzy Jalisco & Gazebo Harmonic. This package only contains world files. See the repository `Autonomous-VEXU/vex_robot` for robot models + spawners.

## Required Packages
- ros_gz_sim
- turtlebot3_gazebo
- turtlebot3_description
- turtlebot3
- xacro

## Launching a World
In order to launch a world, the workspace must first be built and sourced. Make sure you are in the correct directory before running the commands: `colcon build --symlink-install` and then `source install/setup.bash`.

Here is a general command to launch a specific world: </br>
`ros2 launch pushback_sim world_select.launch.py world:=<world>`

> Note: The `</world>` tag is where you put the name of the world that you want to launch.

## World Guide + Descriptions
`block_test`: 3 blocks with various visual and collision geometries</br>
`collision_spheres`: VEX Field with octocube visual meshes + sphere collision meshes</br>
`empty`: Just as it sounds, a completely empty world </br>
`lidar_test`: Asymmetric field used for testing simulated LiDAR sensors</br>
`pushback_spheres`: VEX Push Back full field with sphere primatives for blocks</br>
`pushback_no_blocks`: VEX Push Back field with no blocks</br>
`pushback`: VEX Push Back field set up to competition standards</br>

## Main File Structure:
```
pushback_sim/
├── launch/
│   ├── basic_field.launch.py
│   ├── tb3_field.launch.py
│   └── world_select.launch.py
├── models/
│   ├── blue-sphere
│   ├── clear-objects
│   ├── lidar-test-field
│   ├── red-sphere
│   ├── vex-ball-blue
│   ├── vex-ball-csb
│   ├── vex-ball-csr
│   ├── vex-ball-red
│   └── vex-field
├── worlds/
│   ├── block_test.sdf
│   ├── collision_spheres.sdf
│   ├── empty.sdf
│   ├── lidar_test.sdf
│   ├── pushback_comp.sdf
│   ├── pushback_v2.sdf
│   └── using_full_spheres.sdf
├── CMakeLists.txt
├── package.xml
└── resources.txt
```

#### Model Sub-Directory File Structure:
```
models/
└── model-name/
    ├── meshes/
    │   ├── model-part.dae
    │   └── model-part-collision.dae
    ├── model.config
    └── model.sdf
```

## ROS Resources:
[ROS2 Jazzy Jalisco Documentation](https://docs.ros.org/en/jazzy/index.html)</br>
[ROS Index](https://index.ros.org/?search_packages=true#jazzy)</br>
[Nav2 Documentation](https://docs.nav2.org)</br>
[Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted)</br>
[Open Robotics Discourse](https://discourse.openrobotics.org)</br>
[Robotics Stack Exchange](https://robotics.stackexchange.com)
[Simulation Description Format (SDF)](http://sdformat.org)
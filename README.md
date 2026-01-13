# pushback_sim
Simulation worlds, maps, CAD models, and game behavior nodes for the VEX Push Back field + a few test envionments. See the repository `Autonomous-VEXU/otto_gazebo` for robot simulation assets.

> Note: If you want to prevent infinite rolling, 
## Launching a World
In order to launch a world, the workspace must first be built and sourced. Make sure you are in the correct directory before running the commands: `colcon build --symlink-install` and then `source install/setup.bash`.

Here is a general command to launch a specific world: </br>
`ros2 launch pushback_sim world_select.launch.py world:=<world>`

> Note: `<world>` is where you put the name of the world that you want to launch.

## World Guide + Descriptions
`block_test`: Empty world with one of each block model</br>
`empty`: Just as it sounds, a completely empty world </br>
`sensor_test`: Asymmetric field used for testing sensors setups</br>
`empty_field`: An empty field with no blocks </br>
`pushback`: VEX Push Back field with all blocks in place</br>

## Main File Structure:
```
pushback_sim/
├── launch/
│   ├── basic_field.launch.py
│   ├── tb3_field.launch.py
│   └── world_select.launch.py
├── maps/
│   ├── vex_field_map.pgm
│   └── vex_field_map.yaml
├── models/
│   ├── blue-sphere
│   ├── clear-objects
│   ├── lidar-test-field
│   ├── red-sphere
│   └── vex-field
├── worlds/
│   ├── block_test.sdf
│   ├── empty_field.sdf
│   ├── empty.sdf
│   ├── pushback.sdf
│   └── sensor_test.sdf
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

## Rolling Friction Plugin Setup

Both the blue and red spheres use a gazebo plugin called `rollingFriction` the plugin + install instructions can be found here: [kmhswimgirl/gz_rolling_friction](https://github.com/kmhswimgirl/gz_rolling_friction/tree/main)

## ROS + Gazebo Sim Resources:
[ROS2 Jazzy Jalisco Documentation](https://docs.ros.org/en/jazzy/index.html)</br>
[ROS Index](https://index.ros.org/?search_packages=true#jazzy)</br>
[Nav2 Documentation](https://docs.nav2.org)</br>
[Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted)</br>
[Open Robotics Discourse](https://discourse.openrobotics.org)</br>
[Robotics Stack Exchange](https://robotics.stackexchange.com)</br>
[Simulation Description Format (SDF)](http://sdformat.org) </br>
[RGBA 0-1 Color Picker](https://rgbcolorpicker.com/0-1)
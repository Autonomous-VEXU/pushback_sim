# pushback_sim
Simulation worlds, maps, CAD models, and game behavior nodes for the VEX Push Back field + a few test envionments. See the repository `Autonomous-VEXU/otto_gazebo` for robot simulation assets.

> Note: If you want to prevent infinite rolling, refer to [this section](#rolling-friction-plugin-setup)

## Getting Started
In order to launch a world or really any launch file, the workspace must first be built and sourced. Make sure you are in the correct directory before running the commands: `colcon build --symlink-install` and then `source install/setup.bash`.

### Launching a World (basic)
Here is a general command to launch a specific world: </br>
```bash
ros2 launch pushback_sim world_select.launch.py world:=<world>
```
> Note: `<world>` is where you put the name of the world that you want to launch.

### Running the Full Simulation (advanced)
This launch file is the top level launch file for this package and is highly configurable through launch arguments.
> Documentation can be found [here](#full_simulationlaunchpy)
```bash
ros2 launch pushback_sim full_simulation.launch.py
```
## Table of Contents:
- [Available Worlds](#worlds--descriptions)
- [Package File Tree](#package-file-structure)
- [Launch Files](#launch-files)
- [Nodes](#nodes)
- [Rolling Friction Plugin](#rolling-friction-plugin-setup)
- [ROS + Gazebo Sim Resources](#ros--gazebo-sim-resources)

## Worlds + Descriptions
`block_test`: Empty world with one of each block model</br>
`empty`: Just as it sounds, a completely empty world </br>
`empty_field`: VEX Push Back field with no blocks</br>
`pushback`: VEX Push Back field set up to competition standards</br>
`sensor_test`: Asymmetric field used for testing sensors setups in sim</br>

## Package File Structure:
```
pushback_sim/
├── launch/
│   ├── full_simulation.launch.py
│   ├── opponent.launch.py
│   ├── sim_backend.launch.py
│   ├── strategy_ai.launch.py
│   ├── tb3_field.launch.py
│   └── world_select.launch.py
├── maps/
│   ├── keepout_full_goal.pgm
│   ├── keepout_full_goal.yaml
│   ├── vex_field_map.pgm
│   └── vex_field_map.yaml
├── models/
│   ├── blue-sphere
│   ├── clear-objects
│   ├── lidar-test-field
│   ├── red-sphere
│   └── vex-field
├── src/
│   ├── ai_driver.py
│   ├── field_location.py
│   ├── opponent.py
│   ├── pose_bridge.py
│   ├── scoring.py
│   ├── strategy_ai_bridge.py
│   └── world_services.py
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
## Launch Files
### `full_simulation.launch.py`
Launches the full simulation. This is the top level launch file for this package.
Arguments:
- `keepout_filter`: toggles using the keepout filter for the goals
- `opponent`: toggles opponent spawning into world + other nodes launching
- `teleop`: conditionally launches teleop control
- `world_ctrl`: toggles simulation post tracking and backend services
- `nav2`: toggles nav2 mppi controller
- `sai`: toggles the strategy AI model

### `opponent.launch.py`
Launches the opponent node and gazebo bridge node.

### `sim_backend.launch.py`
Launches all of the nodes that deal with simulaton logic/making the game playable. Includes `field_location.py`, `pose_bridge.py`, `scoring.py`, and `world_services.py`.

### `strategy_ai.launch.py`
Launches the sai (strategy AI) node 
Arguments:
- `ai_delay`: mount of time to delay launching the strategy AI model

### `tb3_field.launch.py`
Turtlebot3 + one of the worlds.
Arguments:
- `x_pose`: spawn in at a specific X coordinate
- `y_pose`: spawn in at a specific Y coordinate
- `z_pose`: spawn in at a specific Z coordinate

### `windows.launch.py`
Launches the sim for Windows :(

### `world_select.launch.py`
Selects a world SDF file to load in Gazebo Sim.
Arguments:
- `world`: the world filename without the '.sdf' 

## Nodes
### `ai_driver.py`
Interprets the output of the strategy AI node and calls actions to control Otto. Just uses the Nav2 move to pose action for now.

### `field_location.py`
Determines if an action (i.e. picking up a ball) can occur based on Otto's location and orientation. Basically collision checking for goal hitboxes to vaildate scoring.

### `opponent.py`
Spawns in and randomly moves a box to predetermined points to simulate an opponent for the strategy AI node.

### `pose_bridge.py`
Purpose of this node is to listen to the gazebo topic `/world/default/dynamic_pose/info`, parse the data (JSON) reformat it to include the name of the model and its ID number and republishes it on the `/object_locations` topic.

### `scoring.py`
Takes in `vex_interfaces/Goal` and returns the overall score. Includes control zones. 

### `strategy_ai_bridge.py`
Collected world state information from various topics, packaged them into a `vex_interfaces/WorldState` message, and published them to a topic that the strategy AI node subscribes to.

### `world_services.py`
Exposes a few services for adding things to loaders, intaking a ball, and outputting a ball (determines if ball falls on the ground or is scored). Main purpose is to communicate with the gazebo sim topic to manipulate entites.

## Rolling Friction Plugin Setup 
Both the blue and red spheres use a gazebo plugin called `rolling_friction::RollingFrictionPlugin` the plugin + install instructions can be found here: [kymadogg/gz_rolling_friction](https://github.com/kymadogg/gz_rolling_friction/tree/main)

## ROS + Gazebo Sim Resources:
[ROS2 Jazzy Jalisco Documentation](https://docs.ros.org/en/jazzy/index.html)</br>
[ROS Index](https://index.ros.org/?search_packages=true#jazzy)</br>
[Nav2 Documentation](https://docs.nav2.org)</br>
[Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted)</br>
[Open Robotics Discourse](https://discourse.openrobotics.org)</br>
[Robotics Stack Exchange](https://robotics.stackexchange.com)</br>
[Simulation Description Format (SDF)](http://sdformat.org) </br>
[RGBA 0-1 Color Picker](https://rgbcolorpicker.com/0-1)
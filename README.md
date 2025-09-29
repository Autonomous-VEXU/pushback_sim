## pushback_sim

Simulation of the VEX Push Back field using ROS2 Jazzy Jalisco & Gazebo Harmonic.

**Useful links:** </br>
SDF Docs: http://sdformat.org

Main File Tree:
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

Model Directory File Tree:
```
models/
└── model-name/
    ├── meshes/
    │   ├── model-part.dae
    │   └── model-part-collision.dae
    ├── model.config
    └── model.sdf
```
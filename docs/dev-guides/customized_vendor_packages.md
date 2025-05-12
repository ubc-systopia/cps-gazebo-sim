### Customized Vendor Packages

Dependencies on customized vendor packages can be resolved by dynamically fetching the source code locally using [vcstool](https://github.com/dirk-thomas/vcstool).

The [`.repos` folder](https://github.com/ubc-systopia/cps-gazebo-sim/tree/bfd1554c8d77eb68e82401cbdf53d5913e8e4c7a/.repos) contains a `.repos` file for each vendor. Running the [`setup.sh`](https://github.com/ubc-systopia/cps-gazebo-sim/blob/bfd1554c8d77eb68e82401cbdf53d5913e8e4c7a/setup.sh) script will import the specified repositories into the corresponding subdirectories under [`ros2_ws/src/vendors/`](https://github.com/ubc-systopia/cps-gazebo-sim/tree/bfd1554c8d77eb68e82401cbdf53d5913e8e4c7a/ros2_ws/src/vendors/).

For example,

```
ros2_ws/src/vendors/
├── interbotix
├── ned2
└── universal_robots
    ├── Universal_Robots_ROS2_Description
    └── Universal_Robots_ROS2_Driver
```

New external dependencies that require source code should be added to the appropriate `.repos` file. If no customization is needed, consider installing the binary package directly via `rosdep` or other supported package managers.

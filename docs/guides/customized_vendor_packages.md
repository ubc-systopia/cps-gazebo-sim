# Customized Vendor Packages

Dependencies on customized vendor packages can be resolved by dynamically fetching the source code locally using [vcstool](https://github.com/dirk-thomas/vcstool).

The [`.repos` folder](https://github.com/ubc-systopia/cps-gazebo-sim/tree/main/.repos) contains a `.repos` file for each vendor. Running the [`scripts/setup.sh`](https://github.com/ubc-systopia/cps-gazebo-sim/blob/main/scripts/setup.sh) script will import the specified repositories into the corresponding subdirectories under [`src/vendors/`](https://github.com/ubc-systopia/cps-gazebo-sim/tree/main/src/vendors).

For example,

```
src/vendors/
├── interbotix
├── ned2
└── universal_robots
    ├── Universal_Robots_ROS2_Description
    └── Universal_Robots_ROS2_Driver
```

New external dependencies that require source code should be added to the appropriate `.repos` file. If no customization is needed, consider installing the binary package directly via `rosdep` or other supported package managers.
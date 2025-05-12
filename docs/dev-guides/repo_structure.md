# Repo Structure

```
├── ros2_ws
│   └── src
│       ├── multi_arm_lab_sim_bringup
│       ├── multi_arm_lab_sim_description
│       ├── multi_arm_lab_sim_gazebo
│       └── vendors
├── scripts
│   ├── blender_resave.py
│   ├── create_worlds.py
│   ├── load_model.sh
│   └── requirements.txt
└── setup.sh
```

### ROS 2 Packages

* **`multi_arm_lab_sim_bringup`**
  Contains launch files and configuration settings required to initialize and run the multi-arm lab simulation. This includes parameters for ROS 2 nodes, controllers, and optional plugins for logging or visualization.

* **`multi_arm_lab_sim_description`**
  Holds custom model definitions used in the simulation. These typically represent lab-specific robots, sensors, or tooling, and **do not** include third-party or vendor-supplied models, which are instead located under the `vendors/` directory.

* **`multi_arm_lab_sim_gazebo`**
  Contains Gazebo-specific resources such as `.sdf` files. These define the simulated environment and may include lab layouts, objects, and assets. Worlds can be written by combining models directly, or generated programmatically using config files and helper scripts as described in the [world generation guide](../user-guides/loading_models.md).

* **`vendors/`**
  Directory reserved for third-party robot and equipment description packages. These are typically cloned or copied from official sources and should remain unmodified.

### Scripts

* **`blender_resave.py`**
  Resaves a COLLADA (.dae) file using Blender to ensure compatibility with Gazebo.

* **`create_worlds.py`**
  Automates the creation of Gazebo world files using provided configuration files. Useful for generating structured, repeatable environments for testing and simulation.

* **`load_model.sh`**

### Top-level Scripts

* **`setup.sh`**
  Convenience script for initializing the workspace.

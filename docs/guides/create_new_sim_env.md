## Creating a New Simulation Environment in Gazebo and Rviz

This is a guide to create a new ROS package with n Universal Robots given a series of inputs.

### Inputs required from the user:
- **Number of robots**
- **Type of robot(s)**
- Starting orientation:
    - **Role**
    - **Pitch**
    - **Yaw**
- Starting position:
    - **x**
    - **y**
    - **z**
- Starting joint positions:
    - **Shoulder Pan Joint**
    - **Shoulder Lift Joint**
    - **Elbow Joint**
    - **Wrist 1 Joint**
    - **Wrist 2 Joint**
    - **Wrist 3 Joint**

### Step-by-step set up

1. Create a ROS package

    1a. Navigate to the root of the project directory.

    2a. Run the ROS package generation script:
    
    `./scripts/create-ros-pkg`



2. Bring up the world

    Execute the following script, but replace [PACKAGE_NAME] with the name of the package that was just created by the user.

    `./scripts/launch_[PACKAGE_NAME]`


### Understanding the newly generated package

Navigate to [Generated ROS package architecture](generated_ros_package_architecture.md) to understand the contents of the ROS package that was generated.

### Understanding how the ROS package generation works

Navigate to [ROS Package Generation Scripts Architecture](ros_pkg_generattion_scripts_arch.md) to understand how the ROS package is generated.
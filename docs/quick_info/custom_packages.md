# Custom packages

Update the git remote url for interbotix_ros_manipulators, which is installed by default to `interbotix_ws/src`.
```
git remote set-url origin git@github.com:irisxu02/interbotix_ros_manipulators.git
```
The custom package supports a new hardware_type:='gz_sim' which is used to launch xsarm robots in the Ignition version of the modern Gazebo Sim.

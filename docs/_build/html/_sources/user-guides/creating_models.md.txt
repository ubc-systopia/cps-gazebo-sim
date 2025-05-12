# Creating Custom 3D models
Available off-the-shelf software:
- [Polycam](https://poly.cam/)
    - Use the [Remesh](https://youtu.be/J3_5X1wqSy8?si=Xaf9qcoB3Mj4cUoM) feature to bring down the face count to under 30,000.

## Importing models into Gazebo Sim
- location
- orientation
- scale (TODO)

Use Fuel.
then mesh to sdf
sdf to generate new world
https://gazebosim.org/api/sim/8/meshtofuel.html


## Modeling
URDF, Unified Robot Description Format is an XML format for representing a robot model in ROS.
https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
They can be parameterized and generated using Xacro, for situations where multiple robots follow a similar structure.
Xacro is an XML macro language and can be used to clean up URDF for reuse
https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html

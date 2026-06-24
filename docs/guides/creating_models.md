# Creating Custom 3D models
Available off-the-shelf software:
- [Polycam](https://poly.cam/)
    - Use the [Remesh](https://youtu.be/J3_5X1wqSy8?si=Xaf9qcoB3Mj4cUoM) feature to bring down the face count to under 30,000.

## Importing models into Gazebo Sim

The easiest path is `scripts/add-model`, which installs a model (from a Fuel URL,
a model folder, or a bare mesh) into the shared library and emits a ready entry.
See the **scripts** page for details. A few properties to know:

- **location / orientation** — set per placement via the static object's `pose`
  (`[x, y, z, R, P, Y]`), or a RABIT entry's `initial_location` /
  `initial_orientation`.
- **scale** — a property of the *model*, stored once in its `model.sdf`
  `<mesh><scale>`. Both Gazebo (via `include`) and MoveIt (via
  `scene_publisher.py`) read that same value, so size is a single source of
  truth. Change it with `add-model --scale F` or `scripts/resize-model <name>
  <factor>` — never by editing the config.

For manual import without the helper: pull a model from Fuel (or convert a mesh
to SDF), drop it under `src/multi_arm_lab_sim_description/models/`, and reference
it from a config. See <https://gazebosim.org/api/sim/8/meshtofuel.html>.


## Modeling
URDF, Unified Robot Description Format is an XML format for representing a robot model in ROS.
https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
They can be parameterized and generated using Xacro, for situations where multiple robots follow a similar structure.
Xacro is an XML macro language and can be used to clean up URDF for reuse
https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html

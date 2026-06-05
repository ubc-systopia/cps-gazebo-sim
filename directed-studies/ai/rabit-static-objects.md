# RABIT Config — Static Objects

The static objects produced from `configs/rabit-config-2.json` (and its
`rabit-config-2-no-robots.json` variant — both yield the identical set). These
are the objects `translate-config` maps into the flat `static_objects`, which
then become the Gazebo `world.sdf` models and the MoveIt planning-scene
collision objects.

In each `static_objects` entry:
- **geometry** (`box: [sx, sy, sz]`, `cylinder: {radius, length}`, `sphere: {radius}`) = the object's **size**, from the SDL `shape.dimensions`.
- **`pose: [x, y, z, R, P, Y]`** = **position** (x,y,z, from `initial_location`) + **orientation** (R,P,Y radians, from `initial_orientation`).

| name | source | geometry (size, m) | position (x, y, z, m) | orientation (R, P, Y rad) |
|---|---|---|---|---|
| `solution_vial` | passive_object | cylinder r=0.3, L=2.0 | (0.15, 0.45, 0.1) | 0, 0, 0 |
| `grid` | passive_object | box 4.0 × 2.0 × 1.0 | (0.5, 0.4, 0.7) | 0, 0, 0 |
| `Virtual_Dosing_Station` | action_device (SimulatedSmartDevice) | box 2.0 × 3.0 × 4.0 | (2.1, 2.2, 2.14) | 0, 0, 0 |
| `COM8` | action_device (DummyThermoshaker) | box 2.0 × 3.0 × 1.0 | (0.2, 0.4, 0.5) | 0, 0, 0 |
| `COM3` | action_device (MockMagneticStirrer) | box 2.0 × 3.0 × 4.0 | (0.3, 0.4, 0.6) | 0, 0, 0 |

Walls: none (`environment.walls_coords` / `walls_shapes` are empty).

## Note: these are placeholder values, not a real layout

The **sizes** are large (a 4×2×1 box, several 2×3×4 boxes) and the **positions**
cluster near the origin, so the objects overlap each other and engulf a robot at
(0, 0, 0). This is why launching the generated `rabit_lab` package fails with a
controller activation timeout: the arm spawns inside solid collision geometry,
the physics solver stalls, and `joint_state_broadcaster` never activates.
Realistic, non-overlapping sizes/positions (as in `configs/test_static_objects.json`)
are needed before the scene is usable.

# RABIT Schema Changes — What's Needed to Generate a ROS Package

## Purpose

`configs/general_schema.json` is the JSON-Schema for the RABIT SDL/digital-twin
world format, and `configs/rabit_config.json` is a valid instance of it. This
doc records the gap between that schema and the input contract our ROS-package
generator actually consumes, so we know what still has to change before a
`rabit_config.json`-style file can drive `create-ros-pkg` / `regenerate-pkg`.

## The generator's input contract

`scripts/create-ros-pkg --from-json <file>` (see `load_from_json()`) and
`scripts/regenerate-pkg` read a small, flat spec — the same shape as
`configs/diff_robots.json`:

- `package_name` (top-level, **required**)
- `launch_rviz` (optional, default `true`)
- `robot_arms[]`, each with:
  - `key`
  - `model` ∈ {ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30}
  - `base_coordinates`: `x, y, z, R, P, Y` (metres + radians)
  - `initial_joints`: `shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3` (radians)

These are *all* the generator reads. Everything else in the SDL config
(action devices, barriers, passive objects, environment, action/status command
mappings) is irrelevant to package generation.

## Gap analysis: generator inputs vs. `general_schema.json`

| Generator needs | In `general_schema.json`? | Status |
|---|---|---|
| `package_name` (required) | nothing | ❌ Missing entirely |
| `launch_rviz` (optional) | nothing | ⚠️ Missing, but optional (defaults true) |
| arm `key` | `robot_arms[].instances[].key` (free string) | ⚠️ Present but not ROS-safe |
| arm `model` (UR enum) | `robot_arms[].class_name` (free string, e.g. "UR3Arm") | ⚠️ Present but wrong form / unconstrained |
| `base_coordinates.x/y/z` | `instances[].robot_transformation.base_x/base_y/base_z` | ⚠️ Present (renamed) |
| `base_coordinates.R` (roll) | nothing | ❌ Missing |
| `base_coordinates.P` (pitch) | nothing | ❌ Missing |
| `base_coordinates.Y` (yaw) | `robot_transformation.rotation_degree` (scalar, degrees) | ⚠️ Present but degrees, not radians |
| `initial_joints` (6 joint angles) | nothing (only `initial_location` = Cartesian x/y/z) | ❌ Missing entirely |

## What still needs to change

In order of impact:

1. **Initial joint angles — the critical gap.** The generator writes these into
   `initial_positions.yaml`, the `<ros2_control>` `initial_value`s in
   `<pkg>.urdf.xacro`, `robot_description_template.xacro`, and the ros2_control
   macro. The schema has **no joint-space concept at all** — only
   `initial_location` (a Cartesian home point) and `robot_transformation` (base
   pose). Six joint angles cannot be derived from a Cartesian point without
   running IK, so this is genuinely absent information, not a rename.
   **Add:** an `initial_joints` object (6 named joint values, radians) to each
   `robot_arms` entry.

2. **Package name.** The schema describes a *world*, not a *deliverable
   package*; there is no field naming the ROS package to emit (and one world
   could map to several packages).
   **Add:** a `package_name` field (or a separate output-mapping section).

3. **Full base orientation (roll + pitch) and angle units.** The generator
   wants `R, P, Y` in **radians**. The schema offers only a single
   `rotation_degree` (one axis, **degrees**).
   **Add:** `R`/`P` (or a full `R/P/Y` block in radians), or document
   `rotation_degree`'s axis and convert degrees→radians at adapter time.

4. **UR model as a constrained type.** `class_name` is free-form ("UR3Arm") and
   unrestricted; the generator's `UR_TYPES` is a closed set of nine values.
   **Add:** a constrained `model`/UR-type field (or an explicit
   `class_name → model` mapping).

5. **ROS-valid identifier for `key`.** `key` is typed as any string and the
   instances use IPs like `"192.168.48.128"`. The generator turns `key` into
   joint names, tf prefixes, xacro args, and controller names, where
   dots/colons are illegal.
   **Add:** a ROS-name constraint (pattern) on `key`, or a separate sanitized
   name field.

6. *(minor)* **`launch_rviz`** — no field, but optional and defaults to true, so
   not a blocker.

## Summary

The schema is a **superset in breadth** (it models far more than the generator
uses) but a **subset in the data the generator requires**. The three things it
cannot express at all are **initial joint angles**, a **package name**, and
**roll/pitch** (plus radians). The half-expressed fields — `model`, base
`x/y/z`, yaw, and `key` — need renaming, unit conversion, enum-mapping, and
sanitization before they're usable. Of these, the initial joint angles are the
only piece carrying information the schema fundamentally does not hold today.

## Related schema hygiene notes (separate from the gap above)

These surfaced while validating `rabit_config.json` against the schema and are
worth fixing so the schema is a real contract:

- **Typo disables a guard:** `robot_arms` items use `"additionProperties": false`
  (missing the `al`), so misnamed/extra properties on a robot arm are silently
  accepted. Should be `additionalProperties`.
- **`command` vs `command_name` mismatch:** `status_commands` blocks define the
  property as `command`, but `rabit_config.json` uses `command_name`. It only
  validates because `additionalProperties` isn't `false` there.
- **No `required` anywhere:** every field is optional, so the schema cannot
  catch a config missing, e.g., an arm's `class_name` or `instances`.

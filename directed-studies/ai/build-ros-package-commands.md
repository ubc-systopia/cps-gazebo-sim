# Building a ROS Package — All Command Variants

All commands run from the workspace root (`cps-gazebo-sim-2/`).

`--no-build` works with every `create-ros-pkg` mode and with `regenerate-pkg`;
it only skips the final `colcon build` (files are still generated). Flag order
does not matter.

## 1. Interactive (prompts for everything)

```bash
./scripts/create-ros-pkg
./scripts/create-ros-pkg --no-build
```

## 2. From a flat diff_robots-style spec (`--from-json`)

```bash
./scripts/create-ros-pkg --from-json configs/diff_robots.json
./scripts/create-ros-pkg --from-json configs/diff_robots.json --no-build
```

## 3. From a RABIT / general-schema config (`--rabit`) — translates internally

```bash
./scripts/create-ros-pkg --rabit configs/rabit-config-2.json
./scripts/create-ros-pkg --rabit configs/rabit-config-2.json --no-build
```

## 4. Two-step: translate manually, then `--from-json`

```bash
./scripts/translate-config configs/rabit-config-2.json configs/rabit-config-2-translated.json
./scripts/create-ros-pkg --from-json configs/rabit-config-2-translated.json
# add --no-build to the second line to skip colcon
```

## 5. Re-derive an already-generated package after editing its robots.json

```bash
# edit src/<pkg>/config/robots.json first, then:
./scripts/regenerate-pkg <pkg>
./scripts/regenerate-pkg <pkg> --no-build
```

## Notes

- `--rabit` and `--from-json` are mutually exclusive. Methods 3 and 4 produce
  the same result — `--rabit` just runs the translate step into a temp file.
- Use `rabit-config-2.json` (the general-schema-2 version with `package_name`,
  `model`, and `initial_joints`), **not** the original `rabit-config.json`,
  which lacks those fields.
- The package name comes from inside the config (`package_name`), so e.g.
  `rabit-config-2.json` lands in `src/rabit_lab/`.
- The scripts self-source `/opt/ros/humble/setup.bash` if `ROS_DISTRO` is unset.
- After a build:

  ```bash
  source install/setup.bash
  ./scripts/launch_<package_name>
  ```

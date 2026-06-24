# Test Plan — `cps-gazebo-sim`

> **Author:** AI-assisted analysis
> **Date:** 2026-06-19
> **Scope:** All entry points under `scripts/`, the runtime control library
> (`src/multi_arm_control`), generated ROS packages, and the existing test tree.
> **Goal:** Define the unit, integration, end-to-end, and regression tests that
> *should* exist so the package-generation pipeline and the control API can be
> changed with confidence.

---

## Table of Contents

1. [Why this matters](#why-this-matters)
2. [Current testing state](#current-testing-state)
3. [What the system actually does (test surface)](#what-the-system-actually-does-test-surface)
4. [Test taxonomy & how to run each tier](#test-taxonomy--how-to-run-each-tier)
5. [Unit tests](#unit-tests)
6. [Integration tests](#integration-tests)
7. [End-to-end (simulation) tests](#end-to-end-simulation-tests)
8. [Schema & contract tests](#schema--contract-tests)
9. [Regression / golden-file tests](#regression--golden-file-tests)
10. [Static analysis & smoke tests (CI)](#static-analysis--smoke-tests-ci)
11. [Test infrastructure to build first](#test-infrastructure-to-build-first)
12. [Prioritized roadmap](#prioritized-roadmap)

---

## Why this matters

The whole project is a **code generator**: a JSON lab config goes in, and a
complete, buildable ROS 2 / MoveIt / Gazebo package comes out, which is then
driven by one reusable control library (`multi_arm_control`). The blast radius of
any change is large — a one-line edit in `scripts/helpers/generate_srdf.sh` can
silently break planning for *every* package the generator has ever produced, and
nothing today would catch it before a human launches Gazebo and notices an arm
clipping through the floor.

The pipeline is also unusually testable: most of the risky logic is **pure data
transformation** (JSON → JSON, JSON → text files) that needs no ROS, no Gazebo,
and no GPU. That logic currently has **zero automated coverage**. This plan
front-loads those cheap, high-value tests.

---

## Current testing state

| Area | State |
|------|-------|
| `pytest.ini` | `testpaths = tests` — **wrong directory** (the tests live in `test/`, singular). `pytest` from root collects nothing. Fix this first. |
| `Makefile` `test` target | Runs `pytest test/integration` after sourcing ROS — correct path, but only covers the collision suite. |
| `scripts/integration-tests` | Thin wrapper around `pytest test/integration`. |
| `test/integration/collision_tests/` | One real suite (`simple_collisions_test.py`) — **hardcodes** `/home/roman/code/cps-gazebo-sim-2/.../two_arm_moveit_config/...launch.py`, so it only runs on one machine, and depends on a package that is not the actively generated one. |
| `test/other/{broken_tests,tests_a,tests_b,tests_c}/` | Dozens of ad-hoc, partly broken collision/movement scripts. Not collected by CI, not maintained. Treat as a quarry for scenarios, not as tests. |
| `scripts/` (the actual entry points) | **No tests at all.** This is the gap this plan targets. |
| `multi_arm_control` (the control API) | **No tests at all** despite being the single most reused code in `src/`. |
| Pre-commit | `black`, `pylint`, `check-json`, `check-yaml` configured — lint only, no test hook. |

**First action item (not a test, but a prerequisite):** fix `pytest.ini`
`testpaths` to `test`, and decide on a layout (`test/unit`, `test/integration`,
`test/e2e`). Mark ROS/Gazebo-dependent tests with the existing `slow` marker (and
add `ros`, `gazebo` markers) so the pure-Python tier can run anywhere, fast.

---

## What the system actually does (test surface)

Entry points, in dependency order, with their testability:

| Entry point | Lang | Core responsibility | Testability |
|-------------|------|---------------------|-------------|
| `scripts/translate-config` | py | general-schema/RABIT config → flat `diff_robots` spec | **Pure functions — unit-test heavily** |
| `scripts/add-model` | py | install a model (Fuel/dir/mesh), measure it, scale it, inject snippet into a config | Pure parts (slugify, measure, snippet, JSON injection) unit-testable; Fuel/CLI parts need mocking |
| `scripts/resize-model` | py | multiply a model's `model.sdf` `<mesh><scale>` | **Pure — unit-test fully** |
| `scripts/preview-world` | py | config → `world.sdf` → launch Gazebo | `objects_from_config` + SDF emission unit-testable; launch is e2e |
| `scripts/create-ros-pkg` | bash | orchestrate ~25 helpers → full package → `colcon build` | Integration / e2e (file existence, build success) |
| `scripts/regenerate-pkg` | bash | re-derive a package from its `config/robots.json` | Integration (idempotency, round-trip) |
| `scripts/helpers/*.sh` (~27) | bash | each emits one generated artifact (SRDF, URDF xacro, controllers, kinematics, …) | Integration (golden-file per helper) |
| `src/multi_arm_control/conventions.py` | py | single source of truth for ROS naming | **Pure — unit-test fully** |
| `src/multi_arm_control/fleet.py` | py | `ArmFleet`/`Arm` control API over MoveGroup | Pure helpers unit-testable; motion is e2e |

---

## Test taxonomy & how to run each tier

Three tiers, by cost and dependencies:

- **Tier 1 — Unit (pure Python, no ROS).** Runs anywhere with `pytest`, sub-second.
  Targets `translate-config`, `add-model`, `resize-model`, `conventions.py`, and the
  pure helpers of `fleet.py`. **This is where most new tests should go.**
- **Tier 2 — Integration (generator on disk, no Gazebo).** Runs the bash
  generators into a temp dir and asserts on the produced files (existence, valid
  XML/YAML/JSON, golden content). Needs `bash`, `python3`, `xacro` and optionally
  `colcon`; does **not** need a running simulator. Mark `@pytest.mark.slow`.
- **Tier 3 — End-to-end (full sim).** Launches Gazebo + `move_group` and exercises
  the control API and collision checking. Needs the full ROS/Gazebo stack and a
  display (or headless GPU). Mark `@pytest.mark.gazebo` (and `slow`).

Suggested commands:

```bash
pytest test/unit                    # Tier 1 — fast, no ROS, runs in CI
pytest test/integration -m slow     # Tier 2 — generator file checks
make test                           # Tier 3 — sources ROS, launches sim
```

---

## Unit tests

### `translate-config` (highest value — pure, branchy, schema-coupled)

Test `translate()` and its helpers directly by importing the module (it has no
`.py` extension — load it the way `preview-world` does, via `SourceFileLoader`, or
add a tiny conftest helper).

- **`resolve_model`**
  - resolves an explicit `model` field that is a valid UR type
  - resolves `class_name` (e.g. `UR5eArm`) via `CLASS_TO_MODEL` when `model` absent
  - `die()`s (SystemExit, non-zero) on an unknown class/model — assert the error
    message lists valid models
  - `model` field wins over `class_name` when both present
- **`slugify`**
  - strips non-`[A-Za-z0-9_]` (e.g. an IP `192.168.0.5` → `192_168_0_5`)
  - collapses repeated underscores, strips leading/trailing underscores
  - prefixes `arm_` when the result starts with a digit
  - **uniqueness:** same key twice yields `name`, `name_2`, `name_3` and updates
    the `used` set
  - empty/garbage key falls back to `arm`
- **`geometry_from_shape`**
  - `cuboid` → `box: [l, w, h]`; `cylinder` → `{radius, length}`; `sphere` → `{radius}`
  - case-insensitive `shape_type`
  - returns `None` for `hemisphere`/`pyramid`/unknown
  - missing dimensions default to `0.0` (assert no `KeyError`)
- **`_make_static_object`**
  - `model.include`/`model.mesh` take precedence over `shape`
  - `scale` carried through only when present
  - falls back to primitive geometry when no model
  - returns `None` (and warns) when neither a model nor a supported shape exists
  - pose assembled correctly from `location`+`orientation`, with missing fields → 0
- **`translate_passive_objects` / `translate_action_devices` / `translate_walls`**
  - action devices flatten every `instances[]` entry
  - walls zip `walls_coords` with `walls_shapes`, tolerate `len` mismatch (fewer
    coords than shapes → defaults), default axis-aligned orientation
  - skipped (None) objects are dropped, not appended
- **`translate` (top-level)**
  - missing `package_name` → `die`
  - one `robot_arms` entry with N `instances` flattens to N arms
  - `rotation_degree` is converted to radians for `Y` (assert `math.radians`)
  - `initial_joints` defaults each of the 6 joints to `0.0` when absent
  - `launch_rviz` / `avoid_ground` / `ground_level` passthrough behavior
  - empty `robot_arms` (or all-empty `instances`) → `die`
  - shared name namespace across passive/action/wall objects stays unique
  - **round-trip against `configs/`:** translating `configs/rabit-config-2.json`
    equals `configs/rabit-config-2-translated.json` (golden); and
    `configs/rabit-config-2-no-robots.json` exits non-zero

### `add-model`

- **`slugify`** — lowercase, collapse to `_`, fallback to `model`
- **`detect_source`** — Fuel URL vs `fuel.gazebosim` substring vs existing dir vs
  mesh extension; unknown input → `die`; dir without `model.sdf`/`model.config`
  routed correctly
- **`find_sdf_mesh_scale`** — parses `<scale>` from a fixture SDF (1-value
  expands to 3; identity scale returns `None`; malformed XML → `None`)
- **`set_sdf_mesh_scale`** — adds `<scale>` where missing, overwrites where
  present, returns the element count; no-mesh SDF returns 0 without crashing
- **`measure`** — on a tiny known fixture mesh (a unit cube `.obj`/`.stl`):
  returns correct extents and `min_z`; applies the Collada `<unit>` for `.dae`;
  returns `None` gracefully when `trimesh` is unavailable (skip if not installed)
- **`build_snippet`** — emits `package://<pkg>/...` `include`/`mesh` URIs and a
  resting Z computed from `-min_z`; omits `mesh` when there is none
- **`rewrite_sdf_mesh_uri`** — repoints only URIs ending in the old name; count
  correct; tolerates absent SDF
- **`_append_into`** — appends to `static_objects` (creating the array if
  missing); preserves existing entries; round-trips JSON
- **`_attach_into_rabit`** — attaches a `model` block to a `passive_object` or
  an `action_devices[].instances[]` by key; overwrites an existing model with a
  note; `die`s listing available keys when the key is missing
- **arg validation** — `--rabit-into` requires `--attach-to`; `--into` and
  `--rabit-into` are mutually exclusive (assert `argparse` errors)

### `resize-model`

- `find_sdf_mesh_scale` defaults to `(1,1,1)` on a missing/identity SDF
- `set_sdf_mesh_scale` round-trips a known factor (e.g. current `(2,2,2)` × `0.5`
  → `(1,1,1)`); applied to every `<mesh>`; returns count
- `factor <= 0` exits non-zero
- missing model dir / missing `model.sdf` exits non-zero with a clear message

### `preview-world`

- `_load_translate_config` imports the hyphenated script without running `main()`
- `objects_from_config` returns flat `static_objects` directly, and otherwise
  delegates to the translate-config mappers (passive + action + walls)
- empty result path produces the "no static objects" `die`
- (Gazebo launch itself is Tier 3.)

### `multi_arm_control/conventions.py`

Tiny but it is the **contract** between the generator and the runtime — test it so
a rename can't silently desync the two:

- `joint_names("robot1")` → the 6 `robot1_*_joint` names in canonical order
- `planning_group` / `tip_link` / `base_frame` / `controller_action` exact strings
- `HOME_JOINTS` / `UPRIGHT_JOINTS` lengths == 6 and match `JOINT_SUFFIXES`

### `multi_arm_control/fleet.py` (pure parts only)

- **`rpy_to_quat`** — known cases (zeros → `(0,0,0,1)`; π about Z; compare against
  `scipy`/`tf_transformations` within tolerance; verify unit norm)
- **`_err`** — maps a known `MoveItErrorCodes` value to `"N (NAME)"`, unknown →
  `UNKNOWN`
- **`_load_arm_names`** — reads keys from a fixture `robots.json`, skips entries
  missing `key` (mock `get_package_share_directory` to a temp dir)
- **`Arm.pose_constraints` / `Arm.joint_constraints`** — construct these without a
  live node and assert frames, link names, joint names/positions, tolerances, and
  that the position region is a small sphere (no ROS spin needed — just message
  construction)

---

## Integration tests

These run the **bash generators** into a temp workspace and assert on the output.
They need `bash`/`python3`/`xacro` but **not** a simulator.

### Per-helper golden-file tests (`scripts/helpers/*.sh`)

For each helper, invoke it with a fixed small arg set (1 arm and 2 arms) into a
temp dir and assert:

- the expected file is created at the expected path
- it parses: XML helpers (`*.srdf`, `*.urdf.xacro`, `world.sdf`,
  `ros2_control.xacro`) pass `xmllint`/`ET.parse`; YAML helpers
  (`controllers.yaml`, `joint_limits.yaml`, `kinematics.yaml`,
  `moveit_controllers.yaml`, `initial_positions.yaml`, `sensors_3d.yaml`,
  `pilz_cartesian_limits.yaml`, `ompl_planning.yaml`) pass `yaml.safe_load`; JSON
  helpers (`robots.json`) pass `json.load`
- key invariants hold (see SRDF below)
- output matches a committed golden file (see [Regression](#regression--golden-file-tests))

**`generate_srdf.sh` specifically** (highest-risk helper):
- one `<group>` per arm with `base_link → tool0` chain
- `all_arms` composite group emitted **iff** `#arms >= 2`, containing each arm
- the 10 per-arm self-collision `disable_collisions` pairs are present per arm
- inter-arm arm-link↔base-link disables generated for every ordered pair `a≠b`
- base-to-base disables only between consecutive arms
- **naming matches `conventions.py`** (`<arm>_base_link_inertia`, `<arm>_tool0`)

### Generator orchestration (`create-ros-pkg --no-build`)

Run with `--no-build` into a temp `src/`:

- **`--from-json`** with `configs/diff_robots.json` produces every expected file
  (`urdf/<pkg>.urdf.xacro`, `config/{robots.json,*.srdf,*.yaml,*.rviz}`,
  `launch/demo.launch.py`, `scripts/workflow_demo.py`, `CMakeLists.txt`,
  `package.xml`)
- **`--rabit`** with `configs/rabit-config-2.json` runs translate-config first and
  yields the same file set; assert arm count/keys match the translation
- `--from-json` and `--rabit` together → exit 2 (mutually exclusive)
- `--from-json` with a non-existent file → exit 1; with invalid JSON → exit 1;
  with missing `package_name`/empty `robot_arms` → exit 1
- `static_objects` from the config land in `config/robots.json` and in `world.sdf`
- the generated `robots.json` round-trips back through `translate`/`regenerate`
  to the same arm set

### `regenerate-pkg`

- **Idempotency:** `create-ros-pkg --no-build` then `regenerate-pkg --no-build`
  on the same package produces byte-identical generated files (the strongest
  single guarantee — proves create and regenerate share one code path)
- editing `robots.json` (add an arm, swap a UR type) and regenerating updates
  SRDF/URDF/controllers consistently (arm count propagates everywhere)
- it **does not** touch `package.xml`, `CMakeLists.txt`, or `robots.json`
- errors: missing package dir → exit 1; package without `robots.json` → exit 1;
  invalid JSON → exit 1

### `add-model` / `resize-model` on-disk integration

- `add-model <mesh.stl> --name foo --models-dir <tmp>` creates
  `<tmp>/foo/{meshes/foo.stl,model.sdf,model.config}` with a valid wrapper SDF
- `add-model ... --into <flat.json>` appends a parseable snippet
- `add-model ... --rabit-into <rabit.json> --attach-to <key>` mutates the right
  entry and leaves the file valid JSON
- duplicate name → exit 1 ("already exists")
- `resize-model foo 2.0` then `0.5` returns the SDF scale to its original value

---

## End-to-end (simulation) tests

Tier 3 — the full stack. Generalize the existing collision suite so it is not
pinned to one machine or one package.

- **Replace the hardcoded launch path** in
  `test/integration/collision_tests/simple_collisions_test.py` with a fixture
  that (a) generates a package from a fixture config, (b) builds it, (c) resolves
  the launch file via `get_package_share_directory`, and (d) launches it. Make the
  package name a parameter so the suite runs against the *generated* package, not
  a stale committed one.
- **Launch smoke test:** for each shipped config (`configs/diff_robots.json`,
  `configs/rabit-config-2.json`, the `static_objects`/`mesh_objects` test
  configs), generate + build + launch and assert `move_group`, the controllers,
  and Gazebo come up (services/topics present within a timeout). This is the
  single most valuable e2e test — it catches "the generated package doesn't even
  start."
- **Control API motion (`fleet.py`) against a live `move_group`:**
  - `ArmFleet(pkg).arm("robot1").move_joints(UPRIGHT_JOINTS)` returns `True`
  - `move_pose` to a reachable Cartesian target succeeds; an unreachable target
    fails cleanly (returns `False`, no hang) within `allowed_planning_time`
  - `compute_ik` returns 6 joints for a reachable pose, `None` for an
    unreachable one
  - `move_all` moves multiple arms in parallel; `move_coordinated` plans the
    `all_arms` group as one collision-checked trajectory
  - `move_pose` before the action server is up returns `False` after the 10 s
    timeout (don't hang)
- **Collision correctness (port the good scenarios from `test/other`):** drive two
  arms into a known self/inter-arm collision configuration and assert MoveIt
  reports it; drive to a known-clear configuration and assert none. Keep the
  data-driven scenario list (`collision_scenarios.py`) but assert hard instead of
  the current lenient "be more lenient on None" logic.
- **`HOME_JOINTS` floor-sweep guard:** a regression for the documented gotcha that
  all-zeros sweeps near the floor — assert a planned home move does not collide
  with the ground plane when `avoid_ground` is set.
- **`preview-world` headless:** `preview-world <config> --no-gui` (or
  `--print-only`) starts the server / emits a valid `world.sdf` and exits cleanly.

> These are expensive and flaky-prone; gate behind `@pytest.mark.gazebo`, give
> generous timeouts, and run them in a nightly/CI job with a display, not on every
> commit.

---

## Schema & contract tests

The configs are validated against JSON Schemas in `configs/schemas/`. Add tests
that keep schemas, example configs, and the translator in agreement:

- every example in `configs/*.json` validates against its declared schema
  (`general-schema-2.json`, `general-schema-diff.json`, `cps-gazebo-schema.json`)
  using `jsonschema`
- a config that the schema rejects is also rejected by `translate-config`
  (no "valid to translator, invalid to schema" drift, and vice versa)
- the **flat `diff_robots` spec** that `translate-config` emits validates against
  `general-schema-diff.json`
- **conventions contract:** a property test asserting that for any arm key, the
  joint names the generator bakes into `controllers.yaml`/SRDF equal
  `conventions.joint_names(key)` — this is the create↔runtime contract and the
  thing most likely to silently break

---

## Regression / golden-file tests

For deterministic generators, commit golden outputs and diff against them.

- check `configs/diff_robots.json` and `configs/rabit-config-2.json` into a
  `test/golden/` fixture set, generate with `--no-build`, and diff the produced
  tree against committed expected files (normalize absolute paths / temp dirs)
- `translate-config configs/rabit-config-2.json` must equal the committed
  `configs/rabit-config-2-translated.json` (already in the repo — wire it up as
  the first golden test)
- provide a `--update-golden` / `pytest --snapshot-update`-style refresh path so
  intended changes are a reviewable diff, not a silent break

---

## Static analysis & smoke tests (CI)

Cheap gates to run on every push, before the heavier tiers:

- **bash:** `shellcheck` on `scripts/*` and `scripts/helpers/*.sh` (these are
  bash-heavy and currently unlinted by pre-commit, which only covers Python)
- **`bash -n`** syntax check on every script as a fast smoke test
- **python:** the existing `black` + `pylint` (already in pre-commit) plus run
  each Python entry point with `--help` and assert exit 0 (catches import-time
  breakage)
- **JSON/YAML:** `check-json`/`check-yaml` already in pre-commit — extend to
  validate `configs/*.json` against schemas in CI
- **packaging:** `colcon test` / `ament_lint` for the C++/ament packages
  (`CMakeLists.txt` already wires `BUILD_TESTING` + `ament_lint_auto`, but nothing
  runs it)

---

## Test infrastructure to build first

1. **Fix `pytest.ini`** — `testpaths = test`; add markers `slow`, `ros`, `gazebo`.
2. **`conftest.py` fixtures:**
   - `load_script(name)` — import a hyphenated, extensionless script in `scripts/`
     as a module (via `SourceFileLoader`), so unit tests can call its functions.
   - `tmp_workspace` — a temp dir with `src/`, `configs/`, and the `scripts/` tree
     symlinked/copied, for generator integration tests.
   - `tiny_mesh` / `tiny_model_dir` — fixture cube mesh + a minimal `model.sdf`
     for `add-model`/`resize-model`/`measure`.
   - `ros_required` / `gazebo_required` — skip markers that auto-skip when
     `ROS_DISTRO` / a simulator binary is absent, so the suite degrades cleanly on
     a laptop.
3. **Golden directory** `test/golden/` + a refresh helper.
4. **Quarantine** `test/other/*` — either delete or move under `test/other/`
   clearly excluded from collection, and harvest its collision scenarios into the
   maintained e2e suite.

---

## Prioritized roadmap

Ordered by value-per-effort (cheap + high-risk first):

1. **Fix `pytest.ini` + add markers + `conftest.py` loader fixture.** Unblocks
   everything. (infra)
2. **Unit-test `translate-config`** — pure, branchy, schema-coupled, zero deps.
   Wire `rabit-config-2.json → rabit-config-2-translated.json` as the first golden.
3. **Unit-test `conventions.py` + the create↔runtime joint-name contract.** Tiny,
   guards the highest-impact silent break.
4. **Unit-test `resize-model` and `add-model` pure functions.**
5. **Per-helper golden tests + `create-ros-pkg --no-build` file-existence test.**
6. **`regenerate-pkg` idempotency test.** Strong guarantee for cheap.
7. **Schema validation tests** for `configs/*.json`.
8. **Unit-test `fleet.py` pure helpers** (`rpy_to_quat`, constraint builders).
9. **shellcheck + `--help`/`bash -n` smoke gates in CI.**
10. **Generalize the e2e collision/launch suite** (remove the hardcoded path) and
    add the launch-smoke + control-API motion tests behind `@pytest.mark.gazebo`.

Items 1–8 need no ROS and no Gazebo and would take the pipeline from **0%** to
covering essentially all of the generator's decision logic — which is where the
real bugs live.

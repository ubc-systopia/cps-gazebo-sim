# AlabOS — Paper Summary & Architectural Notes

> **Paper:** Fei, Rendy, Kumar, et al. *AlabOS: a Python-based reconfigurable
> workflow management framework for autonomous laboratories.* Digital Discovery,
> 2024, 3, 2275–2288. DOI: 10.1039/d4dd00129j
> **Code:** https://github.com/CederGroupHub/alabos
> **Deployment:** A-Lab (Lawrence Berkeley National Lab) — 3500+ samples
> synthesized and characterized over ~1.5 years.

---

## 1. What AlabOS is

AlabOS is a general-purpose **workflow management framework** for autonomous
materials labs. It is the "operating system" that sits between human-submitted
experiments and the physical hardware (furnaces, robot arms, diffractometers,
balances), orchestrating who does what, in what order, on which samples.

It is *not* a physics simulator, *not* a motion planner, and *not* a Bayesian
optimization layer — it sits above all of those concerns and treats devices as
opaque, command-driven black boxes.

---

## 2. The five-bullet TL;DR

- **What it is** — open-source Python software that runs an autonomous chemistry
  lab; tells robots, furnaces, and instruments what to do, in what order, on
  which samples.
- **The problem it solves** — when a lab runs many machines in parallel on many
  samples, resources collide (two tasks grab the same arm, a sample is sent to
  a full furnace). AlabOS prevents this with a formal *resource reservation*
  mechanism: every task must reserve devices + sample positions before acting,
  and reservations auto-release when the task exits.
- **How users describe work** — a scientist submits an *experiment* (a batch of
  samples + a recipe of steps). AlabOS compiles that into a dependency graph
  (DAG), schedules it, and batches samples when a machine can process several
  at once (e.g. 8 samples per furnace run).
- **What you get around it** — a web dashboard with live status of every sample
  / task / device; a notification system (Slack, email, web prompts) that pings
  operators when a human is needed; and a **simulation mode** for testing
  workflows before risking real hardware.
- **Proof it works** — deployed at LBNL's A-Lab, where it autonomously
  synthesized and characterized 3500+ distinct materials samples over ~1.5
  years.

---

## 3. The entity model (§2.1)

Four nested entity types, each with its own MongoDB collection:

| Entity | What it is | Example |
|---|---|---|
| **Sample** | A physical thing being processed; occupies exactly one *sample position* at a time. | One powder mixture in one crucible. |
| **Sample position** | A space in the lab that can hold one sample. | Slot 3 inside `box_a`. |
| **Device** | A piece of hardware that performs operations. May contain multiple sample positions. | `box_a` (BoxFurnace, 8 inner slots). |
| **Task** | A procedure that operates on samples using devices. Has a *capacity* — how many samples it can process at once. | `Heating` (capacity 8). |
| **Experiment** | A DAG of tasks applied to a batch of samples. The user-submitted unit of work. | "Synthesize and characterize these 20 samples." |

**Key non-obvious relationships:**

- Tasks are **not 1:1 with samples.** One `Heating` task heats 8 samples
  together; one `Diffraction` task scans 1 sample. The DAG fan-out is implicit
  in each task's capacity.
- Sample positions live *inside* devices, but are tracked as first-class
  reservable resources separately from the device itself.
- A single experiment of 16 samples in the A-Lab workflow expands to ~52 task
  instances (1 dosing × 3 heatings × 16 recoveries × 16 diffractions × 16
  endings).

---

## 4. Manager-worker architecture (§2.2, Fig. 2)

AlabOS uses a **manager-worker** pattern with one task actor per running task:

**Manager processes** (singleton, long-lived):

- **Dashboard server** — web UI + REST API for operators
- **Experiment manager** — parses submissions, builds task graphs
- **Task manager** — launches task actors, monitors task status
- **Resource manager** — assigns/tracks devices + sample positions
- **Device manager** — RPC intermediary between tasks and physical devices
- **Logger** — writes all data to MongoDB
- **User request module** — emits notifications, awaits human acknowledgement

**Worker process** = **Task Actor** — one Python subprocess per task instance.
Each actor:

1. Requests resources from the resource manager
2. Sends commands to devices via the device manager (RPC proxy)
3. Logs data and raises user requests as needed
4. Releases resources on exit

**Why this matters:** task actors never talk to each other — only to managers.
This eliminates a whole class of race conditions; task logic can be written
single-threaded without worrying about concurrent access to shared hardware.

---

## 5. Resource management — the load-bearing idea (§3.1)

This is the cleverest part of AlabOS and the reason it can run a busy lab
without operator-side scheduling.

**Mechanism:** cooperative multitasking via Python `with` contexts (Fig. 3):

```python
class Heating(BaseTask):
    def run(self):
        with self.request_resources({BoxFurnace: {"inside": 8}}) as req_1:
            with self.request_resources({RobotArm: {}}) as req_2:
                # task owns furnace AND arm; load samples
                ...
            # arm released here; furnace still held
            furnace.heat()
            with self.request_resources({RobotArm: {}}) as req_3:
                # re-acquire arm to unload
                ...
        # all resources released
```

**Properties:**

- Requests can specify an exact device (`furnace_a`) or just a type
  (`BoxFurnace`); the resource manager picks any available match.
- Sample-position requests are *nested* inside device requests
  (`{BoxFurnace: {"inside": 8}}` = "any box furnace + 8 positions inside it").
- Priority is 1–100 (default 20). Same-priority requests are FCFS.
- Resources auto-release on `__exit__`, including on exception → no "dead
  resources."
- Non-preemptive — granted resources are held until the `with` block ends.
- Benchmarks (Fig. 4): ~7.4 ms per task at 50 devices / 1000 positions; ~14 ms
  per device at 200 concurrent tasks. Negligible vs. the minutes-to-hours
  duration of real lab operations.

**This is the only conflict-avoidance mechanism in AlabOS.** It is logical,
not spatial — it prevents two tasks from *reserving* the same device, but
knows nothing about whether the robot arm physically reaches the furnace door
or whether two arms would collide en route. That gap is precisely the space
that a physics-aware simulator (e.g. Gazebo + MoveIt) fills.

---

## 6. Graph-based workflow — backend vs. UX

The DAG is **load-bearing on the backend** but **deliberately hidden from
users**:

- **Backend** — each task row stores `previous_tasks` and `next_tasks` fields
  in MongoDB; the task manager only launches a task when all parents are
  `Completed`. Cancellation, error propagation, and fan-out scheduling walk
  the graph.
- **Submission** — users do *not* draw graphs. Raw format is JSON; in practice
  they use an `ExperimentBuilder` Python class with `add_sample(...)` and a
  per-sample sequence of tasks. The builder *infers* DAG edges from sequences
  and per-task capacity.
- **Monitoring** — the dashboard (Fig. 5) shows a **flat scrollable table** of
  tasks with status columns, *not* a DAG visualization. Experiment progress is
  a single colored bar (blue / green / red).

**Takeaway for UX design:** the Ceder group found graph-authoring was a
friction point for sophisticated operators and chose to abstract it away on
both ends (submission + monitoring). Users get a sequential mental model; the
system silently compiles it into a DAG.

---

## 7. Other features

### Device control (§3.3)
Centralized via RPC proxies. Each task actor gets an in-place RPC proxy of
each device; calling `furnace.heat(...)` in task code looks identical to a
local method call but actually routes through the device manager. This
guarantees singleton ownership and prevents two tasks from issuing concurrent
commands to the same hardware.

### Simulation mode (§3.3)
A `@mock` decorator applied to device communication methods. The whole
orchestration stack (managers, DB, task actors, dashboard, resource manager,
scheduler) runs normally; only the leaf-level methods that would send
bytes-over-the-wire short-circuit and return canned values. Used to validate
workflow logic — resource reservations, DAG correctness, exception handling —
before deploying to real hardware.

**Limitation:** it is *not* a physics simulator. No thermodynamics, no robot
kinematics, no 3D space, no time simulation (sleeps still sleep). This is the
gap our gazebo-sim work targets.

### Data storage (§3.4)
MongoDB-backed logger; each logged datum tagged with type (device signal,
sample amount, characterization result, system log, other) and severity (10
debug ↔ 50 fatal). Large blobs go to MongoDB GridFS via a `LargeResult`
wrapper. Completed experiments are mirrored to a backup DB.

### Exception handling (§3.2.3)
Two classes:
- **Recoverable** — caught in `try/except`, retried or surfaced as user
  notifications.
- **Unrecoverable** — propagate out of the task; default handler releases
  resources, notifies operators, and continues the rest of the experiment.

### Notifications (§3.2.2)
First-class concept. Tasks can raise a `user_request` with a prompt + list of
response options; the system pauses and waits for operator acknowledgement.
Sent through the dashboard, optionally bridged to Slack / email / IFTTT.

---

## 8. A-Lab deployment (§4)

The canonical workflow (Fig. 6a):

```
PowderDosing → Heating | HeatingWithAtmosphere → RecoverPowder → Diffraction → Ending
```

**Numbers as of paper publication:**

- **16 device types**, **28 device instances**, **289 sample positions**
  defined (Table 1).
- Devices speak HTTP, MODBUS-over-serial, XML-RPC, raw sockets, SSH, or
  Arduino-serial — protocol per device class.
- ~3500 samples processed across ~6 quarters; peak of 149 samples submitted in
  a single day (Feb 9, 2024).
- Exception rates per task (Table 2): `HeatingWithAtmosphere` highest at 22%
  with-exception / 7% unrecoverable; `Ending` lowest at 0.12% / 0.04%.

The deployment is the strongest validation in the paper — most other workflow
systems described in the SDL literature have been demonstrated on a handful of
experiments, not thousands of distinct samples over more than a year of
continuous operation.

---

## 9. Why this matters for our work

Two specific angles connect AlabOS to the cps-gazebo-sim project:

### Automated ROS package generation
AlabOS's `BaseTask` / `BaseDevice` class hierarchy is exactly the kind of
declarative-to-runtime pattern we want to target. A lab developer subclasses
`BaseDevice`, declares its sample positions and communication protocol, and
AlabOS handles registration, scheduling, and remote method dispatch. Our
generator should output ROS packages from a similar declarative spec.

See [[arm-control-api-design]] for the parallel: a `pyniryo`-style API per
robot package, layered on top of generated ROS plumbing.

### Collision detection / avoidance
AlabOS deliberately *only* handles **logical** conflicts — it reserves devices
so two tasks don't issue commands to the same furnace, but it has no model of
3D space, arm reach, or device-to-device collision. Operators must encode
those constraints implicitly in their task definitions and verify via soak
testing.

This is the gap a Gazebo-backed sim fills: physical feasibility checks
(does the arm reach this position? does it collide with the furnace door on
the way?) that AlabOS structurally cannot answer. The two layers are
complementary, not competing.

See [[sdl-overview]] for how other SDLs split this responsibility (e.g.
Matter Lab's ORGANA uses MoveIt + PDDLStream specifically because its
workload needs spatial reasoning).

---

## 10. Glossary of paper-specific terms

- **Sample position** — a single slot that holds at most one sample; tracked
  as a first-class reservable resource.
- **Task capacity** — max samples one task instance can process at once;
  controls DAG fan-out at submission time.
- **Resource request context** — a Python `with` block whose entry reserves
  devices/positions and whose exit auto-releases them.
- **Task actor** — Python subprocess running one task's code; the only place
  task logic executes.
- **Mock decorator** — `@mock(return_value=...)` applied to device methods
  for simulation mode.
- **User request** — a paused-task prompt that requires operator
  acknowledgement through the dashboard before the task resumes.
- **LargeResult** — wrapper for results too big for a single MongoDB document;
  spills to GridFS.

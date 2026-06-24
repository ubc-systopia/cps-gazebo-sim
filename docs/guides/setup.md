# Setup

This guide takes you from **nothing installed** — no Linux, no ROS, no Gazebo —
to a fully built workspace you can launch simulations from. Follow it top to
bottom the first time you set up the project.

By the end you will have:

- **Ubuntu 22.04 LTS (Jammy)** as your operating system
- **ROS 2 Humble**, **Gazebo Fortress**, **MoveIt 2**, and `ros2_control`
- This repository cloned and built into a ready-to-use colcon workspace

---

## Step 1 — Install Ubuntu 22.04 LTS

Install **Ubuntu 22.04 LTS (Jammy)**. This is the only supported OS — ROS 2
Humble and Gazebo Fortress ship prebuilt packages for exactly this release.

---

## Step 2 — System prerequisites and cloning the repo

Open a terminal (in Ubuntu: `Ctrl`+`Alt`+`T`) and run the following.

1. Update the system and install Git:
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y git
   ```

2. Clone this repository. Use **HTTPS** if you don't have GitHub SSH keys set up:
   ```bash
   git clone https://github.com/ubc-systopia/cps-gazebo-sim.git
   cd cps-gazebo-sim
   ```
   Or **SSH** if you do (also needed if you later pull the private vendor
   hardware-driver repos):
   ```bash
   git clone git@github.com:ubc-systopia/cps-gazebo-sim.git
   cd cps-gazebo-sim
   ```

> **GitHub SSH access (optional):** the simulation builds without it. You only
> need an SSH key on GitHub to pull the optional real-hardware vendor drivers
> (`.repos/`). To add one, see
> [Generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

---

## Step 3 — Run the setup script

From the repository root, run the one-shot installer:

```bash
scripts/setup.sh
```

This script is **idempotent** (safe to re-run) and does everything needed to
match the supported environment:

- installs **ROS 2 Humble** (desktop + development tools)
- installs **Gazebo Fortress** and the **`ros-gz`** bridge
- installs **MoveIt 2**, **`ros2_control`** / **`gz_ros2_control`**, and the
  controllers used by generated packages
- installs the Python tooling the `scripts/` commands need (e.g. `trimesh`)
- resolves all workspace dependencies with **`rosdep`**
- builds the workspace with **`colcon build --symlink-install`**

It will ask for your password (it uses `sudo` to install system packages) and
takes several minutes — the `colcon build` step is the longest part. Re-run it
any time after pulling new changes to pick up new dependencies.

---

## Step 4 — Source the workspace

ROS and your freshly built workspace must be **sourced** in every new shell
before you can launch anything:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

So you don't have to type the first line every time, add it to your shell
startup file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

(You still source `install/setup.bash` from inside the repository, since it is
specific to this workspace.)

---

## Step 5 — Verify it works

Launch one of the bundled demos:

```bash
scripts/launch-wrappers/launch_diff_robots
```

Gazebo should open with the robots loaded. If it does, your environment is ready.

From here, see the [Scripts](scripts.md) reference for the commands you'll use to
generate and launch your own lab configurations.

---

## Troubleshooting

- **Gazebo doesn't open / hangs (firewall).** Gazebo uses local network
  discovery that a firewall can block. Disable `ufw` if it is enabled:
  ```bash
  sudo ufw disable
  ```
- **`command not found: ros2` / `colcon`.** You haven't sourced ROS in this
  shell — re-run the commands in **Step 4** above.
- **Black screen or slow/garbled 3D in a VM.** This is a graphics acceleration
  limitation, not a project bug. Enable 3D acceleration in your VM settings, or
  prefer a native install for real work.
- **Wrong Ubuntu version.** `lsb_release -a` must report `22.04` / `jammy`. The
  ROS 2 Humble and Gazebo Fortress packages do not exist for other releases.

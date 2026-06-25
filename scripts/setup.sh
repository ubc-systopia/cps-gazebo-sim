#!/bin/bash
#
# setup.sh - One-shot environment setup for cps-gazebo-sim.
#
# Reproduces the development environment this project is built and run against:
#   - Ubuntu 22.04 (Jammy)
#   - ROS 2 Humble (desktop + dev tools)
#   - Gazebo Fortress (Ignition) + ros-gz bridge
#   - MoveIt 2, ros2_control / gz_ros2_control, and the controllers we use
#   - Python tooling the scripts/ commands need (trimesh for add-model/resize-model)
#   - All workspace dependencies via rosdep, then a full colcon build
#
# The script is idempotent: re-running it skips work that is already done, so it
# is safe to run again after pulling new changes. Run it from anywhere:
#
#     scripts/setup.sh
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Resolve the workspace root (this script lives in <workspace>/scripts).
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_ROOT}"

ROS_DISTRO=humble

log()  { echo -e "\033[0;34m[setup]\033[0m $*"; }
warn() { echo -e "\033[1;33m[setup]\033[0m $*"; }

# ---------------------------------------------------------------------------
# 0. Sanity check: Ubuntu 22.04 Jammy is the supported platform.
# ---------------------------------------------------------------------------
if command -v lsb_release >/dev/null 2>&1; then
    UBUNTU_CODENAME_DETECTED="$(lsb_release -cs)"
    if [ "${UBUNTU_CODENAME_DETECTED}" != "jammy" ]; then
        warn "This project targets Ubuntu 22.04 (jammy); detected '${UBUNTU_CODENAME_DETECTED}'."
        warn "Continuing, but ROS 2 Humble / Gazebo Fortress packages may not be available."
    fi
fi

# ---------------------------------------------------------------------------
# 1. Locale (ROS 2 requires a UTF-8 locale).
# ---------------------------------------------------------------------------
log "Configuring UTF-8 locale..."
sudo apt update
sudo apt install -y locales git curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------------
# 2. ROS 2 Humble (desktop) + development tools.
# ---------------------------------------------------------------------------
if ! dpkg-query -W -f='${Status}' "ros-${ROS_DISTRO}-desktop" 2>/dev/null | grep -q "install ok installed"; then
    log "Installing ROS 2 ${ROS_DISTRO}..."

    sudo apt install -y software-properties-common curl
    sudo add-apt-repository -y universe

    # Official ros-apt-source package: registers the ROS 2 apt repo + key.
    log "Adding the ROS 2 apt repository..."
    ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F\" '{print $4}')"
    if [ -z "${ROS_APT_SOURCE_VERSION}" ]; then
        warn "Could not determine ros-apt-source version (GitHub API rate-limited?). Aborting."
        exit 1
    fi
    curl -fL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb

    sudo apt update
    sudo apt upgrade -y

    log "Installing ros-${ROS_DISTRO}-desktop and ros-dev-tools (compilers, colcon, vcstool, rosdep)..."
    sudo apt install -y "ros-${ROS_DISTRO}-desktop" ros-dev-tools
else
    log "ROS 2 ${ROS_DISTRO} already installed. Skipping."
fi

# ---------------------------------------------------------------------------
# 3. rosdep (resolves package.xml dependencies to apt packages).
# ---------------------------------------------------------------------------
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "Initializing rosdep..."
    sudo rosdep init
fi
log "Updating rosdep..."
rosdep update

# ---------------------------------------------------------------------------
# 4. Gazebo Fortress (Ignition) from the OSRF apt repository.
# ---------------------------------------------------------------------------
if ! dpkg-query -W -f='${Status}' ignition-fortress 2>/dev/null | grep -q "install ok installed"; then
    log "Installing Gazebo Fortress (Ignition)..."
    sudo apt install -y lsb-release gnupg
    sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt update
    sudo apt install -y ignition-fortress
else
    log "Gazebo Fortress already installed. Skipping."
fi

# ---------------------------------------------------------------------------
# 5. ROS <-> Gazebo bridge, MoveIt 2, ros2_control and the controllers we use.
#    rosdep (step 7) pulls most package.xml deps, but installing these meta
#    packages up front keeps a fresh machine self-contained and reproducible.
# ---------------------------------------------------------------------------
log "Installing ROS 2 stack packages (ros-gz, MoveIt, ros2_control, gz_ros2_control)..."
sudo apt install -y \
    "ros-${ROS_DISTRO}-ros-gz" \
    "ros-${ROS_DISTRO}-gz-ros2-control" \
    "ros-${ROS_DISTRO}-moveit" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-ros2-controllers" \
    "ros-${ROS_DISTRO}-joint-trajectory-controller" \
    "ros-${ROS_DISTRO}-joint-state-broadcaster" \
    "ros-${ROS_DISTRO}-pick-ik" \
    "ros-${ROS_DISTRO}-sdformat-urdf"

# ---------------------------------------------------------------------------
# 6. Python tooling for the scripts/ commands.
#    add-model / resize-model measure and convert meshes with trimesh.
# ---------------------------------------------------------------------------
log "Installing Python tooling for scripts/ (trimesh, numpy, pycollada) and dev tools (pre-commit)..."
sudo apt install -y python3-pip
pip3 install --user trimesh numpy pycollada
pip3 install --user -r requirements.dev.txt

# ---------------------------------------------------------------------------
# 7. Source ROS so colcon / rosdep / vcs see the distro.
# ---------------------------------------------------------------------------
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# ---------------------------------------------------------------------------
# 8. Optional vendor packages (real-hardware drivers / extra robot descriptions).
#    The simulation builds without these: the UR description used in sim is the
#    committed src/multi_ur_description. The .repos files are imported only if
#    they list anything (interbotix/ned2 are intentionally empty placeholders).
#    SSH-based repos require GitHub SSH access; failures here are non-fatal.
# ---------------------------------------------------------------------------
VENDOR_PATH="src/vendors"
mkdir -p "${VENDOR_PATH}"
if ls .repos/*.repos >/dev/null 2>&1; then
    log "Importing optional vendor repos into ${VENDOR_PATH} (if any are listed)..."
    for repo_file in .repos/*.repos; do
        # Skip files that contain no repositories (just "repositories:").
        if grep -qE '^\s+\w' "${repo_file}"; then
            log "  vcs import < ${repo_file}"
            vcs import "${VENDOR_PATH}" < "${repo_file}" || \
                warn "  Failed to import ${repo_file} (need GitHub SSH access?); continuing."
        fi
    done
fi

# ---------------------------------------------------------------------------
# 9. Resolve and install workspace dependencies from package.xml files.
#    -r keeps going past dependencies that have no apt package on this distro
#    (e.g. optional MoveIt/interbotix extras we don't build).
# ---------------------------------------------------------------------------
log "Installing workspace dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}"

# ---------------------------------------------------------------------------
# 10. Build the workspace.
# ---------------------------------------------------------------------------
log "Building the colcon workspace (this can take several minutes)..."
colcon build --symlink-install

# ---------------------------------------------------------------------------
# 11. Persist ROS source in ~/.bashrc so every new shell has ROS in PATH.
# ---------------------------------------------------------------------------
ROS_SOURCE_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
if ! grep -qF "${ROS_SOURCE_LINE}" "${HOME}/.bashrc"; then
    log "Adding ROS ${ROS_DISTRO} source line to ~/.bashrc..."
    echo "" >> "${HOME}/.bashrc"
    echo "# ROS 2 ${ROS_DISTRO} — added by cps-gazebo-sim setup.sh" >> "${HOME}/.bashrc"
    echo "${ROS_SOURCE_LINE}" >> "${HOME}/.bashrc"
else
    log "ROS ${ROS_DISTRO} source line already in ~/.bashrc. Skipping."
fi

LOCAL_BIN_LINE='export PATH="${HOME}/.local/bin:${PATH}"'
if ! grep -qF '.local/bin' "${HOME}/.bashrc"; then
    log "Adding ~/.local/bin to PATH in ~/.bashrc..."
    echo "${LOCAL_BIN_LINE}" >> "${HOME}/.bashrc"
fi
export PATH="${HOME}/.local/bin:${PATH}"

# ---------------------------------------------------------------------------
# Done.
# ---------------------------------------------------------------------------
cat <<EOF

[setup] Setup complete.

To use the workspace in a new shell, source the local overlay:

    source ${WORKSPACE_ROOT}/install/setup.bash

ROS 2 ${ROS_DISTRO} has been added to ~/.bashrc and is available in every new
shell. Re-source your current shell with:

    source ~/.bashrc

Next steps:
    scripts/launch-wrappers/launch_diff_robots     # launch a demo
    scripts/create-ros-pkg --help                  # generate your own lab package
EOF

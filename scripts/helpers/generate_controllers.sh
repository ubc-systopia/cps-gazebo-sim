#!/bin/bash
# Usage: generate_controllers.sh <package_name> <pkg_dir> <controller1:j1,j2,...> <controller2:j1,j2,...> ...

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2

out_file="${PKG_DIR}/config/controllers.yaml"
echo "# controllers.yaml generated dynamically" > "$out_file"
echo "controller_manager:" >> "$out_file"
echo "  ros__parameters:" >> "$out_file"
echo "    update_rate: 100  # Hz" >> "$out_file"
echo "    use_sim_time: true" >> "$out_file"
echo "" >> "$out_file"
echo "    # Controller definitions" >> "$out_file"
joint_state_joints=()
for arg in "$@"; do
    controller_name="${arg%%:*}"
    joint_list="${arg#*:}"
    echo "    ${controller_name}:" >> "$out_file"
    echo "      type: joint_trajectory_controller/JointTrajectoryController" >> "$out_file"
    # Save joints for later
    IFS=',' read -ra joints <<< "$joint_list"
    joint_state_joints+=("${joints[@]}")
done
echo "" >> "$out_file"
echo "    joint_state_broadcaster:" >> "$out_file"
echo "      type: joint_state_broadcaster/JointStateBroadcaster" >> "$out_file"
echo "" >> "$out_file"

# Parameters for each controller
for arg in "$@"; do
    controller_name="${arg%%:*}"
    joint_list="${arg#*:}"
    echo "${controller_name}:" >> "$out_file"
    echo "  ros__parameters:" >> "$out_file"
    echo "    use_sim_time: true" >> "$out_file"
    echo "    joints:" >> "$out_file"
    IFS=',' read -ra joints <<< "$joint_list"
    for j in "${joints[@]}"; do
            echo "      - $j" >> "$out_file"
    done
    echo "    command_interfaces:" >> "$out_file"
    echo "      - position" >> "$out_file"
    echo "    state_interfaces:" >> "$out_file"
    echo "      - position" >> "$out_file"
    echo "      - velocity" >> "$out_file"
    echo "" >> "$out_file"
done

echo "joint_state_broadcaster:" >> "$out_file"
echo "  ros__parameters:" >> "$out_file"
echo "    use_sim_time: true" >> "$out_file"
echo "    joints:" >> "$out_file"
for j in "${joint_state_joints[@]}"; do
    echo "      - $j" >> "$out_file"
done

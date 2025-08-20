#!/bin/bash
# Usage: generate_srdf.sh <package_name> <pkg_dir> <robot1> [<robot2> ...]

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2

OUT_FILE="${PKG_DIR}/config/${PACKAGE_NAME}.srdf"

{
  echo "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
  echo "<!--This does not replace URDF, and is not an extension of URDF."
  echo "    This is a format for representing semantic information about the robot structure."
  echo "    A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined" 
  echo "-->"
  echo "<robot name=\"${PACKAGE_NAME}\">"
  echo "    <!--GROUPS: Representation of a set of joints and links. This can be useful for specifying DOF to plan for, defining arms, end effectors, etc-->"
  echo "    <!--LINKS: When a link is specified, the parent joint of that link (if it exists) is automatically included-->"
  echo "    <!--JOINTS: When a joint is specified, the child link of that joint (which will always exist) is automatically included-->"
  echo "    <!--CHAINS: When a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group-->"
  echo "    <!--SUBGROUPS: Groups can also be formed by referencing to already defined group names-->"

  # Groups: one per robot, using base_link -> tool0 chain
  for robot in "$@"; do
    echo "    <group name=\"${robot}\">"
    echo "        <chain base_link=\"${robot}_base_link_inertia\" tip_link=\"${robot}_tool0\"/>"
    echo "    </group>"
  done

  echo "    <!--DISABLE COLLISIONS: By default, any link pair may collide. The pairs below are disabled for proper operation. -->"
  # Per-robot self-collision disables (UR-style link set)
  for robot in "$@"; do
    echo ""
    echo "    <!-- ${robot} SELF-COLLISION DISABLES -->"
    echo "    <disable_collisions link1=\"${robot}_base_link_inertia\" link2=\"${robot}_shoulder_link\" reason=\"Adjacent\"/>"
    echo "    <disable_collisions link1=\"${robot}_base_link_inertia\" link2=\"${robot}_upper_arm_link\" reason=\"Never\"/>"
    echo "    <disable_collisions link1=\"${robot}_base_link_inertia\" link2=\"${robot}_wrist_1_link\" reason=\"Never\"/>"
    echo "    <disable_collisions link1=\"${robot}_base_link_inertia\" link2=\"${robot}_wrist_2_link\" reason=\"Never\"/>"
    echo "    <disable_collisions link1=\"${robot}_forearm_link\" link2=\"${robot}_upper_arm_link\" reason=\"Adjacent\"/>"
    echo "    <disable_collisions link1=\"${robot}_forearm_link\" link2=\"${robot}_wrist_1_link\" reason=\"Adjacent\"/>"
    echo "    <disable_collisions link1=\"${robot}_shoulder_link\" link2=\"${robot}_upper_arm_link\" reason=\"Adjacent\"/>"
    echo "    <disable_collisions link1=\"${robot}_shoulder_link\" link2=\"${robot}_wrist_1_link\" reason=\"Never\"/>"
    echo "    <disable_collisions link1=\"${robot}_shoulder_link\" link2=\"${robot}_wrist_2_link\" reason=\"Never\"/>"
    echo "    <disable_collisions link1=\"${robot}_wrist_1_link\" link2=\"${robot}_wrist_2_link\" reason=\"Adjacent\"/>"
    echo "    <disable_collisions link1=\"${robot}_wrist_1_link\" link2=\"${robot}_wrist_3_link\" reason=\"Never\"/>"
    echo "    <disable_collisions link1=\"${robot}_wrist_2_link\" link2=\"${robot}_wrist_3_link\" reason=\"Adjacent\"/>"
  done

  echo ""
  echo "    <!-- ALL INTER-ARM COLLISIONS ENABLED BY DEFAULT. Add specific disables here if needed. -->"

  # Optional: disable base-to-base collisions between consecutive robots (common for rigidly connected bases)
  if [ "$#" -ge 2 ]; then
    prev=""
    echo ""
    echo "    <!-- BASE-TO-BASE CONNECTION (keep this since the bases are physically connected) -->"
    for robot in "$@"; do
      if [ -n "$prev" ]; then
        echo "    <disable_collisions link1=\"${prev}_base_link_inertia\" link2=\"${robot}_base_link_inertia\" reason=\"Adjacent\"/>"
      fi
      prev="$robot"
    done
  fi

  echo ""
  echo "</robot>"
} > "$OUT_FILE"

echo "Generated SRDF: $OUT_FILE"

#!/usr/bin/env python3
"""Add rabit_lab's static_objects (from config/robots.json) to the MoveIt
planning scene as primitive collision objects, so motion planning avoids them.

Launched automatically by demo.launch.py. Supported geometries: box, cylinder,
sphere (mesh/include objects are skipped with a warning)."""

import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

PACKAGE = "rabit_lab"
PLANNING_FRAME = "world"   # URDF/SRDF root link in generated packages


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def make_pose(p):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
    qx, qy, qz, qw = quat_from_rpy(float(p[3]), float(p[4]), float(p[5]))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    return pose


def make_primitive(obj):
    sp = SolidPrimitive()
    if "box" in obj:
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [float(v) for v in obj["box"]]
    elif "cylinder" in obj:
        c = obj["cylinder"]
        sp.type = SolidPrimitive.CYLINDER
        sp.dimensions = [float(c["length"]), float(c["radius"])]  # [height, radius]
    elif "sphere" in obj:
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [float(obj["sphere"]["radius"])]
    else:
        return None
    return sp


def load_static_objects():
    share = get_package_share_directory(PACKAGE)
    with open(os.path.join(share, "config", "robots.json")) as f:
        return json.load(f).get("static_objects") or []


def main():
    rclpy.init()
    node = rclpy.create_node("scene_publisher")
    log = node.get_logger()

    objects = load_static_objects()
    if not objects:
        log.info("no static_objects; nothing to add to the planning scene")
        node.destroy_node()
        rclpy.shutdown()
        return

    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    log.info("waiting for /apply_planning_scene ...")
    if not cli.wait_for_service(timeout_sec=30.0):
        log.error("/apply_planning_scene unavailable; planning scene NOT populated")
        node.destroy_node()
        rclpy.shutdown()
        return

    scene = PlanningScene()
    scene.is_diff = True
    added = []
    for obj in objects:
        prim = make_primitive(obj)
        if prim is None:
            log.warn(f"skipping '{obj.get('name')}' (no box/cylinder/sphere geometry)")
            continue
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = obj["name"]
        co.primitives = [prim]
        co.primitive_poses = [make_pose(obj["pose"])]
        co.operation = CollisionObject.ADD
        scene.world.collision_objects.append(co)
        added.append(obj["name"])

    req = ApplyPlanningScene.Request()
    req.scene = scene
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()
    if result is not None and result.success:
        log.info(f"added to planning scene: {added}")
    else:
        log.error("apply_planning_scene call failed")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

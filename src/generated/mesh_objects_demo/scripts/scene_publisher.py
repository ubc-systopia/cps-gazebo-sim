#!/usr/bin/env python3
"""Add mesh_objects_demo's static_objects (from config/robots.json) to the MoveIt
planning scene as primitive collision objects, so motion planning avoids them.

Launched automatically by demo.launch.py. Supported geometries: box, cylinder,
sphere, and mesh. A 'mesh' object is loaded as its true triangle geometry so the
arms plan around the actual shape, not a bounding box. 'include'-only objects
that carry no collision geometry are skipped with a warning."""

import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

PACKAGE = "mesh_objects_demo"
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


def resolve_uri(uri):
    """Resolve a package://pkg/rel or file:// uri (or plain path) to a filesystem path."""
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://"):].partition("/")
        return os.path.join(get_package_share_directory(pkg), rel)
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


def _scale_triple(obj):
    s = obj.get("scale", 1.0)
    if isinstance(s, (int, float)):
        return (float(s), float(s), float(s))
    return (float(s[0]), float(s[1]), float(s[2]))


def make_mesh(obj, log):
    """Load obj['mesh'] into a shape_msgs/Mesh (true triangle geometry).

    Uses trimesh; returns None (after logging why) if the file is missing or
    trimesh (plus pycollada for .dae) isn't installed."""
    path = resolve_uri(obj["mesh"])
    sx, sy, sz = _scale_triple(obj)
    if not os.path.exists(path):
        log.warn(f"mesh file not found for '{obj.get('name')}': {path}")
        return None

    # Collada (.dae) files carry a <unit meter="..."> that assimp/Gazebo apply to
    # reach meters (e.g. 0.001 for a model authored in mm) but trimesh does NOT,
    # so without this the mesh shows up ~1000x too large in RViz. Apply it here so
    # the planning-scene size matches Gazebo.
    if path.lower().endswith(".dae"):
        try:
            import collada
            unit = float(collada.Collada(path).assetInfo.unitmeter or 1.0)
            sx, sy, sz = sx * unit, sy * unit, sz * unit
        except Exception as e:
            log.warn(f"could not read collada unit for '{obj.get('name')}': {e}; assuming meters")

    # trimesh is the primary loader: pyassimp segfaults (SIGSEGV) on many .dae
    # files on Ubuntu 22.04 and the crash can't be caught from Python, so we
    # only fall back to it if trimesh is entirely unavailable.
    verts = faces = None
    try:
        import trimesh
        tm = trimesh.load(path, force="mesh")
        verts = [(float(v[0]), float(v[1]), float(v[2])) for v in tm.vertices]
        faces = [tuple(int(i) for i in f) for f in tm.faces]
    except Exception as e_trimesh:
        log.error(
            f"cannot load mesh '{obj.get('name')}' ({path}) with trimesh: {e_trimesh}. "
            f"Install it with: pip install trimesh  (and 'pip install pycollada' for .dae files)."
        )
        return None

    mesh = Mesh()
    mesh.vertices = [Point(x=v[0] * sx, y=v[1] * sy, z=v[2] * sz) for v in verts]
    for f in faces:
        t = MeshTriangle()
        t.vertex_indices = [f[0], f[1], f[2]]
        mesh.triangles.append(t)
    return mesh


def load_robots_json():
    share = get_package_share_directory(PACKAGE)
    with open(os.path.join(share, "config", "robots.json")) as f:
        return json.load(f)


def make_ground(data):
    """A large thin box just below the world origin so the arm won't plan through
    the floor (MoveIt's planning scene has no ground by default). Disable with
    "avoid_ground": false in robots.json; set the floor height with "ground_level"
    (meters, default 0.0). The box top sits 2 cm below ground_level so a base
    mounted at that height isn't flagged as already in collision."""
    if not data.get("avoid_ground", True):
        return None
    thickness = 1.0
    top = float(data.get("ground_level", 0.0)) - 0.02
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.BOX
    sp.dimensions = [10.0, 10.0, thickness]
    pose = Pose()
    pose.position.z = top - thickness / 2.0
    pose.orientation.w = 1.0
    co = CollisionObject()
    co.header.frame_id = PLANNING_FRAME
    co.id = "ground"
    co.operation = CollisionObject.ADD
    co.primitives = [sp]
    co.primitive_poses = [pose]
    return co


def main():
    rclpy.init()
    node = rclpy.create_node("scene_publisher")
    log = node.get_logger()

    data = load_robots_json()
    objects = data.get("static_objects") or []
    ground = make_ground(data)
    if not objects and ground is None:
        log.info("no static_objects and avoid_ground=false; nothing to add")
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
        co = CollisionObject()
        co.header.frame_id = PLANNING_FRAME
        co.id = obj["name"]
        co.operation = CollisionObject.ADD
        pose = make_pose(obj["pose"])

        prim = make_primitive(obj)
        if prim is not None:
            co.primitives = [prim]
            co.primitive_poses = [pose]
        elif "mesh" in obj:
            mesh = make_mesh(obj, log)
            if mesh is None:
                continue  # make_mesh already logged the reason
            co.meshes = [mesh]
            co.mesh_poses = [pose]
        else:
            log.warn(f"skipping '{obj.get('name')}' (no box/cylinder/sphere/mesh geometry)")
            continue

        scene.world.collision_objects.append(co)
        added.append(obj["name"])

    if ground is not None:
        scene.world.collision_objects.append(ground)
        added.append("ground")

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

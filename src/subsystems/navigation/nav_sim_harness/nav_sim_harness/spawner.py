#!/usr/bin/env python3
"""Gazebo entity spawner for the nav_sim_harness.

Wraps the bridged ``/world/<name>/create`` service (``ros_gz_interfaces/srv/
SpawnEntity``) so scenarios can drop mission artifacts into a running world:

  * ``spawn_box``        — a static coloured box (obstacle / placeholder target).
  * ``spawn_aruco_post`` — the pre-built ``model://aruco_post`` (3-sided ArUco
    marker). No runtime texture work — it just instantiates the committed model
    via an SDF ``<include>``, resolved through ``GZ_SIM_RESOURCE_PATH``.

The create service must be bridged from Gazebo to ROS (the harness launch file
adds the bridge). The bridge name is ``/world/<world_name>/create``.

``Spawner`` does NOT spin its own executor — it expects the caller's node to be
spinning (the runner adds the node to a background executor). For standalone use
this module's ``main()`` sets up that executor itself, e.g.::

    python3 -m nav_sim_harness.spawner --world default box  --pose 5 0 0.5 --size 1 1 1
    python3 -m nav_sim_harness.spawner --world default aruco --pose 12 3 0
"""
import argparse
import math
import sys
import threading
import time
from typing import Optional, Sequence

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SpawnEntity


# A pose is (x, y, z, roll, pitch, yaw) in the world (ENU) frame, radians.
Pose6 = Sequence[float]


def _pose_str(pose: Pose6) -> str:
    """Format a 6-DoF pose for an SDF ``<pose>`` element."""
    vals = list(pose) + [0.0] * (6 - len(pose))
    return " ".join(f"{v:.6g}" for v in vals[:6])


def _to_ros_pose(pose: Pose6) -> Pose:
    """Convert an (x, y, z, roll, pitch, yaw) tuple to a geometry_msgs/Pose.

    Set on ``EntityFactory.pose``, which is authoritative for spawn placement —
    the create service ignores the SDF ``<pose>`` and uses this (defaulting to
    the origin if left unset, which is why an unset pose spawns at 0,0,0).
    """
    vals = list(pose) + [0.0] * (6 - len(pose))
    x, y, z, roll, pitch, yaw = vals[:6]
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.w = cr * cp * cy + sr * sp * sy
    p.orientation.x = sr * cp * cy - cr * sp * sy
    p.orientation.y = cr * sp * cy + sr * cp * sy
    p.orientation.z = cr * cp * sy - sr * sp * cy
    return p


def _box_sdf(name: str, pose: Pose6, size: Sequence[float],
             rgba: Sequence[float] = (0.85, 0.2, 0.2, 1.0)) -> str:
    sx, sy, sz = size
    r, g, b, a = rgba
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>{_pose_str(pose)}</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{sx:.6g} {sy:.6g} {sz:.6g}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx:.6g} {sy:.6g} {sz:.6g}</size></box></geometry>
        <material>
          <ambient>{r:.3g} {g:.3g} {b:.3g} {a:.3g}</ambient>
          <diffuse>{r:.3g} {g:.3g} {b:.3g} {a:.3g}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def _include_sdf(name: str, uri: str, pose: Pose6) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <include>
    <uri>{uri}</uri>
    <name>{name}</name>
    <pose>{_pose_str(pose)}</pose>
  </include>
</sdf>"""


class SpawnError(RuntimeError):
    """Raised when a spawn request fails or times out."""


class Spawner:
    """Thin rclpy client around the Gazebo create service.

    Parameters
    ----------
    node : rclpy.node.Node
        A node that is already being spun by an executor (the create service
        client's async future is dispatched by that executor).
    world_name : str
        Gazebo world name; selects the ``/world/<world_name>/create`` service.
    """

    def __init__(self, node: Node, world_name: str = "default"):
        self._node = node
        self._world_name = world_name
        self._srv_name = f"/world/{world_name}/create"
        self._client = node.create_client(SpawnEntity, self._srv_name)

    def wait_for_service(self, timeout: float = 15.0) -> bool:
        deadline = time.monotonic() + timeout
        while not self._client.service_is_ready():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.1)
        return True

    # ── low-level call ──────────────────────────────────────────────────────

    def _spawn_sdf(self, name: str, sdf: str, pose: Pose6,
                   timeout: float = 15.0) -> None:
        if not self.wait_for_service(timeout):
            raise SpawnError(f"create service {self._srv_name} not available "
                             f"(is the harness create-bridge running?)")
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        req.entity_factory.allow_renaming = False
        # Authoritative spawn pose — the service uses this, not the SDF <pose>.
        req.entity_factory.pose = _to_ros_pose(pose)

        future = self._client.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise SpawnError(f"spawn of '{name}' timed out after {timeout:.0f}s")
            time.sleep(0.05)

        result = future.result()
        if result is None or not result.success:
            raise SpawnError(f"Gazebo rejected spawn of '{name}'")
        self._node.get_logger().info(f"Spawned '{name}' into world '{self._world_name}'")

    # ── public spawn helpers ─────────────────────────────────────────────────

    def spawn_box(self, name: str, pose: Pose6,
                  size: Sequence[float] = (1.0, 1.0, 1.0),
                  rgba: Sequence[float] = (0.85, 0.2, 0.2, 1.0),
                  timeout: float = 15.0) -> None:
        """Drop a static coloured box centred at ``pose`` with the given size."""
        self._spawn_sdf(name, _box_sdf(name, pose, size, rgba), pose, timeout)

    def spawn_aruco_post(self, name: str, pose: Pose6,
                         timeout: float = 15.0) -> None:
        """Instantiate the pre-built ``model://aruco_post`` at ``pose``.

        The model origin is the base of the post (z=0), so ``pose`` z is the
        ground height at the drop point.
        """
        self._spawn_sdf(name, _include_sdf(name, "model://aruco_post", pose), pose, timeout)


# ── standalone CLI ──────────────────────────────────────────────────────────


def _parse_args(argv):
    ap = argparse.ArgumentParser(
        description="Spawn a box or the aruco_post model into a running Gazebo world.")
    ap.add_argument("--world", default="default", help="Gazebo world name (default: default)")
    ap.add_argument("--name", default=None, help="entity name (default: derived from kind)")
    ap.add_argument("kind", choices=["box", "aruco", "aruco_post"],
                    help="what to spawn")
    ap.add_argument("--pose", type=float, nargs="+", default=[0, 0, 0],
                    metavar="V", help="x y z [roll pitch yaw] in world ENU (default: 0 0 0)")
    ap.add_argument("--size", type=float, nargs=3, default=[1.0, 1.0, 1.0],
                    metavar=("SX", "SY", "SZ"), help="box size (box only)")
    return ap.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)

    rclpy.init()
    node = rclpy.create_node("nav_sim_spawner")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    rc = 0
    try:
        spawner = Spawner(node, world_name=args.world)
        if args.kind == "box":
            name = args.name or "harness_box"
            spawner.spawn_box(name, args.pose, args.size)
        else:
            name = args.name or "aruco_post"
            spawner.spawn_aruco_post(name, args.pose)
        print(f"[✓] spawned {args.kind} '{name}' at {_pose_str(args.pose)}")
    except SpawnError as e:
        print(f"[✗] {e}", file=sys.stderr)
        rc = 1
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

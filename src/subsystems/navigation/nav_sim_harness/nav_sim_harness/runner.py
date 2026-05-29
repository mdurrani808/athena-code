#!/usr/bin/env python3
"""Scenario runner for the nav_sim_harness.

Loads a YAML scenario, spawns its artifacts, then drives each navigation step
through the ``mission_cli`` subprocess — exactly the operator path — while
background-subscribing to ``/nav_status``, ``/led_status`` and
``/odom/ground_truth``. Each step is scored off those observers (NOT the
subprocess exit code, which ``mission_cli`` returns 0 for on both pass and
fail). Prints a per-step pass/fail summary and exits 0 only if every step
passed.

Scenario schema
───────────────
    name: single_aruco_post
    world: terrain_world.sdf          # informational; run.sh launches the world
    world_name: default               # Gazebo world name (for the spawn service)
    timeout_s: 240                    # per-step nav timeout
    setup:                            # optional, spawned before any step
      - spawn: {type: aruco_post, name: post1, pose: [12, 3, 0]}
      - spawn: {type: box, name: rock, pose: [6, 0, 0.5], size: [1, 1, 1]}
    steps:
      - cli: nav post1 38.42392 -110.78484 13
        assert: {kind: aruco_post, target: post1, max_dist_m: 2.0,
                 expect_state: STOPPED_AT_TARGET}
      - cli: nav meter 5 0 3
        assert: {kind: gnss, max_dist_m: 3.0, expect_state: STOPPED_AT_TARGET}

Assert kinds
────────────
  gnss        : nav_status.state == expect_state AND distance_to_goal_m < max_dist_m
  aruco_post  : nav_status.state == expect_state AND flashing-green LED seen AND
                ground-truth distance to the spawned post < max_dist_m
"""
import math
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from msgs.action import NavigateToTarget
from msgs.msg import LedStatus, NavStatus
from nav_msgs.msg import Odometry

from nav_sim_harness.spawner import Spawner, SpawnError

# ── ANSI ────────────────────────────────────────────────────────────────────
RED, GRN, YLW, CYN, BLD, DIM, RST = (
    "\033[0;31m", "\033[0;32m", "\033[1;33m", "\033[0;36m",
    "\033[1m", "\033[2m", "\033[0m",
)


def info(m):
    print(f"{GRN}[✓]{RST} {m}")


def warn(m):
    print(f"{YLW}[!]{RST} {m}")


def err(m):
    print(f"{RED}[✗]{RST} {m}", file=sys.stderr)


def hdr(m):
    print(f"\n{BLD}{m}{RST}")


# ── background observers ─────────────────────────────────────────────────────


class Observers:
    """Latest-message cache for the topics scoring depends on.

    A per-step ``reset()`` clears the transient 'flashing-green seen' latch so
    each step is scored only on events that occur during/after its own nav.
    """

    def __init__(self, node):
        self._lock = threading.Lock()
        self.nav_status: Optional[NavStatus] = None
        self.odom: Optional[Odometry] = None
        self.led: Optional[LedStatus] = None
        self._flash_green_seen = False

        node.create_subscription(NavStatus, "/nav_status", self._on_nav, 10)
        node.create_subscription(LedStatus, "/led_status", self._on_led, 10)
        node.create_subscription(Odometry, "/odom/ground_truth", self._on_odom, 10)

    def _on_nav(self, msg):
        with self._lock:
            self.nav_status = msg

    def _on_odom(self, msg):
        with self._lock:
            self.odom = msg

    def _on_led(self, msg):
        with self._lock:
            self.led = msg
            if _is_flash_green(msg):
                self._flash_green_seen = True

    def reset_flash_latch(self):
        with self._lock:
            self._flash_green_seen = False

    def snapshot(self):
        with self._lock:
            return (self.nav_status, self.odom, self.led, self._flash_green_seen)


def _is_flash_green(led: LedStatus) -> bool:
    """LED is in the 'arrived' flashing-green state."""
    return (led is not None
            and led.cmd == LedStatus.CMD_FLASH
            and led.g > 100 and led.r < 80 and led.b < 80)


# ── scenario model ───────────────────────────────────────────────────────────


@dataclass
class Spawn:
    kind: str
    name: str
    pose: List[float]
    size: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])


@dataclass
class Step:
    cli: str
    assert_block: dict


@dataclass
class Scenario:
    name: str
    world: str
    world_name: str
    timeout_s: float
    setup: List[Spawn]
    steps: List[Step]


def _load_scenario(path: str) -> Scenario:
    with open(path) as f:
        doc = yaml.safe_load(f)

    setup = []
    for item in doc.get("setup", []) or []:
        s = item["spawn"] if "spawn" in item else item
        kind = s["type"]
        setup.append(Spawn(
            kind=kind,
            name=s.get("name", kind),
            pose=[float(v) for v in s["pose"]],
            size=[float(v) for v in s.get("size", [1.0, 1.0, 1.0])],
        ))

    steps = []
    for st in doc.get("steps", []) or []:
        steps.append(Step(cli=st["cli"], assert_block=st.get("assert", {}) or {}))

    return Scenario(
        name=doc.get("name", "scenario"),
        world=doc.get("world", "terrain_world.sdf"),
        world_name=doc.get("world_name", "default"),
        timeout_s=float(doc.get("timeout_s", 240)),
        setup=setup,
        steps=steps,
    )


# ── runner ───────────────────────────────────────────────────────────────────


class Runner:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        rclpy.init()
        self.node = rclpy.create_node("nav_sim_runner")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="runner_spin")
        self._spin_thread.start()

        self.obs = Observers(self.node)
        self.spawner = Spawner(self.node, world_name=scenario.world_name)
        # Created only to confirm the operator action surface is up; nav goals
        # themselves go through the mission_cli subprocess.
        self._action_client = ActionClient(
            self.node, NavigateToTarget, "/mission_executive/navigate_to_target")
        # name -> spawned world pose (for ground-truth proximity scoring)
        self._spawned: Dict[str, List[float]] = {}

    def destroy(self):
        self._executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        rclpy.shutdown()

    # ── readiness ─────────────────────────────────────────────────────────

    def wait_until_ready(self, timeout: float = 120.0) -> bool:
        """Block until the action server and /nav_status are both live."""
        hdr("Waiting for mission_executive stack")
        deadline = time.monotonic() + timeout

        if not self._action_client.wait_for_server(timeout_sec=timeout):
            err("navigate_to_target action server never came up")
            return False
        info("action server up")

        while self.obs.snapshot()[0] is None:
            if time.monotonic() > deadline:
                err("/nav_status never published")
                return False
            time.sleep(0.2)
        info("/nav_status publishing")
        return True

    # ── setup spawns ──────────────────────────────────────────────────────

    def run_setup(self) -> bool:
        if not self.scenario.setup:
            return True
        hdr("Spawning scenario artifacts")
        for sp in self.scenario.setup:
            try:
                if sp.kind in ("aruco_post", "aruco"):
                    self.spawner.spawn_aruco_post(sp.name, sp.pose)
                elif sp.kind == "box":
                    self.spawner.spawn_box(sp.name, sp.pose, sp.size)
                else:
                    err(f"unknown spawn type '{sp.kind}'")
                    return False
                self._spawned[sp.name] = sp.pose
                info(f"spawned {sp.kind} '{sp.name}' at {sp.pose}")
            except SpawnError as e:
                err(str(e))
                return False
        return True

    # ── per-step execution + scoring ────────────────────────────────────────

    def _run_cli(self, cli_args: str, timeout: float) -> Optional[int]:
        cmd = ["ros2", "run", "mission_executive", "mission_cli"] + shlex.split(cli_args)
        print(f"  {DIM}$ {' '.join(cmd)}{RST}")
        try:
            proc = subprocess.run(cmd, timeout=timeout)
            return proc.returncode
        except subprocess.TimeoutExpired:
            warn(f"mission_cli timed out after {timeout:.0f}s — aborting mission")
            subprocess.run(["ros2", "run", "mission_executive", "mission_cli", "abort"],
                           timeout=15)
            return None

    def _ground_truth_dist(self, target_name: Optional[str]) -> Optional[float]:
        _, odom, _, _ = self.obs.snapshot()
        if odom is None:
            return None
        # Resolve which spawned entity to measure against.
        if target_name and target_name in self._spawned:
            tx, ty = self._spawned[target_name][0], self._spawned[target_name][1]
        elif len(self._spawned) == 1:
            (only,) = self._spawned.values()
            tx, ty = only[0], only[1]
        else:
            return None
        rx = odom.pose.pose.position.x
        ry = odom.pose.pose.position.y
        return math.hypot(rx - tx, ry - ty)

    def _score(self, step: Step) -> bool:
        a = step.assert_block
        kind = a.get("kind", "gnss")
        expect_state = a.get("expect_state", "STOPPED_AT_TARGET")
        max_dist = float(a.get("max_dist_m", 3.0))

        # Let the terminal state settle into the observers.
        time.sleep(1.5)
        nav, _, led, flash_seen = self.obs.snapshot()

        if nav is None:
            err("no /nav_status received — cannot score")
            return False

        state_ok = nav.state == expect_state
        print(f"    state         : {nav.state} "
              f"({'ok' if state_ok else 'expected ' + expect_state})")

        if kind == "gnss":
            dist = nav.distance_to_goal_m
            dist_ok = dist < max_dist
            print(f"    dist_to_goal  : {dist:.2f} m (< {max_dist} → {dist_ok})")
            return state_ok and dist_ok

        if kind in ("aruco_post", "aruco"):
            gt = self._ground_truth_dist(a.get("target"))
            gt_ok = gt is not None and gt < max_dist
            gt_s = f"{gt:.2f} m" if gt is not None else "unknown"
            print(f"    gt_dist_post  : {gt_s} (< {max_dist} → {gt_ok})")
            print(f"    flash_green   : {flash_seen} "
                  f"(latest cmd={getattr(led, 'cmd', '?')})")
            return state_ok and gt_ok and flash_seen

        err(f"unknown assert kind '{kind}'")
        return False

    def run_steps(self) -> bool:
        all_pass = True
        results = []
        for i, step in enumerate(self.scenario.steps, 1):
            hdr(f"Step {i}/{len(self.scenario.steps)}: {step.cli}")
            self.obs.reset_flash_latch()
            rc = self._run_cli(step.cli, self.scenario.timeout_s)
            if rc is None:
                passed = False
            elif not step.assert_block:
                # No assert → a non-nav setup step (e.g. set-target). Pass as
                # long as the CLI call itself completed.
                print(f"    {DIM}(no assert — setup step){RST}")
                passed = True
            else:
                passed = self._score(step)
            results.append((i, step.cli, passed))
            (info if passed else err)(f"step {i} {'PASS' if passed else 'FAIL'}")
            all_pass = all_pass and passed

        # ── summary ───────────────────────────────────────────────────────
        hdr(f"Scenario '{self.scenario.name}' summary")
        for i, cli, passed in results:
            tag = f"{GRN}PASS{RST}" if passed else f"{RED}FAIL{RST}"
            print(f"  [{tag}] step {i}: {cli}")
        print(f"\n  {BLD}{'ALL PASSED' if all_pass else 'FAILED'}{RST} "
              f"({sum(p for _, _, p in results)}/{len(results)} steps)")
        return all_pass


def main(argv: Optional[List[str]] = None) -> int:
    args = sys.argv[1:] if argv is None else list(argv)
    if not args:
        err("usage: python3 -m nav_sim_harness.runner <scenario.yaml>")
        return 2
    scenario = _load_scenario(args[0])

    runner = Runner(scenario)
    try:
        if not runner.wait_until_ready():
            return 1
        if not runner.run_setup():
            return 1
        return 0 if runner.run_steps() else 1
    finally:
        runner.destroy()


if __name__ == "__main__":
    raise SystemExit(main())

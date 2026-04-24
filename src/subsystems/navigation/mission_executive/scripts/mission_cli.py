#!/usr/bin/env python3
"""
mission_cli — Interactive CLI for the MissionExecutive node

Usage:
  ros2 run mission_executive mission_cli [command] [args...]

Commands:
  status                          Print current nav status once
  watch                           Stream nav status (Ctrl-C to stop)
  abort                           Abort the current mission
  teleop on|off                   Enable / disable teleop mode

  nav gps    <lat> <lon> [tol]   Navigate to GPS coords (GNSS_ONLY target)
  nav meter  <x>   <y>   [tol]   Navigate to ENU metre coords
  nav aruco  <lat> <lon> [tol]   Navigate + spiral for ArUco post
  nav object <lat> <lon> [tol]   Navigate + spiral for OBJECT detection

  set-target gps   <id> <lat> <lon> [type] [tol]   Register GPS target
  set-target meter <id> <x>   <y>   [type] [tol]   Register ENU target
  nav-by-id <id> [is_return]     Navigate to a pre-registered target id

  menu                            Interactive menu (default if no args)

target_type: 0=GNSS_ONLY  1=ARUCO_POST  2=OBJECT  3=LOCAL
tol: arrival radius in metres (default 3.0)
"""

import signal
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from msgs.action import NavigateToTarget
from msgs.msg import NavStatus
from msgs.srv import SetTarget
from std_srvs.srv import SetBool, Trigger

# ── Constants ─────────────────────────────────────────────────────────────────

NODE_NS = "/mission_executive"
STATUS_TOPIC = "/nav_status"

RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
DIM = "\033[2m"
RST = "\033[0m"

TARGET_TYPE_NAMES = {0: "GNSS_ONLY", 1: "ARUCO_POST", 2: "OBJECT", 3: "LOCAL"}

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN:   "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED:  "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED:  "CANCELED",
    GoalStatus.STATUS_ABORTED:   "ABORTED",
}

# ── Helpers ───────────────────────────────────────────────────────────────────

def info(msg):  print(f"{GRN}[✓]{RST} {msg}")
def warn(msg):  print(f"{YLW}[!]{RST} {msg}")
def error(msg): print(f"{RED}[✗]{RST} {msg}", file=sys.stderr)
def title(msg): print(f"\n{BLD}{msg}{RST}")

def target_type_name(t: int) -> str:
    return TARGET_TYPE_NAMES.get(t, f"UNKNOWN({t})")

# ── MissionCLI ────────────────────────────────────────────────────────────────

class MissionCLI:
    """
    Thin wrapper around rclpy primitives.  A single Node is created once; a
    background executor thread keeps it spinning so every async future and
    subscription callback is dispatched automatically.
    """

    def __init__(self):
        rclpy.init()
        self._node = rclpy.create_node("mission_cli")

        # Background executor — all ROS 2 I/O happens here
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="ros2_spin"
        )
        self._spin_thread.start()

        # Service clients (created once, reused for every call)
        self._abort_client = self._node.create_client(
            Trigger, f"{NODE_NS}/abort"
        )
        self._teleop_client = self._node.create_client(
            SetBool, f"{NODE_NS}/teleop"
        )
        self._set_target_client = self._node.create_client(
            SetTarget, f"{NODE_NS}/set_target"
        )

        # Action client
        self._action_client = ActionClient(
            self._node, NavigateToTarget, f"{NODE_NS}/navigate_to_target"
        )

    def destroy(self):
        self._executor.shutdown(timeout_sec=2.0)
        self._node.destroy_node()
        rclpy.shutdown()

    # ── Low-level helpers ──────────────────────────────────────────────────

    def _wait_for_service(self, client, timeout: float = 5.0) -> bool:
        """Poll service readiness without calling spin (executor already running)."""
        deadline = time.monotonic() + timeout
        while not client.service_is_ready():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def _wait_for_action_server(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while not self._action_client.server_is_ready():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def _call_service(self, client, request, timeout: float = 5.0):
        """Send a service request and block until the response arrives."""
        if not self._wait_for_service(client, timeout):
            error(f"Service {client.srv_name} not available")
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                error("Service call timed out")
                return None
            time.sleep(0.05)
        return future.result()

    def _get_status_once(self, timeout: float = 2.0) -> Optional[NavStatus]:
        """Subscribe to /nav_status, return the first message, then unsubscribe."""
        event = threading.Event()
        holder: list[Optional[NavStatus]] = [None]

        def _cb(msg: NavStatus):
            holder[0] = msg
            event.set()

        sub = self._node.create_subscription(NavStatus, STATUS_TOPIC, _cb, 10)
        try:
            event.wait(timeout=timeout)
        finally:
            self._node.destroy_subscription(sub)
        return holder[0]

    # ── cmd_status / cmd_watch ─────────────────────────────────────────────

    def cmd_status(self):
        title("Nav Status")
        msg = self._get_status_once()
        if msg is None:
            warn("No message received — is mission_executive running?")
            return
        print(f"  {'State:':<22} {BLD}{msg.state}{RST}")
        print(f"  {'Active target:':<22} {msg.active_target_id or '(none)'}")
        print(f"  {'Target type:':<22} {target_type_name(msg.active_target_type)}")
        print(f"  {'Dist to goal:':<22} {msg.distance_to_goal_m:.3f} m")
        print(f"  {'Cross-track err:':<22} {msg.cross_track_error_m:.3f} m")
        print(f"  {'Robot speed:':<22} {msg.robot_speed_mps:.3f} m/s")
        print(f"  {'Is return:':<22} {msg.is_return}")

    def cmd_watch(self):
        title(f"Watching {STATUS_TOPIC}  (Ctrl-C to stop)")
        done = threading.Event()

        def _cb(msg: NavStatus):
            print(
                f"state={msg.state:<20}  "
                f"tgt={msg.active_target_id:<12}  "
                f"dist={msg.distance_to_goal_m:>7.2f} m  "
                f"xte={msg.cross_track_error_m:>7.2f} m"
            )

        sub = self._node.create_subscription(NavStatus, STATUS_TOPIC, _cb, 10)
        try:
            done.wait()  # blocks until KeyboardInterrupt below
        except KeyboardInterrupt:
            pass
        finally:
            self._node.destroy_subscription(sub)

    # ── cmd_abort / cmd_teleop ─────────────────────────────────────────────

    def cmd_abort(self):
        title("Abort Mission")
        resp = self._call_service(self._abort_client, Trigger.Request())
        if resp:
            print(f"  success : {resp.success}")
            print(f"  message : {resp.message}")

    def cmd_teleop(self, mode: str):
        mode = mode.lower()
        if mode in ("on", "1", "true"):
            title("Teleop ON")
            req = SetBool.Request()
            req.data = True
        elif mode in ("off", "0", "false"):
            title("Teleop OFF")
            req = SetBool.Request()
            req.data = False
        else:
            error("Usage: teleop on|off")
            sys.exit(1)
        resp = self._call_service(self._teleop_client, req)
        if resp:
            print(f"  success : {resp.success}")
            print(f"  message : {resp.message}")

    # ── Live-status display while an action runs ───────────────────────────

    def _run_nav_with_status(self, goal: NavigateToTarget.Goal):
        target_type = goal.target_type
        if target_type in (1, 2):
            print(f"  {CYN}Note: After arrival the node will enter SPIRAL_COVERAGE{RST}")
            print(f"  {CYN}      and complete when a detection is received or it times out.{RST}")
            print()

        if not self._wait_for_action_server(timeout=5.0):
            error("navigate_to_target action server not available")
            return

        prev_state: list[str] = [""]
        start_t = time.monotonic()
        interrupted = False
        goal_handle_holder: list = [None]
        goal_accepted_event = threading.Event()
        result_event = threading.Event()
        result_holder: list = [None]

        def _feedback_cb(feedback_msg):
            fb = feedback_msg.feedback
            elapsed = int(time.monotonic() - start_t)
            if fb.state != prev_state[0]:
                print(f"\n  {YLW}→ State: {BLD}{fb.state}{RST}")
                prev_state[0] = fb.state
            print(
                f"\r  [{elapsed:3d}s]  "
                f"dist={fb.distance_to_goal_m:>8.2f} m  "
                f"xte={fb.cross_track_error_m:>8.2f} m  ",
                end="", flush=True,
            )

        def _on_result(future):
            result_holder[0] = future.result()
            result_event.set()

        def _on_goal_response(future):
            gh = future.result()
            goal_handle_holder[0] = gh
            if gh and gh.accepted:
                gh.get_result_async().add_done_callback(_on_result)
            goal_accepted_event.set()

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=_feedback_cb
        )
        send_future.add_done_callback(_on_goal_response)

        def _on_sigint(sig, frame):
            nonlocal interrupted
            interrupted = True
            print()
            warn("Interrupted — calling abort...")
            self._abort_client.call_async(Trigger.Request())
            gh = goal_handle_holder[0]
            if gh:
                gh.cancel_goal_async()
            result_event.set()

        old_handler = signal.signal(signal.SIGINT, _on_sigint)
        print(f"  {DIM}Waiting for goal acceptance...{RST}")

        goal_accepted_event.wait(timeout=10.0)
        gh = goal_handle_holder[0]
        if not gh or not gh.accepted:
            error("Goal was rejected by the action server")
            signal.signal(signal.SIGINT, old_handler)
            return

        print(f"  {DIM}Goal accepted — navigating...{RST}")
        result_event.wait()

        signal.signal(signal.SIGINT, old_handler)
        print("\n")

        if not interrupted:
            result_wrapper = result_holder[0]
            if result_wrapper:
                print(f"{BLD}── Action Result ──────────────────────────────{RST}")
                print(f"  success : {result_wrapper.result.success}")
                print(f"  message : {result_wrapper.result.message}")
                status = GOAL_STATUS_NAMES.get(result_wrapper.status,
                                               str(result_wrapper.status))
                print(f"  status  : {status}")
                print(f"{BLD}───────────────────────────────────────────────{RST}")

    # ── _build_nav_goal / cmd_nav ──────────────────────────────────────────

    def _build_nav_goal(
        self,
        goal_type: int,
        target_id: str,
        a: str,
        b: str,
        target_type: int = 0,
        tolerance: float = 3.0,
        is_return: bool = False,
    ) -> NavigateToTarget.Goal:
        goal = NavigateToTarget.Goal()
        goal.target_id  = target_id
        goal.goal_type  = goal_type
        goal.target_type = target_type
        goal.tolerance_m = tolerance
        goal.is_return  = is_return
        if goal_type == NavigateToTarget.Goal.GPS:
            goal.lat = float(a)
            goal.lon = float(b)
        else:
            goal.x_m = float(a)
            goal.y_m = float(b)

        title("navigate_to_target")
        print(f"  target_id   : {target_id or '(inline)'}")
        print(f"  goal_type   : {'GPS' if goal_type == 0 else 'METER'}")
        if goal_type == NavigateToTarget.Goal.GPS:
            print(f"  lat / lon   : {goal.lat} / {goal.lon}")
        else:
            print(f"  x_m / y_m   : {goal.x_m} / {goal.y_m}")
        print(f"  target_type : {target_type_name(target_type)}")
        print(f"  tolerance_m : {tolerance}")
        print(f"  is_return   : {is_return}")
        print()
        return goal

    def _nav_usage(self):
        error("Usage:")
        error("  nav gps    <lat> <lon>  [tol]   — GNSS_ONLY target")
        error("  nav meter  <x>   <y>    [tol]   — ENU metre target")
        error("  nav aruco  <lat> <lon>  [tol]   — navigate + spiral (ArUco)")
        error("  nav object <lat> <lon>  [tol]   — navigate + spiral (YOLO object)")
        sys.exit(1)

    def cmd_nav(self, args: list):
        if not args:
            self._nav_usage()
        coord = args[0].lower()
        tol = float(args[3]) if len(args) > 3 else 3.0

        if coord == "gps":
            if len(args) < 3: self._nav_usage()
            goal = self._build_nav_goal(0, "", args[1], args[2], 0, tol)
        elif coord in ("meter", "m"):
            if len(args) < 3: self._nav_usage()
            goal = self._build_nav_goal(1, "", args[1], args[2], 0, tol)
        elif coord == "aruco":
            if len(args) < 3: self._nav_usage()
            goal = self._build_nav_goal(0, "", args[1], args[2], 1, tol)
        elif coord in ("object", "obj"):
            if len(args) < 3: self._nav_usage()
            goal = self._build_nav_goal(0, "", args[1], args[2], 2, tol)
        else:
            self._nav_usage()
            return
        self._run_nav_with_status(goal)

    def cmd_nav_by_id(self, target_id: str, is_return: bool = False):
        title(f"navigate_to_target  (id={target_id}  is_return={is_return})")
        goal = NavigateToTarget.Goal()
        goal.target_id   = target_id
        goal.goal_type   = NavigateToTarget.Goal.GPS
        goal.target_type = NavigateToTarget.Goal.GNSS_ONLY
        goal.tolerance_m = 3.0
        goal.is_return   = is_return
        self._run_nav_with_status(goal)

    # ── cmd_set_target ─────────────────────────────────────────────────────

    def _set_target_usage(self):
        error("Usage: set-target gps   <id> <lat> <lon> [type 0-3] [tol]")
        error("       set-target meter <id> <x>   <y>   [type 0-3] [tol]")
        sys.exit(1)

    def cmd_set_target(self, args: list):
        if not args:
            self._set_target_usage()
        coord = args[0].lower()
        if coord == "gps":
            if len(args) < 4: self._set_target_usage()
            self._call_set_target(0, args[1], args[2], args[3],
                                  int(args[4]) if len(args) > 4 else 0,
                                  float(args[5]) if len(args) > 5 else 3.0)
        elif coord in ("meter", "m"):
            if len(args) < 4: self._set_target_usage()
            self._call_set_target(1, args[1], args[2], args[3],
                                  int(args[4]) if len(args) > 4 else 0,
                                  float(args[5]) if len(args) > 5 else 3.0)
        else:
            self._set_target_usage()

    def _call_set_target(self, goal_type: int, target_id: str, a: str, b: str,
                         target_type: int = 0, tolerance: float = 3.0):
        req = SetTarget.Request()
        req.target_id   = target_id
        req.goal_type   = goal_type
        req.target_type = target_type
        req.tolerance_m = tolerance
        if goal_type == SetTarget.Request.GPS:
            req.lat = float(a)
            req.lon = float(b)
        else:
            req.x_m = float(a)
            req.y_m = float(b)

        title(f"set_target  (id={target_id}  type={target_type_name(target_type)})")
        resp = self._call_service(self._set_target_client, req)
        if resp:
            print(f"  success : {resp.success}")
            print(f"  message : {resp.message}")

    # ── Interactive menu ───────────────────────────────────────────────────

    def _menu_nav_goal(self):
        print()
        print("Navigation type:")
        print("  1) GPS point  (GNSS_ONLY)")
        print("  2) GPS point  (ArUco post  — will spiral after arrival)")
        print("  3) GPS point  (YOLO object — will spiral after arrival)")
        print("  4) ENU metres (GNSS_ONLY)")
        print("  5) By registered target ID")
        ct = input("Choice: ").strip()

        if ct == "5":
            tid = input("Target ID: ").strip()
            ret = input("Is return? [y/N]: ").strip().lower()
            self.cmd_nav_by_id(tid, ret in ("y", "yes"))
            return

        goal_type, target_type = 0, 0
        if ct == "2":   target_type = 1
        elif ct == "3": target_type = 2
        elif ct == "4": goal_type   = 1

        if goal_type == 0:
            a = input("Latitude:  ").strip()
            b = input("Longitude: ").strip()
        else:
            a = input("x_m: ").strip()
            b = input("y_m: ").strip()

        tol_s = input("Arrival tolerance in metres [3.0]: ").strip()
        goal = self._build_nav_goal(goal_type, "", a, b, target_type,
                                    float(tol_s) if tol_s else 3.0, False)
        self._run_nav_with_status(goal)

    def _menu_set_target(self):
        print()
        print("Coordinate type:")
        print("  1) GPS (lat/lon)    2) ENU metres (x/y)")
        ct   = input("Choice: ").strip()
        tid  = input("Target ID: ").strip()
        if ct == "1":
            a, b, goal_type = input("Latitude:  ").strip(), input("Longitude: ").strip(), 0
        else:
            a, b, goal_type = input("x_m: ").strip(), input("y_m: ").strip(), 1
        print("Target type:  0=GNSS_ONLY  1=ARUCO_POST  2=OBJECT  3=LOCAL")
        ttype_s = input("Target type [0]: ").strip()
        tol_s   = input("Arrival tolerance [3.0]: ").strip()
        self._call_set_target(goal_type, tid, a, b,
                              int(ttype_s) if ttype_s else 0,
                              float(tol_s) if tol_s else 3.0)

    def cmd_menu(self):
        while True:
            print()
            print(f"{BLD}╔══ Mission Executive CLI ════════════════════╗{RST}")
            print(f"{BLD}║{RST}  1) Status (snapshot)                       {BLD}║{RST}")
            print(f"{BLD}║{RST}  2) Watch status (stream)                   {BLD}║{RST}")
            print(f"{BLD}║{RST}  3) Navigate (GPS / meter / aruco / object) {BLD}║{RST}")
            print(f"{BLD}║{RST}  4) Register target (set_target service)    {BLD}║{RST}")
            print(f"{BLD}║{RST}  5) Abort mission                           {BLD}║{RST}")
            print(f"{BLD}║{RST}  6) Teleop ON                               {BLD}║{RST}")
            print(f"{BLD}║{RST}  7) Teleop OFF                              {BLD}║{RST}")
            print(f"{BLD}║{RST}  q) Quit                                    {BLD}║{RST}")
            print(f"{BLD}╚═════════════════════════════════════════════╝{RST}")
            try:
                choice = input("Choice: ").strip()
            except (KeyboardInterrupt, EOFError):
                info("Bye.")
                return

            if   choice == "1": self.cmd_status()
            elif choice == "2": self.cmd_watch()
            elif choice == "3": self._menu_nav_goal()
            elif choice == "4": self._menu_set_target()
            elif choice == "5": self.cmd_abort()
            elif choice == "6": self.cmd_teleop("on")
            elif choice == "7": self.cmd_teleop("off")
            elif choice in ("q", "Q"):
                info("Bye.")
                return
            else:
                warn(f"Unknown option '{choice}'")

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    args = sys.argv[1:]
    cmd  = args[0] if args else "menu"
    rest = args[1:]

    cli = MissionCLI()
    try:
        if cmd in ("-h", "--help", "help"):
            print(__doc__)
        elif cmd == "status":
            cli.cmd_status()
        elif cmd == "watch":
            cli.cmd_watch()
        elif cmd == "abort":
            cli.cmd_abort()
        elif cmd == "teleop":
            if not rest:
                error("Usage: teleop on|off"); sys.exit(1)
            cli.cmd_teleop(rest[0])
        elif cmd == "nav":
            cli.cmd_nav(rest)
        elif cmd == "nav-by-id":
            if not rest:
                error("nav-by-id requires a target_id"); sys.exit(1)
            is_return = len(rest) > 1 and rest[1].lower() in ("true", "1", "yes")
            cli.cmd_nav_by_id(rest[0], is_return)
        elif cmd == "set-target":
            cli.cmd_set_target(rest)
        elif cmd == "menu":
            cli.cmd_menu()
        else:
            error(f"Unknown command: {cmd}")
            print("Run with 'help' for usage.")
            sys.exit(1)
    finally:
        cli.destroy()


if __name__ == "__main__":
    main()

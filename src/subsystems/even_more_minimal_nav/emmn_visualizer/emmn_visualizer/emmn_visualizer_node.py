import math
import threading
from collections import deque

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import CheckButtons
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, QoSPresetProfiles
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Bool, String

from msgs.msg import NavStatus, Heading, PlannerEvent

# ─── Constants ────────────────────────────────────────────────────────────────

STATE_COLORS = {
    'IDLE':              '#888888',
    'NAVIGATING':        '#00cc55',
    'ARRIVING':          '#ffaa00',
    'STOPPED_AT_TARGET': '#44aaff',
    'STOPPED_AT_RETURN': '#44aaff',
    'ABORTING':          '#ff3333',
    'RETURNING':         '#aa44ff',
    'TELEOP':            '#ff8800',
}
DEFAULT_STATE_COLOR = '#444444'

PLANNER_EVENT_NAMES = {
    PlannerEvent.NEW_GOAL:       'NEW_GOAL',
    PlannerEvent.PLANNING:       'PLANNING',
    PlannerEvent.PLAN_SUCCEEDED: 'PLAN_OK',
    PlannerEvent.PLAN_FAILED:    'PLAN_FAILED',
}
PLAN_FAILED_COLOR = '#ff4444'
PLAN_OK_COLOR     = '#44ff88'

# ─── Coordinate utilities ──────────────────────────────────────────────────────

def quat_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CoordinateConverter:
    """WGS-84 ENU ↔ LLA conversion for hover coordinate display."""

    def __init__(self):
        self.origin_lat = None
        self.a  = 6378137.0
        self.f  = 1 / 298.257223563
        self.b  = self.a * (1 - self.f)
        self.e2 = (self.a**2 - self.b**2) / self.a**2

    def set_origin(self, lat, lon, alt):
        self.origin_lat = lat
        lr, lo = math.radians(lat), math.radians(lon)
        self.origin_lat_rad, self.origin_lon_rad = lr, lo
        self.x0, self.y0, self.z0 = self._lla_to_ecef(lat, lon, alt)
        self.s_lat, self.c_lat = math.sin(lr), math.cos(lr)
        self.s_lon, self.c_lon = math.sin(lo), math.cos(lo)

    def _lla_to_ecef(self, lat, lon, alt):
        lr, lo = math.radians(lat), math.radians(lon)
        N = self.a / math.sqrt(1 - self.e2 * math.sin(lr)**2)
        return (
            (N + alt) * math.cos(lr) * math.cos(lo),
            (N + alt) * math.cos(lr) * math.sin(lo),
            (N * (1 - self.e2) + alt) * math.sin(lr),
        )

    def enu_to_lla(self, e, n, u=0.0):
        if self.origin_lat is None:
            return None, None
        dx = -self.s_lon * e - self.c_lon * self.s_lat * n + self.c_lon * self.c_lat * u
        dy =  self.c_lon * e - self.s_lon * self.s_lat * n + self.s_lon * self.c_lat * u
        dz =  self.c_lat * n + self.s_lat * u
        x, y, z = dx + self.x0, dy + self.y0, dz + self.z0
        lon = math.atan2(y, x)
        p   = math.sqrt(x**2 + y**2)
        lat = math.atan2(z, p * (1 - self.e2))
        for _ in range(5):
            N   = self.a / math.sqrt(1 - self.e2 * math.sin(lat)**2)
            lat = math.atan2(z + self.e2 * N * math.sin(lat), p)
        return math.degrees(lat), math.degrees(lon)


# ─── ROS node (data collection only) ─────────────────────────────────────────

class EMMNVisualizerNode(Node):
    def __init__(self):
        super().__init__('emmn_visualizer')

        self.lock = threading.Lock()

        # Data store
        self.robot_pose        = None
        self.global_path       = None
        self.goal_pose         = None
        self.nav_status        = None
        self.nav_enabled       = False
        self.nav_mode          = 'stopped'
        self.heading_msg       = None
        self.costmap           = None
        self.latest_scan       = None   # sensor_msgs/LaserScan
        self.converter         = CoordinateConverter()
        self.event_log         = deque(maxlen=10)   # (timestamp_str, event_name, is_failure)
        self.new_path_received    = False
        self.new_costmap_received = False

        # QoS profiles
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(PoseStamped,   '/robot_pose',      self._pose_cb,    10)
        self.create_subscription(Path,          '/global_path',     self._path_cb,    transient_qos)
        self.create_subscription(OccupancyGrid, '/map',             self._map_cb,     transient_qos)
        self.create_subscription(PoseStamped,   '/goal_pose',       self._goal_cb,    10)
        self.create_subscription(NavStatus,     '/nav_status',      self._status_cb,  10)
        self.create_subscription(Bool,          '/nav_enabled',     self._enabled_cb, reliable_qos)
        self.create_subscription(String,        '/nav_mode',        self._mode_cb,    10)
        self.create_subscription(PlannerEvent,  '/planner_event',   self._event_cb,   10)
        self.create_subscription(NavSatFix,     '/gps/fix',         self._gps_cb,     10)
        self.create_subscription(Heading,       '/gps/heading',     self._heading_cb, 10)
        self.create_subscription(LaserScan,     '/scan',            self._scan_cb,
                                 QoSPresetProfiles.SENSOR_DATA.value)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _pose_cb(self, msg):
        with self.lock:
            self.robot_pose = msg

    def _path_cb(self, msg):
        with self.lock:
            self.global_path      = msg
            self.new_path_received = True

    def _map_cb(self, msg):
        with self.lock:
            self.costmap              = msg
            self.new_costmap_received = True

    def _goal_cb(self, msg):
        with self.lock:
            self.goal_pose = msg

    def _status_cb(self, msg):
        with self.lock:
            self.nav_status = msg

    def _enabled_cb(self, msg):
        with self.lock:
            self.nav_enabled = msg.data

    def _mode_cb(self, msg):
        with self.lock:
            self.nav_mode = msg.data

    def _event_cb(self, msg):
        now    = self.get_clock().now().to_msg()
        ts     = f'{now.sec % 86400 // 3600:02d}:{now.sec % 3600 // 60:02d}:{now.sec % 60:02d}'
        name   = PLANNER_EVENT_NAMES.get(msg.event, f'EVENT_{msg.event}')
        failed = (msg.event == PlannerEvent.PLAN_FAILED)
        with self.lock:
            self.event_log.append((ts, name, failed))

    def _gps_cb(self, msg):
        with self.lock:
            if self.converter.origin_lat is None and msg.status.status >= 0:
                self.converter.set_origin(msg.latitude, msg.longitude, msg.altitude)

    def _heading_cb(self, msg):
        with self.lock:
            self.heading_msg = msg

    def _scan_cb(self, msg):
        with self.lock:
            self.latest_scan = msg


# ─── Plotting ─────────────────────────────────────────────────────────────────

def _find_lookahead(path_poses, rx, ry, lookahead_m):
    """Approximate the VFP lookahead point: closest path index + walk forward."""
    if not path_poses:
        return None
    best_i, best_d2 = 0, float('inf')
    for i, p in enumerate(path_poses):
        dx, dy = p.pose.position.x - rx, p.pose.position.y - ry
        d2 = dx*dx + dy*dy
        if d2 < best_d2:
            best_d2, best_i = d2, i
    for i in range(best_i, len(path_poses)):
        dx = path_poses[i].pose.position.x - rx
        dy = path_poses[i].pose.position.y - ry
        if math.hypot(dx, dy) >= lookahead_m:
            return path_poses[i].pose.position.x, path_poses[i].pose.position.y
    last = path_poses[-1].pose.position
    return last.x, last.y


def _closest_path_point(path_poses, rx, ry):
    """Return the closest point on the path (segment-level) and its distance."""
    if not path_poses or len(path_poses) < 2:
        return None
    min_dist, best_pt = float('inf'), None
    for i in range(len(path_poses) - 1):
        ax, ay = path_poses[i].pose.position.x,   path_poses[i].pose.position.y
        bx, by = path_poses[i+1].pose.position.x, path_poses[i+1].pose.position.y
        dx, dy = bx - ax, by - ay
        len2   = dx*dx + dy*dy
        if len2 < 1e-10:
            cx, cy = ax, ay
        else:
            t      = max(0.0, min(1.0, ((rx-ax)*dx + (ry-ay)*dy) / len2))
            cx, cy = ax + t*dx, ay + t*dy
        d = math.hypot(rx - cx, ry - cy)
        if d < min_dist:
            min_dist, best_pt = d, (cx, cy)
    return best_pt


class VisualizerPlot:
    LOOKAHEAD_M = 3.0   # default — matches nav_params.yaml lookahead_dist_m

    def __init__(self, node: EMMNVisualizerNode):
        self.node = node

        # ── Figure layout ──────────────────────────────────────────────────────
        self.fig = plt.figure(figsize=(15, 9))
        self.fig.patch.set_facecolor('#1a1a2e')

        gs = gridspec.GridSpec(
            1, 2,
            width_ratios=[3, 1],
            left=0.04, right=0.98,
            top=0.96, bottom=0.10,
            wspace=0.04,
        )

        self.ax_map    = self.fig.add_subplot(gs[0, 0])
        self.ax_status = self.fig.add_subplot(gs[0, 1])

        self._style_map_axes()
        self._style_status_axes()

        # ── Map elements ────────────────────────────────────────────────────────
        # Costmap (imshow)
        self.costmap_img = None  # created lazily on first costmap

        # Global path
        self.path_line,   = self.ax_map.plot([], [], '--',  color='#5588ff',
                                              linewidth=1.8, label='Global Path',  zorder=2)

        # Cross-track error: line from robot to closest path point
        self.xte_line,    = self.ax_map.plot([], [], '-',   color='#ffff00',
                                              linewidth=1.5, label='XTE',           zorder=5,
                                              linestyle='dashed')
        self.xte_dot,     = self.ax_map.plot([], [], 'o',   color='#ffff00',
                                              markersize=6,                          zorder=6)

        # Lookahead circle + line + point
        self.lookahead_circle = plt.Circle((0, 0), self.LOOKAHEAD_M,
                                           color='#88ccff', fill=False, linewidth=1.2,
                                           linestyle='dotted', zorder=4)
        self.ax_map.add_patch(self.lookahead_circle)
        self.lookahead_line,  = self.ax_map.plot([], [], '-',  color='#88ccff',
                                                  linewidth=1.2, zorder=5, linestyle='dotted')
        self.lookahead_pt,    = self.ax_map.plot([], [], 'D',  color='#88ccff',
                                                  markersize=7,  zorder=7, label='Lookahead')

        # Robot marker + heading arrow
        self.robot_dot,   = self.ax_map.plot([], [], 'o',   color='#ff4444',
                                              markersize=10, zorder=10, label='Robot')
        self.heading_line,= self.ax_map.plot([], [], '-',   color='#ffaa00',
                                              linewidth=2.5, zorder=11)

        # Goal marker
        self.goal_marker, = self.ax_map.plot([], [], 'x',   color='white',
                                              markersize=12, markeredgewidth=2.5,
                                              zorder=9, label='Goal')

        # Laser scan hits — scatter of obstacle points in map frame
        self.scan_scatter = self.ax_map.scatter(
            [], [], s=4, c='#ff6633', alpha=0.65, zorder=8, label='Scan hits')

        self.ax_map.legend(loc='lower left', framealpha=0.6,
                           facecolor='#1a1a2e', labelcolor='white',
                           fontsize=9)

        # Hover text
        self.hover_text = self.ax_map.text(
            0.02, 0.02, '', transform=self.ax_map.transAxes,
            fontsize=8, color='#cccccc',
            bbox=dict(facecolor='#1a1a2e', alpha=0.8, edgecolor='#555555'),
            verticalalignment='bottom',
        )

        # ── Status panel elements ───────────────────────────────────────────────
        self.status_text = self.ax_status.text(
            0.05, 0.98, 'Waiting for data...',
            transform=self.ax_status.transAxes,
            fontsize=9, color='white', family='monospace',
            verticalalignment='top',
            bbox=dict(facecolor='#0d0d1a', alpha=0.0, edgecolor='none'),
        )

        # ── Checkboxes (bottom strip) ───────────────────────────────────────────
        cb_labels = ['Auto-Fit', 'Show Costmap', 'Show Lookahead', 'Show XTE', 'Show Scan']
        cb_states = [True,       True,            True,             True,        True]
        rax = self.fig.add_axes([0.04, 0.01, 0.38, 0.07])
        rax.set_facecolor('#1a1a2e')
        self.check = CheckButtons(rax, cb_labels, cb_states)
        for txt in self.check.labels:
            txt.set_color('white')
            txt.set_fontsize(9)
        self.auto_fit       = True
        self.show_costmap   = True
        self.show_lookahead = True
        self.show_xte       = True
        self.show_scan      = True
        self.check.on_clicked(self._on_toggle)

        # ── Events ─────────────────────────────────────────────────────────────
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_hover)

    # ── Axis styling ───────────────────────────────────────────────────────────

    def _style_map_axes(self):
        ax = self.ax_map
        ax.set_facecolor('#0d0d1a')
        ax.tick_params(colors='#888888', labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor('#333355')
        ax.grid(True, linestyle=':', alpha=0.3, color='#444466')
        ax.set_aspect('equal')
        ax.set_xlabel('East (m)', color='#888888', fontsize=9)
        ax.set_ylabel('North (m)', color='#888888', fontsize=9)
        ax.set_title('Even More Minimal Nav — Live Visualizer', color='#aaaacc', fontsize=11)

    def _style_status_axes(self):
        ax = self.ax_status
        ax.set_facecolor('#0d0d1a')
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis('off')
        for spine in ax.spines.values():
            spine.set_edgecolor('#333355')

    # ── Toggles ────────────────────────────────────────────────────────────────

    def _on_toggle(self, label):
        if label == 'Auto-Fit':
            self.auto_fit = not self.auto_fit
            if self.auto_fit:
                with self.node.lock:
                    self.node.new_path_received = True
        elif label == 'Show Costmap':
            self.show_costmap = not self.show_costmap
            if self.costmap_img is not None:
                self.costmap_img.set_visible(self.show_costmap)
        elif label == 'Show Lookahead':
            self.show_lookahead = not self.show_lookahead
        elif label == 'Show XTE':
            self.show_xte = not self.show_xte
        elif label == 'Show Scan':
            self.show_scan = not self.show_scan
            self.scan_scatter.set_visible(self.show_scan)

    # ── Hover ──────────────────────────────────────────────────────────────────

    def _on_hover(self, event):
        if event.inaxes != self.ax_map or event.xdata is None:
            return
        with self.node.lock:
            lat, lon = self.node.converter.enu_to_lla(event.xdata, event.ydata)
        if lat is not None:
            self.hover_text.set_text(
                f'E:{event.xdata:+.1f}m  N:{event.ydata:+.1f}m\n'
                f'Lat:{lat:.6f}  Lon:{lon:.6f}'
            )
        else:
            self.hover_text.set_text(f'E:{event.xdata:+.1f}m  N:{event.ydata:+.1f}m')

    # ── Main update (called at ~10 Hz by FuncAnimation) ───────────────────────

    def update(self, _frame):
        with self.node.lock:
            robot_pose   = self.node.robot_pose
            global_path  = self.node.global_path
            goal_pose    = self.node.goal_pose
            nav_status   = self.node.nav_status
            nav_enabled  = self.node.nav_enabled
            nav_mode     = self.node.nav_mode
            heading_msg  = self.node.heading_msg
            costmap      = self.node.costmap
            latest_scan  = self.node.latest_scan
            event_log    = list(self.node.event_log)
            new_path     = self.node.new_path_received
            new_costmap  = self.node.new_costmap_received

            if new_costmap:
                self.node.new_costmap_received = False

        path_poses = global_path.poses if global_path else []

        rx = ry = yaw = None
        if robot_pose:
            rx, ry = robot_pose.pose.position.x, robot_pose.pose.position.y
            if heading_msg:
                yaw = math.radians(heading_msg.heading)
            else:
                yaw = quat_to_yaw(robot_pose.pose.orientation)

        # ── Costmap ─────────────────────────────────────────────────────────────
        if new_costmap and costmap is not None:
            self._render_costmap(costmap)

        # ── Global path ─────────────────────────────────────────────────────────
        if path_poses:
            xs = [p.pose.position.x for p in path_poses]
            ys = [p.pose.position.y for p in path_poses]
            self.path_line.set_data(xs, ys)
        else:
            self.path_line.set_data([], [])

        # ── Robot + heading ─────────────────────────────────────────────────────
        if rx is not None:
            self.robot_dot.set_data([rx], [ry])
            L = 2.5
            self.heading_line.set_data(
                [rx, rx + L * math.cos(yaw)],
                [ry, ry + L * math.sin(yaw)],
            )
            self.lookahead_circle.center = (rx, ry)
        else:
            self.robot_dot.set_data([], [])
            self.heading_line.set_data([], [])

        # ── Goal ────────────────────────────────────────────────────────────────
        if goal_pose:
            self.goal_marker.set_data(
                [goal_pose.pose.position.x], [goal_pose.pose.position.y])
        else:
            self.goal_marker.set_data([], [])

        # ── XTE indicator ───────────────────────────────────────────────────────
        if self.show_xte and rx is not None and len(path_poses) >= 2:
            cp = _closest_path_point(path_poses, rx, ry)
            if cp:
                self.xte_line.set_data([rx, cp[0]], [ry, cp[1]])
                self.xte_dot.set_data([cp[0]], [cp[1]])
            else:
                self.xte_line.set_data([], [])
                self.xte_dot.set_data([], [])
        else:
            self.xte_line.set_data([], [])
            self.xte_dot.set_data([], [])

        # ── Lookahead ────────────────────────────────────────────────────────────
        lp = None
        if self.show_lookahead and rx is not None and path_poses:
            lp = _find_lookahead(path_poses, rx, ry, self.LOOKAHEAD_M)
        self.lookahead_circle.set_visible(self.show_lookahead and rx is not None)
        if lp and self.show_lookahead:
            self.lookahead_pt.set_data([lp[0]], [lp[1]])
            self.lookahead_line.set_data([rx, lp[0]], [ry, lp[1]])
        else:
            self.lookahead_pt.set_data([], [])
            self.lookahead_line.set_data([], [])

        # ── Laser scan hits ──────────────────────────────────────────────────────
        # Convert scan from base_link polar coords → map Cartesian and scatter-plot.
        # Assumes scan is in base_link frame (target_frame: base_link in p2l config).
        if self.show_scan and latest_scan is not None and rx is not None:
            sc = latest_scan
            pts_x, pts_y = [], []
            for i, r in enumerate(sc.ranges):
                if not math.isfinite(r) or r < sc.range_min or r > sc.range_max:
                    continue
                alpha = sc.angle_min + i * sc.angle_increment   # angle in base_link
                map_x = rx + r * math.cos(yaw + alpha)
                map_y = ry + r * math.sin(yaw + alpha)
                pts_x.append(map_x)
                pts_y.append(map_y)
            if pts_x:
                self.scan_scatter.set_offsets(np.column_stack([pts_x, pts_y]))
            else:
                self.scan_scatter.set_offsets(np.empty((0, 2)))
            self.scan_scatter.set_visible(True)
        else:
            self.scan_scatter.set_offsets(np.empty((0, 2)))
            self.scan_scatter.set_visible(self.show_scan)

        # ── Viewport ─────────────────────────────────────────────────────────────
        if self.auto_fit and new_path:
            self._fit_viewport(path_poses, rx, ry)
            with self.node.lock:
                self.node.new_path_received = False

        # ── Status panel ─────────────────────────────────────────────────────────
        self._update_status(nav_status, nav_enabled, nav_mode, event_log,
                            heading_msg, costmap, path_poses, rx, ry,
                            latest_scan=latest_scan)

    # ── Costmap renderer ─────────────────────────────────────────────────────

    def _render_costmap(self, costmap):
        info = costmap.info
        W, H = info.width, info.height
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        arr = np.array(costmap.data, dtype=np.int16).reshape(H, W).astype(np.float32)
        arr[arr < 0] = np.nan          # unknown cells → transparent
        arr = np.clip(arr, 0, 254) / 254.0

        extent = [ox, ox + W * res, oy, oy + H * res]
        cmap = plt.cm.RdYlGn_r.copy()
        cmap.set_bad(color='none')

        if self.costmap_img is None:
            self.costmap_img = self.ax_map.imshow(
                arr, origin='lower', extent=extent,
                cmap=cmap, vmin=0.0, vmax=1.0,
                alpha=0.45, zorder=1,
                interpolation='nearest',
            )
            self.fig.colorbar(
                self.costmap_img, ax=self.ax_map,
                fraction=0.025, pad=0.01,
                label='Slope Cost (0=safe, 1=lethal)',
            ).ax.yaxis.label.set_color('#888888')
        else:
            self.costmap_img.set_data(arr)
            self.costmap_img.set_extent(extent)

        self.costmap_img.set_visible(self.show_costmap)

    # ── Viewport fit ────────────────────────────────────────────────────────────

    def _fit_viewport(self, path_poses, rx, ry):
        pts_x, pts_y = [], []
        for p in path_poses:
            pts_x.append(p.pose.position.x)
            pts_y.append(p.pose.position.y)
        if rx is not None:
            pts_x.append(rx)
            pts_y.append(ry)
        if not pts_x:
            return
        mn_x, mx_x = min(pts_x), max(pts_x)
        mn_y, mx_y = min(pts_y), max(pts_y)
        dx = max(mx_x - mn_x, 15.0)
        dy = max(mx_y - mn_y, 15.0)
        cx = (mn_x + mx_x) / 2.0
        cy = (mn_y + mx_y) / 2.0
        pad = 0.65
        self.ax_map.set_xlim(cx - dx * pad, cx + dx * pad)
        self.ax_map.set_ylim(cy - dy * pad, cy + dy * pad)

    # ── Status panel text ────────────────────────────────────────────────────────

    def _update_status(self, nav_status, nav_enabled, nav_mode, event_log,
                       heading_msg, costmap, path_poses, rx, ry, latest_scan=None):
        state  = nav_status.state if nav_status else 'NO DATA'
        color  = STATE_COLORS.get(state, DEFAULT_STATE_COLOR)

        lines = []
        lines.append(f'{"═"*26}')
        lines.append(f'  {state:^24s}')
        lines.append(f'{"═"*26}')

        # Target info
        if nav_status and nav_status.active_target_id:
            ttype = nav_status.active_target_type
            lines.append(f'  TARGET : {nav_status.active_target_id}')
            lines.append(f'  Type   : {ttype}')
            lines.append(f'  Return : {"Yes" if nav_status.is_return else "No"}')
        else:
            lines.append('  TARGET : (none)')
        lines.append('')

        # Metrics
        lines.append('── Metrics ──────────────────')
        if nav_status:
            dist = nav_status.distance_to_goal_m
            xte  = nav_status.cross_track_error_m
            herr = math.degrees(nav_status.heading_error_rad)
            avel = nav_status.robot_speed_mps   # actually imu angular vel
            dist_str = f'{dist:.1f}m'   if dist >= 0 else 'N/A'
            xte_str  = f'{xte:.2f}m'   if xte  >= 0 else 'N/A'
            lines.append(f'  Dist to Goal : {dist_str:>8s}')
            lines.append(f'  Cross-Track  : {xte_str:>8s}')
            lines.append(f'  Heading Err  : {herr:>+7.1f}°')
            lines.append(f'  AngVel (IMU) : {avel:>7.3f} r/s')
        else:
            lines.append('  (no nav_status)')
        lines.append('')

        # Heading
        lines.append('── Heading ──────────────────')
        if heading_msg:
            lines.append(f'  ENU heading  : {heading_msg.heading:>+7.1f}°')
            lines.append(f'  Compass      : {heading_msg.compass_bearing:>+7.1f}°')
        else:
            lines.append('  (no heading msg)')
        lines.append('')

        # System state
        lines.append('── System ───────────────────')
        enabled_str = 'YES' if nav_enabled else 'NO'
        lines.append(f'  Nav Enabled : {enabled_str}')
        lines.append(f'  Nav Mode    : {nav_mode}')
        if costmap is not None:
            W = costmap.info.width
            H = costmap.info.height
            lines.append(f'  Costmap     : {W}×{H}')
        else:
            lines.append('  Costmap     : not received')
        lines.append(f'  Path poses  : {len(path_poses)}')
        lines.append('')

        # Robot position
        lines.append('── Robot Position ───────────')
        if rx is not None:
            lat, lon = self.node.converter.enu_to_lla(rx, ry)
            lines.append(f'  E: {rx:>+8.2f}m')
            lines.append(f'  N: {ry:>+8.2f}m')
            if lat is not None:
                lines.append(f'  Lat: {lat:.6f}')
                lines.append(f'  Lon: {lon:.6f}')
        else:
            lines.append('  (no robot pose)')
        lines.append('')

        # Laser scan stats
        lines.append('── Laser Scan ───────────────')
        if latest_scan is not None:
            sc = latest_scan
            valid_ranges = [r for r in sc.ranges
                            if math.isfinite(r) and sc.range_min <= r <= sc.range_max]
            close_ranges = [r for r in valid_ranges if r < 3.0]
            min_r = min(valid_ranges) if valid_ranges else float('nan')
            node_now = self.node.get_clock().now().to_msg()
            stamp    = sc.header.stamp
            age_s    = (node_now.sec - stamp.sec) + (node_now.nanosec - stamp.nanosec) * 1e-9
            lines.append(f'  Frame   : {sc.header.frame_id}')
            lines.append(f'  Age     : {age_s:>6.2f}s')
            lines.append(f'  Beams   : {len(sc.ranges)} total / {len(valid_ranges)} valid')
            lines.append(f'  Min rng : {min_r:>6.2f}m')
            lines.append(f'  <3m hits: {len(close_ranges)}')
        else:
            lines.append('  (no scan received)')
        lines.append('')

        # Event log
        lines.append('── Planner Events ───────────')
        if event_log:
            for ts, name, failed in reversed(event_log):
                marker = '!!' if failed else '  '
                lines.append(f' {marker}[{ts}] {name}')
        else:
            lines.append('  (none yet)')

        text = '\n'.join(lines)
        self.status_text.set_text(text)

        # Color the state line background dynamically via the text bbox
        self.status_text.set_bbox(dict(
            facecolor=color + '33',   # hex color + 20% alpha
            edgecolor=color,
            linewidth=0.8,
        ))


# ─── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = EMMNVisualizerNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    viz = VisualizerPlot(node)
    ani = FuncAnimation(viz.fig, viz.update, interval=100, blit=False, cache_frame_data=False)

    plt.show()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

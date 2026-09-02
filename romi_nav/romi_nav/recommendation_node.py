#!/usr/bin/env python3
"""
recommendation_node.py
----------------------
Human-in-the-loop recommendation harness for imitation-learning data
collection (ROMI Lab).

Instead of letting Nav2 auto-drive to a goal, this node breaks the run
into short "legs" (default 0.5 m). At each leg it presents TWO candidate
next moves:

    * Nav2's candidate  — a point ~0.5 m along the global path returned
      by planner_server's ComputePathToPose action (a pure query: the
      planner computes, nothing moves),
    * the agent's candidate — from AgentPolicy, currently a heuristic
      stub (# TODO: replace with trained IL/RL model).

A human picks 1 or 2 in the terminal. Only then is the CHOSEN leg sent
as a short path segment DIRECTLY to controller_server's FollowPath
action, so Nav2's DWB controller drives that one leg (with live
obstacle avoidance) and stops. We deliberately do NOT go through
bt_navigator's NavigateToPose for legs: its behavior tree adds
replanning and recovery spins we don't want inside a labeled leg, and
its Humble-era action-client layer has a race that instantly aborts
fast-completing micro-goals ("unknown goal response" +
"BtActionNode::Tick: invalid status value" in the logs).
Every decision is appended as one JSON line to
~/romi_ws/data/decisions_<timestamp>.jsonl.

Two safety layers wrap execution WITHOUT touching the data contract:
  * validate_path() gates every chosen leg against the global costmap
    (the agent's straight-line leg can cut through a wall; the robot
    must never be commanded through one) — a blocked leg is clipped or
    not driven, and logged once with that true outcome;
  * recover() frees a stuck/wedged robot with a scan+costmap-guided
    escape move, REVERSE-FIRST (the LDS is blind below 0.12 m, so an
    obstacle the robot is touching is invisible to the scan alone —
    the costmap still knows it is there; and backing out along the
    path the robot drove in by is the one motion that cannot scrape).
    Recovery is housekeeping: it runs only AFTER the stuck leg was
    logged, is NEVER itself logged as a decision, and the human then
    gets fresh candidates from the freed position.

Why Nav2 never auto-executes: bt_navigator only starts driving when it
receives a NavigateToPose goal (RViz's "2D Goal Pose" tool normally
triggers this via the /goal_pose topic). This node listens on its OWN
topic, /harness/goal — so you must retarget the RViz tool once:
    RViz: Panels -> Tool Properties -> 2D Goal Pose -> Topic: /harness/goal
After that, clicking a goal in RViz feeds only this node.

Run order (each in its own terminal, workspace sourced):
    1. ros2 launch romi_nav bringup.launch.py
    2. ros2 launch romi_nav nav.launch.py map:=$HOME/romi_ws/maps/romi_map_clean.yaml
    3. rviz2  (localize with "2D Pose Estimate" if AMCL isn't already)
    4. ros2 run romi_nav recommendation_node

Threading model (a common rclpy pattern worth understanding): ROS
callbacks (subscriptions, TF, action responses) need someone to "spin"
the node, but our human prompt blocks on input(). So a background
thread runs the executor (spin), and the MAIN thread runs the decision
loop, blocking freely on input() and polling action futures with short
sleeps — the background thread is what actually completes them.
"""

import json
import math
import os
import random
import threading
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


# OccupancyGrid rendering of costmap_2d costs: 100 = lethal obstacle,
# 99 = inscribed (robot CENTER here means the body overlaps the
# obstacle), -1 = unknown. NavFn refuses cells >= 99 too (>= 253 on its
# internal scale), so this is the same "untraversable" line the
# planner uses.
BLOCKED_COST = 99


# ---------------------------------------------------------------- helpers

def yaw_from_quat(q):
    """Extract heading (rotation about Z) from a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fill_quat_from_yaw(q, yaw):
    """Write a pure-Z-rotation quaternion for `yaw` into message `q`."""
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


def norm_angle(a):
    """Wrap any angle into (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def scan_clearance(scan, rel, half_window=0.15):
    """Min LiDAR range in a +-half_window rad cone around bearing `rel`
    RELATIVE to the robot (0 = straight ahead, pi = behind). The LDS is
    yaw-aligned with base_link, so a body-relative bearing is a beam
    angle directly."""
    if scan is None:
        return float('inf')     # no scan yet: assume clear
    rel = norm_angle(rel)
    n = len(scan.ranges)
    lo = int((rel - half_window - scan.angle_min) / scan.angle_increment)
    hi = int((rel + half_window - scan.angle_min) / scan.angle_increment)
    clearance = scan.range_max
    for i in range(lo, hi + 1):
        r = scan.ranges[i % n]     # LDS covers 360 deg: wrap indices
        if math.isfinite(r) and r > 0.0:
            clearance = min(clearance, r)
    return clearance


def downsample_scan(scan: LaserScan, n_beams: int):
    """Evenly sample the scan down to n_beams ranges for the log.
    inf/nan (no return within range) is recorded as range_max."""
    if scan is None:
        return None
    ranges = scan.ranges
    step = len(ranges) / float(n_beams)
    out = []
    for i in range(n_beams):
        r = ranges[int(i * step)]
        if not math.isfinite(r) or r <= 0.0:
            r = scan.range_max
        out.append(round(min(r, scan.range_max), 3))
    return out


# ---------------------------------------------------------------- agent

class AgentPolicy:
    """Placeholder policy producing the 'agent' candidate next move.

    # TODO: replace with trained IL/RL model.

    Contract (keep this stable so a trained model can drop in):
        propose(obs) -> {'x','y','yaw'}
    where obs = {'pose': {x,y,yaw}, 'final_goal': {x,y,yaw},
                 'scan': LaserScan or None}.
    A future visual policy would just add an 'image' key to obs —
    nothing in the harness loop needs to change.

    Stub heuristic: head roughly toward the final goal but perturb the
    heading by a random 20-45 degrees (either side), resampling up to
    10 times if the LiDAR shows an obstacle within the step distance in
    that direction. Falls back to the least-blocked direction it tried,
    with a shortened step. This makes the alternative *plausible*
    (not a wall-crash) while still clearly differing from Nav2.
    """

    def __init__(self, step=0.5, min_deg=20.0, max_deg=45.0):
        self.step = step
        self.min_rad = math.radians(min_deg)
        self.max_rad = math.radians(max_deg)

    def propose(self, obs):
        pose, goal, scan = obs['pose'], obs['final_goal'], obs['scan']
        goal_heading = math.atan2(goal['y'] - pose['y'],
                                  goal['x'] - pose['x'])

        best_theta, best_clearance = goal_heading, 0.0
        for _ in range(10):
            delta = random.uniform(self.min_rad, self.max_rad)
            delta *= random.choice((-1.0, 1.0))
            theta = norm_angle(goal_heading + delta)
            clearance = self._clearance(scan, pose['yaw'], theta)
            if clearance > self.step + 0.15:          # enough room: take it
                return self._step_pose(pose, theta, self.step)
            if clearance > best_clearance:
                best_theta, best_clearance = theta, clearance
        # Everything sampled was blocked-ish: take the least-blocked
        # direction but shorten the step to stay safely inside it.
        step = max(0.15, min(self.step, best_clearance - 0.15))
        return self._step_pose(pose, best_theta, step)

    @staticmethod
    def _step_pose(pose, theta, step):
        return {'x': round(pose['x'] + step * math.cos(theta), 3),
                'y': round(pose['y'] + step * math.sin(theta), 3),
                'yaw': round(theta, 3)}

    @staticmethod
    def _clearance(scan, robot_yaw, world_theta, half_window=0.15):
        """Min LiDAR range in a +-half_window rad cone around the
        world-frame direction world_theta (beam angle = world - robot
        yaw, see scan_clearance)."""
        return scan_clearance(scan, world_theta - robot_yaw, half_window)


# ---------------------------------------------------------------- node

class RecommendationHarness(Node):

    def __init__(self):
        super().__init__(
            'recommendation_harness',
            # Webots is the clock source; without this our timestamps
            # and TF lookups would use wall time and disagree with Nav2.
            parameter_overrides=[Parameter('use_sim_time',
                                           Parameter.Type.BOOL, True)])

        self.declare_parameter('horizon', 0.5)         # m per leg
        self.declare_parameter('goal_tolerance', 0.3)  # m, episode done
        self.declare_parameter('scan_beams', 72)       # logged obs size
        self.declare_parameter('leg_timeout', 60.0)    # s per leg
        self.horizon = self.get_parameter('horizon').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.scan_beams = self.get_parameter('scan_beams').value
        self.leg_timeout = self.get_parameter('leg_timeout').value

        # Latest data from callbacks (written by executor thread, read
        # by the main decision loop — simple attribute swaps are atomic
        # in Python, so no lock needed for these).
        self.latest_scan = None
        self.latest_costmap = None   # global costmap as OccupancyGrid
        self.latest_goal = None      # (seq, {'x','y','yaw'})
        self._goal_seq = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, '/harness/goal',
                                 self._goal_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb,
                                 qos_profile_sensor_data)
        # Costmap publishers are latched (transient local), so the
        # full grid arrives right away and after every update
        # (always_send_full_costmap is set in nav2_params.yaml).
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb,
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # planner_server: pure path query, never drives.
        self.plan_client = ActionClient(self, ComputePathToPose,
                                        'compute_path_to_pose')
        # controller_server: actually drives — only ever sent the exact
        # short path segment the human approved (bt_navigator bypassed).
        self.nav_client = ActionClient(self, FollowPath, 'follow_path')

        # Recovery-only plumbing. cmd_vel is published directly for the
        # un-logged recovery nudge: when the robot is wedged inside
        # lethal cells, every Nav2 action (FollowPath, the recovery
        # server's own BackUp/Spin) fails its collision precheck and
        # refuses to move at all — the scan-guarded nudge is the only
        # way out. The clear-costmap services drop contact-range
        # obstacle marks that would keep the planner boxed in after
        # the robot is free.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clear_global_cli = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_cli = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        self.agent = AgentPolicy(step=self.horizon)

        data_dir = os.path.expanduser('~/romi_ws/data')
        os.makedirs(data_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(data_dir, f'decisions_{stamp}.jsonl')
        self.log_file = open(self.log_path, 'a')

    # ------------------------------------------------------- callbacks

    def _goal_cb(self, msg: PoseStamped):
        self._goal_seq += 1
        self.latest_goal = (self._goal_seq,
                            {'x': msg.pose.position.x,
                             'y': msg.pose.position.y,
                             'yaw': yaw_from_quat(msg.pose.orientation)})
        self.get_logger().info(
            f"Goal #{self._goal_seq} received: "
            f"({self.latest_goal[1]['x']:.2f}, {self.latest_goal[1]['y']:.2f})")

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    # ------------------------------------------------------- utilities

    def _wait(self, future, timeout=None):
        """Block the MAIN thread until the executor thread (spinning in
        the background) completes this future. Returns None on timeout."""
        deadline = time.time() + timeout if timeout else None
        while not future.done():
            if deadline and time.time() > deadline:
                return None
            time.sleep(0.02)
        return future.result()

    def get_pose(self):
        """Robot pose in the map frame = AMCL's estimate, read from TF
        (map -> base_link). Returns None until AMCL has localized."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return {'x': round(t.transform.translation.x, 3),
                'y': round(t.transform.translation.y, 3),
                'yaw': round(yaw_from_quat(t.transform.rotation), 3)}

    def costmap_cost(self, x, y):
        """Cost of world point (x, y) in the latest global costmap:
        0..100 (see BLOCKED_COST), -1 for unknown, None if the point is
        off the costmap or no costmap has arrived yet."""
        grid = self.latest_costmap
        if grid is None:
            return None
        info = grid.info
        # floor, not int(): int() truncates toward zero, which would
        # map points up to one cell BELOW the origin onto index 0
        # instead of failing the bounds check.
        mx = math.floor((x - info.origin.position.x) / info.resolution)
        my = math.floor((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return grid.data[my * info.width + mx]

    @staticmethod
    def _pose_stamped(p):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = p['x']
        msg.pose.position.y = p['y']
        fill_quat_from_yaw(msg.pose.orientation, p['yaw'])
        return msg

    # ------------------------------------------------------- Nav2 query

    def nav2_candidate(self, pose, final_goal):
        """Ask planner_server for the global path to final_goal (pure
        computation, nothing moves) and reduce it to a micro-goal
        `horizon` metres along the path. Returns
        (candidate, path_len, path_segment) — the segment is the slice
        of the global path up to the micro-goal, ready to hand to
        FollowPath if the human picks Nav2 — or (None, reason, None)."""
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_stamped(final_goal)
        goal.use_start = False           # plan from current robot pose

        handle = self._wait(self.plan_client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return None, 'planner rejected the request', None
        result = self._wait(handle.get_result_async(), 10.0)
        if result is None or len(result.result.path.poses) < 2:
            return None, 'planner returned no path (goal unreachable?)', None

        poses = result.result.path.poses
        pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
        total, walked, idx = 0.0, 0.0, len(pts) - 1
        for i in range(1, len(pts)):
            d = math.dist(pts[i - 1], pts[i])
            total += d
            # First point at least `horizon` along the path = micro-goal.
            if walked < self.horizon <= walked + d:
                idx = i
            walked += d

        x, y = pts[idx]
        # NavFn doesn't produce meaningful orientations along the path,
        # so face the direction of travel (the path tangent).
        px, py = pts[max(idx - 1, 0)]
        yaw = math.atan2(y - py, x - px) if idx > 0 else pose['yaw']

        # The leg the controller would execute: the global path sliced
        # at the micro-goal, its last pose facing the path tangent (the
        # goal checker compares against this final pose).
        segment = result.result.path
        segment.poses = segment.poses[:idx + 1]
        fill_quat_from_yaw(segment.poses[-1].pose.orientation, yaw)

        return ({'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)},
                round(total, 3), segment)

    # ------------------------------------------------------- execution

    def straight_path(self, pose, candidate):
        """Synthesize a straight-line Path (5 cm pose spacing) from the
        robot to the agent's candidate. DWB needs a global path to
        track; it still does local obstacle avoidance around it."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        dist = math.dist((pose['x'], pose['y']),
                         (candidate['x'], candidate['y']))
        n = max(2, int(dist / 0.05) + 1)
        for i in range(n):
            f = i / (n - 1)
            path.poses.append(self._pose_stamped(
                {'x': pose['x'] + f * (candidate['x'] - pose['x']),
                 'y': pose['y'] + f * (candidate['y'] - pose['y']),
                 'yaw': candidate['yaw']}))
        return path

    def validate_path(self, path):
        """Gate EVERY chosen leg against the global costmap before it
        is handed to FollowPath, no matter which candidate produced it:
        the planner's slices normally avoid obstacles, but the agent's
        leg is a synthesized straight line that can cut through a wall.

        Returns (safe_path, note):
          (path, None)        — every pose traversable, drive as-is;
          (clipped, why)      — path crosses a blocked cell; clipped to
                                end ~0.15 m before it (last pose faces
                                the tangent for the goal checker);
          (None, why)         — blocked so close that no usable prefix
                                remains; must not be driven.
        """
        if self.latest_costmap is None:
            self.get_logger().warn(
                'no global costmap received yet - leg not validated')
            return path, None

        blocked, why, along = None, '', 0.0
        prev = None
        for i, ps in enumerate(path.poses):
            pt = (ps.pose.position.x, ps.pose.position.y)
            if prev is not None:
                along += math.dist(prev, pt)
            prev = pt
            c = self.costmap_cost(*pt)
            if c is None or c < 0 or c >= BLOCKED_COST:
                blocked = i
                why = ('leaves the mapped area' if c is None else
                       'enters unmapped space' if c < 0 else
                       'passes through a wall/furniture (or hugs it '
                       'closer than the robot fits)')
                break
        if blocked is None:
            return path, None

        # Walk back ~0.15 m from the blocked pose as stopping margin.
        j, back = blocked, 0.0
        while j > 0 and back < 0.15:
            a = path.poses[j - 1].pose.position
            b = path.poses[j].pose.position
            back += math.hypot(b.x - a.x, b.y - a.y)
            j -= 1
        kept = path.poses[:j + 1]
        length = sum(
            math.dist((kept[k - 1].pose.position.x,
                       kept[k - 1].pose.position.y),
                      (kept[k].pose.position.x, kept[k].pose.position.y))
            for k in range(1, len(kept)))
        if len(kept) < 2 or length < 0.15:
            return None, f'{why} only {along:.2f} m ahead'

        clipped = Path()
        clipped.header = path.header
        clipped.poses = kept
        a, b = kept[-2].pose.position, kept[-1].pose.position
        fill_quat_from_yaw(kept[-1].pose.orientation,
                           math.atan2(b.y - a.y, b.x - a.x))
        return clipped, f'{why} {along:.2f} m ahead - clipped to {length:.2f} m'

    def explain_planning_failure(self, pose, goal):
        """Turn a bare 'no path' into a reason the labeler can act on,
        by looking the start and goal up in the global costmap.
        Returns (message, start_wedged) — start_wedged means the
        robot's OWN cell is untraversable (usually after getting stuck
        against furniture), so every goal in every direction will fail
        until the robot moves: the caller should recover, not abort."""
        if self.latest_costmap is None:
            return 'no costmap available to diagnose why', False
        gc = self.costmap_cost(goal['x'], goal['y'])
        sc = self.costmap_cost(pose['x'], pose['y'])
        if sc is not None and sc >= BLOCKED_COST:
            return ("robot is wedged against an obstacle (its own cell "
                    "is in the costmap's lethal zone, so no start is "
                    "valid and ALL goals fail)"), True
        if gc is None:
            return 'goal is outside the mapped area', False
        if gc < 0:
            return 'goal is in unmapped space', False
        if gc >= BLOCKED_COST:
            return 'goal is inside or too close to a wall/furniture', False
        return ('no route found from here (a passage between robot and '
                'goal may be blocked)'), False

    # ------------------------------------------------------- recovery

    def _guarded_move(self, vx, wz, duration_s, watch_rel=None,
                      min_range=0.13, travel=None):
        """Publish cmd_vel for up to duration_s of SIM time, stopping
        early if any of:
          * the scan cone at body-relative bearing watch_rel drops
            below min_range (robot radius ~0.105 m -> 0.13 leaves a
            small physical margin);
          * net displacement reaches `travel` metres (bounded nudge);
          * translation is commanded but the robot is not actually
            moving — wheels grinding against an obstacle the lidar
            cannot see (anything inside its 0.12 m minimum range).
        Recovery-only helper."""
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = wz
        p0 = self.get_pose()
        best_disp, last_progress = 0.0, 0.0
        start = self.get_clock().now()
        while True:
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed >= duration_s:
                break
            if (watch_rel is not None and
                    scan_clearance(self.latest_scan, watch_rel, 0.4)
                    < min_range):
                break
            if p0 is None:              # TF was late at move start —
                p0 = self.get_pose()    # arm the guards when it appears
            p1 = self.get_pose()
            if p0 is not None and p1 is not None:
                disp = math.dist((p0['x'], p0['y']), (p1['x'], p1['y']))
                if travel is not None and disp >= travel:
                    break
                # Windowed progress check, not stall-from-start: a
                # robot that moves a little and THEN wedges (contact
                # inside the lidar blind zone) must also stop.
                if disp > best_disp + 0.01:
                    best_disp = disp
                    last_progress = elapsed
                if vx != 0.0 and elapsed - last_progress > 1.5:
                    print('  guarded move: commanded but not moving - '
                          'stopping (grinding against an unseen '
                          'obstacle?)')
                    break
            self.cmd_pub.publish(twist)
            time.sleep(0.02)
        self.cmd_pub.publish(Twist())      # full stop

    def clear_costmaps(self):
        """Ask Nav2 to rebuild both costmaps from fresh sensor data —
        contact-range obstacle marks otherwise linger where the robot
        was wedged and keep the planner rejecting every goal."""
        for cli in (self.clear_global_cli, self.clear_local_cli):
            if cli.service_is_ready():
                self._wait(cli.call_async(ClearEntireCostmap.Request()),
                           2.0)

    def _await_fresh_costmap(self, after, timeout=6.0):
        """Block until a global costmap STAMPED (sim time) at or after
        `after` arrives — publishes are 1 Hz — so post-recovery
        planning and validate_path don't run against the pre-recovery
        grid. An "any newer message" identity check is NOT enough: a
        grid published just before the clear can be delivered just
        after it and would pass. Returns False on timeout (slow sim);
        callers should warn, since stale marks may then linger."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            grid = self.latest_costmap
            if grid is not None:
                stamp = rclpy.time.Time.from_msg(grid.header.stamp)
                if stamp.nanoseconds >= after.nanoseconds:
                    return True
            time.sleep(0.1)
        return False

    def _probe_costmap(self, pose, rel, max_dist=0.6, step=0.025,
                       start=0.12):
        """Distance from the robot CENTER to the first untraversable
        global-costmap cell along body-relative bearing `rel`, probing
        outward from just past the robot's own footprint (whose cells
        are often legitimately >= BLOCKED_COST while wedged). The
        costmap's static/obstacle layers still see walls inside the
        lidar's 0.12 m blind zone, so this is the recovery's defense
        against invisible contact. Unknown and off-map cells count as
        blocked (never back into space we know nothing about).
        Returns +inf if nothing blocks within max_dist (or no costmap
        yet)."""
        if self.latest_costmap is None:
            return float('inf')
        theta = pose['yaw'] + rel
        d = start
        while d <= max_dist:
            c = self.costmap_cost(pose['x'] + d * math.cos(theta),
                                  pose['y'] + d * math.sin(theta))
            if c is None or c < 0 or c >= BLOCKED_COST:
                return d
            d += step
        return float('inf')

    def _recovery_clearance(self, pose, rel):
        """Fused clearance for recovery decisions. The scan ALONE lies
        exactly when it matters: the LDS returns +inf for anything
        closer than its 0.12 m minimum range, and the robot's shell is
        only ~0.07 m from the lidar axis — an obstacle the robot is
        touching reads as 'clear'. So take the MIN of the scan cone
        and a costmap probe in the same direction."""
        return min(scan_clearance(self.latest_scan, rel, 0.4),
                   self._probe_costmap(pose, rel))

    def _pick_recovery_move(self, pose):
        """Decide the escape move from fused clearances. Pure decision,
        separated from execution so it can be replayed offline against
        logged wedge scans. Returns (move, clearances) with move one of
        'reverse'|'left'|'right'|'forward'|None.

        REVERSE-FIRST policy: legs only ever drive forward (DWB's
        min_vel_x is 0.0, it cannot even command reverse), so a stuck
        robot is nosed into something on its front arc, and backing
        out along the swept volume it entered by is the one motion
        that cannot scrape. In-place rotation — what a max-clearance
        rule would usually pick here — is exactly the motion contact
        prevents, so sides are only tried when the rear is blocked,
        and forward-creep is last resort."""
        c = {'rear': self._recovery_clearance(pose, math.pi),
             'front': self._recovery_clearance(pose, 0.0),
             'left': self._recovery_clearance(pose, math.pi / 2),
             'right': self._recovery_clearance(pose, -math.pi / 2)}
        # Thresholds are on the FUSED metric. Note the two fused
        # sources measure differently: the scan gives center-to-
        # surface, the costmap probe gives center-to-first-blocked-
        # cell, which sits ~0.10 m (inscribed ring) SHY of the
        # surface — so probe-dominated readings are already
        # conservative. Reverse gets a lower bar because its travel
        # adapts to the measured pocket (see recover()); sides/front
        # creep a fixed 0.2 m plus ~0.105 m body overhang.
        if c['rear'] >= 0.30:
            return 'reverse', c
        if max(c['left'], c['right']) >= 0.35:
            return ('left' if c['left'] >= c['right'] else 'right'), c
        if c['front'] >= 0.35:
            return 'forward', c
        return None, c

    def recover(self):
        """Free a stuck/wedged robot with a fused-clearance escape
        move (see _pick_recovery_move), then rebuild the costmaps.
        Returns True if the robot actually moved.

        HOUSEKEEPING ONLY — this is deliberately NOT a decision and is
        never written to the JSONL (the caller has already logged the
        stuck leg once, with its true outcome). Logging recovery motion
        as if a human chose it would contaminate the imitation data —
        the same reason legs bypass bt_navigator's recovery behaviors.
        """
        pose0 = self.get_pose()
        if self.latest_scan is None or pose0 is None:
            print('  cannot recover automatically: no scan/pose yet')
            return False

        move, c = self._pick_recovery_move(pose0)
        summary = (f"rear {c['rear']:.2f}, front {c['front']:.2f}, "
                   f"left {c['left']:.2f}, right {c['right']:.2f} m")
        if move is None:
            print(f'  recovery aborted: no direction is open ({summary})'
                  ' - please teleop the robot clear')
            return False
        print(f'  recovering (housekeeping, NOT logged): {move} '
              f'({summary})')

        if move == 'reverse':
            # Back out along the entry path at 6 cm/s. Travel adapts
            # to the measured pocket: fused rear clearance minus the
            # 0.105 m rear body overhang minus margin, capped at
            # 0.25 m. Even the 0.12 m floor pulls the robot's cell
            # out of the inscribed zone (~0.08 m does it).
            self._guarded_move(-0.06, 0.0, 6.0, watch_rel=math.pi,
                               travel=max(0.12, min(0.25,
                                                    c['rear'] - 0.13)))
        elif move == 'forward':
            self._guarded_move(0.05, 0.0, 5.0, watch_rel=0.0,
                               travel=0.2)
        else:
            rel = math.pi / 2 if move == 'left' else -math.pi / 2
            self._guarded_move(0.0, 0.5 if rel > 0 else -0.5,
                               abs(rel) / 0.5)
            self._guarded_move(0.05, 0.0, 5.0, watch_rel=0.0,
                               travel=0.2)

        # Rebuild costmaps from the freed pose and WAIT for a grid
        # stamped after the clear (1 Hz publisher) — otherwise the
        # next plan/validation still sees the wedge-era marks.
        self.clear_costmaps()
        cleared_at = self.get_clock().now()
        if not self._await_fresh_costmap(cleared_at):
            print('  warning: no fresh costmap arrived after the '
                  'clear - the next validation may still see stale '
                  'obstacle marks')
        pose1 = self.get_pose() or pose0
        moved = math.dist((pose0['x'], pose0['y']),
                          (pose1['x'], pose1['y']))
        print(f'  recovery moved the robot {moved:.2f} m; '
              'fresh candidates from the new position')
        return moved > 0.03

    def drive_leg(self, path):
        """Send the HUMAN-CHOSEN path segment straight to
        controller_server's FollowPath action. This is the only place
        the robot is ever commanded to move. Going direct (instead of
        via bt_navigator's NavigateToPose) means: no behavior tree, no
        recovery spins inside a labeled leg, no Humble BT action-client
        race — and 'reached' is the controller's own verdict on this
        exact segment (goal checker: xy 0.10 m / yaw 0.5 rad)."""
        goal = FollowPath.Goal()
        goal.path = path

        t0 = time.time()
        handle = self._wait(self.nav_client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return {'reached': False, 'duration_s': 0.0,
                    'note': 'follow_path rejected'}

        result = self._wait(handle.get_result_async(), self.leg_timeout)
        if result is None:                      # stuck: cancel and move on
            self._wait(handle.cancel_goal_async(), 5.0)
            return {'reached': False,
                    'duration_s': round(time.time() - t0, 2),
                    'note': 'leg timeout, cancelled'}
        return {'reached': result.status == GoalStatus.STATUS_SUCCEEDED,
                'duration_s': round(time.time() - t0, 2)}

    # ------------------------------------------------------- logging

    def log_decision(self, row):
        self.log_file.write(json.dumps(row) + '\n')
        self.log_file.flush()       # survive Ctrl+C mid-episode


# ---------------------------------------------------------------- loop

def describe_move(rel, dist):
    """Plain-language description of a candidate move for the human
    labeler, RELATIVE TO THE ROBOT'S CURRENT HEADING. `rel` is the
    bearing to the candidate minus the robot's yaw, wrapped to
    (-pi, pi]; positive = counterclockwise = a LEFT turn (REP-103).
    Only the on-screen wording is bucketed — the log always keeps the
    exact x/y/yaw floats."""
    deg = math.degrees(rel)
    side = 'left' if deg > 0 else 'right'
    a = abs(deg)
    if a <= 15:
        word = 'straight ahead'
    elif a <= 45:
        word = f'slight {side}'
    elif a <= 90:
        word = side
    elif a <= 135:
        word = f'hard {side}'
    else:
        word = f'behind you, to the {side}'
    return f'{word}, ~{round(dist / 0.05) * 0.05:g} m'


def ask_choice():
    """Blocking terminal prompt. Returns '1', '2' or 'q'."""
    while True:
        ans = input('Choose 1/2 (q aborts this goal): ').strip().lower()
        if ans in ('1', '2', 'q'):
            return ans
        print("  please type 1, 2 or q")


def decision_loop(node: RecommendationHarness):
    print(f'\nDecision log: {node.log_path}')
    print('Waiting for Nav2 action servers...')
    node.plan_client.wait_for_server()
    node.nav_client.wait_for_server()
    print('Nav2 is up. In RViz set the "2D Goal Pose" tool topic to '
          '/harness/goal\n(Panels -> Tool Properties), then click a goal.\n')

    done_seq = 0            # goal sequence numbers already finished
    while rclpy.ok():
        # ---- wait for a fresh goal click ----
        if node.latest_goal is None or node.latest_goal[0] <= done_seq:
            time.sleep(0.2)
            continue
        goal_seq, final_goal = node.latest_goal
        print(f"\n=== New goal #{goal_seq}: "
              f"({final_goal['x']:.2f}, {final_goal['y']:.2f}) ===")

        step = 0
        wedge_recoveries = 0    # un-logged recoveries: planning failed
        veto_recoveries = 0     # un-logged recoveries: legs vetoed
        while rclpy.ok():
            # A newer RViz click mid-episode redirects the episode.
            if node.latest_goal[0] > goal_seq:
                goal_seq, final_goal = node.latest_goal
                print(f"--- redirected to goal #{goal_seq}: "
                      f"({final_goal['x']:.2f}, {final_goal['y']:.2f}) ---")

            pose = node.get_pose()
            if pose is None:
                print('No map->base_link TF yet — localize AMCL in RViz '
                      '("2D Pose Estimate"), retrying...')
                time.sleep(1.0)
                continue

            dist = math.dist((pose['x'], pose['y']),
                             (final_goal['x'], final_goal['y']))
            if dist <= node.goal_tolerance:
                print(f'=== Goal reached ({dist:.2f} m away) '
                      f'after {step} decisions ===\n')
                break

            # ---- the two candidates ----
            nav2_cand, path_len, nav2_path = node.nav2_candidate(
                pose, final_goal)
            if nav2_cand is None:
                reason, start_wedged = node.explain_planning_failure(
                    pose, final_goal)
                print(f'Nav2 planning failed: {path_len}\n'
                      f'  why: {reason}')
                # A wedged START fails every goal — recover (un-logged
                # housekeeping) and re-plan instead of aborting.
                if start_wedged and wedge_recoveries < 2:
                    wedge_recoveries += 1
                    if node.recover():
                        continue
                print('Aborting this goal.\n')
                break
            wedge_recoveries = 0    # planning works again: renew allowance
            scan = node.latest_scan
            agent_cand = node.agent.propose(
                {'pose': pose, 'final_goal': final_goal, 'scan': scan})

            # ---- prompt (source order randomized: position bias would
            # poison the labels; the log records the true order) ----
            slots = [('nav2', nav2_cand), ('agent', agent_cand)]
            random.shuffle(slots)
            print(f"\n[step {step}] robot at ({pose['x']:.2f}, "
                  f"{pose['y']:.2f}), {dist:.2f} m to goal "
                  f"(path {path_len} m)")
            for i, (_, c) in enumerate(slots, 1):
                # Bearing relative to the robot's CURRENT heading, so
                # "left"/"right" match what the labeler sees on screen.
                rel = norm_angle(
                    math.atan2(c['y'] - pose['y'], c['x'] - pose['x'])
                    - pose['yaw'])
                d = math.dist((pose['x'], pose['y']), (c['x'], c['y']))
                print(f"  [{i}] {describe_move(rel, d)}")

            ans = ask_choice()
            if ans == 'q':
                print('Aborted by user. Waiting for a new goal.\n')
                break
            chosen_src, chosen_cand = slots[int(ans) - 1]
            print(f'  -> executing [{ans}] ({chosen_src})')

            # Nav2's leg is the planner's own path slice; the agent's
            # leg is a synthesized straight line to its candidate.
            leg_path = (nav2_path if chosen_src == 'nav2'
                        else node.straight_path(pose, chosen_cand))

            # Safety gate: never drive a leg through a blocked cell,
            # whichever candidate produced it. A rejected/clipped leg
            # is still the human's decision — it is logged once, with
            # its true outcome.
            leg_path, safety_note = node.validate_path(leg_path)
            if leg_path is None:
                print(f'  NOT driving this leg: path {safety_note}')
                outcome = {'reached': False, 'duration_s': 0.0,
                           'note': f'not executed: path {safety_note}'}
            else:
                veto_recoveries = 0    # a leg passed validation again
                if safety_note:
                    print(f'  caution: path {safety_note}')
                outcome = node.drive_leg(leg_path)
                if safety_note:
                    outcome['note'] = '; '.join(
                        filter(None, [outcome.get('note'), safety_note]))

            # Stuck = commanded to move but went nowhere until the
            # controller's progress checker (10 s) / leg timeout gave
            # up. Detect it BEFORE logging so the row carries the true
            # reason, then recover AFTER logging (never as a decision).
            end_pose = node.get_pose() or pose
            moved = math.dist((pose['x'], pose['y']),
                              (end_pose['x'], end_pose['y']))
            stuck = (leg_path is not None and not outcome['reached']
                     and outcome['duration_s'] >= 8.0 and moved < 0.2)
            if stuck:
                outcome['note'] = '; '.join(
                    filter(None, [outcome.get('note'), 'stuck']))
                print(f"  leg stalled: no progress in "
                      f"{outcome['duration_s']:.0f} s (moved "
                      f"{moved:.2f} m) - robot is likely pressing "
                      "against an obstacle; will recover after logging")
            elif not outcome['reached']:
                print(f"  leg did not complete: {outcome}")

            node.log_decision({
                't_wall': round(time.time(), 3),
                't_sim': round(node.get_clock().now().nanoseconds * 1e-9, 3),
                'step': step,
                'goal_seq': goal_seq,
                'final_goal': final_goal,
                'pose': pose,
                'scan': downsample_scan(scan, node.scan_beams),
                'nav2_candidate': {**nav2_cand, 'path_length': path_len},
                'agent_candidate': agent_cand,
                'chosen': chosen_src,
                'display_order': [s for s, _ in slots],
                'outcome': outcome,
            })
            step += 1

            # Recovery runs strictly AFTER the leg was logged once
            # with its true outcome, and is itself never logged: the
            # next loop iteration presents fresh candidates from the
            # freed position.
            if stuck:
                node.recover()
            elif leg_path is None:
                # Leg vetoed without driving (duration 0.0, so the
                # `stuck` detector can never fire). If the veto is
                # because the robot's OWN cell is untraversable — it
                # is wedged — then every future candidate will fail
                # validation too, while the planner may keep
                # "succeeding": without recovery here the loop
                # freezes (observed 2026-07-15: 12 frozen rows
                # across three goals at one pose). Checked against
                # the LIVE pose, not the iteration-start `pose` —
                # the human may have teleoped the robot clear while
                # the prompt was blocking. Budgeted like the
                # planning-failure recoveries so a wedge recovery
                # cannot fix doesn't reverse-ratchet forever.
                cur = node.get_pose() or pose
                sc = node.costmap_cost(cur['x'], cur['y'])
                if sc is not None and sc >= BLOCKED_COST:
                    if veto_recoveries < 2:
                        veto_recoveries += 1
                        print("  the robot's own cell is blocked in "
                              'the costmap - wedged; recovering '
                              f'(attempt {veto_recoveries}/2)')
                        node.recover()
                    else:
                        print('  still wedged after 2 recovery '
                              'attempts - please teleop the robot '
                              'clear, then choose again or set a '
                              'new goal')
        done_seq = goal_seq


def main():
    rclpy.init()
    node = RecommendationHarness()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        decision_loop(node)
    except (KeyboardInterrupt, EOFError):
        print('\nShutting down.')
    finally:
        node.log_file.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

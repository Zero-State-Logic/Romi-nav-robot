#!/usr/bin/env python3
import json, math, time, os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose

STEP_TIME = 0.6
STEP_SPEED = 0.12
STEP_DIST = STEP_TIME * STEP_SPEED

def yaw_from_quat(z, w): return math.atan2(2.0*w*z, 1.0-2.0*z*z)
def wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a
def bearing_words(rel):
    d = math.degrees(rel)
    if abs(d) <= 15: return "straight ahead"
    if 15 < d <= 45: return "slight left"
    if 45 < d <= 135: return "left"
    if d > 135: return "behind (left)"
    if -45 <= d < -15: return "slight right"
    if -135 <= d < -45: return "right"
    return "behind (right)"

class Rec(Node):
    def __init__(self):
        super().__init__('recommendation_node')
        self.pose = None
        self.goal = None
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        d = os.path.expanduser('~/romi_ws/data')
        os.makedirs(d, exist_ok=True)
        self.log_path = os.path.join(d, 'decisions_%s.jsonl' % time.strftime('%Y%m%d_%H%M%S'))
        self.log_f = open(self.log_path, 'a')
        self.step = 0
        self.get_logger().info('recommendation_node started. Set goal in RViz (2D Goal Pose, frame=odom).')

    def odom_cb(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation.z, p.orientation.w))

    def goal_cb(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y)
        self.get_logger().info('Goal: (%.2f, %.2f)' % self.goal)

    def nav2_suggestion(self):
        if not self.planner.wait_for_server(timeout_sec=3.0):
            return None, 'planner not available'
        g = ComputePathToPose.Goal()
        gp = PoseStamped()
        gp.header.frame_id = 'odom'
        gp.pose.position.x = self.goal[0]
        gp.pose.position.y = self.goal[1]
        gp.pose.orientation.w = 1.0
        g.goal = gp
        g.use_start = False
        g.planner_id = 'GridBased'
        fut = self.planner.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, 'rejected'
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=5.0)
        res = rf.result()
        if res is None or len(res.result.path.poses) < 2:
            return None, 'no path'
        px, py, pth = self.pose
        for ps in res.result.path.poses:
            x = ps.pose.position.x; y = ps.pose.position.y
            if math.hypot(x-px, y-py) >= STEP_DIST:
                return (x, y), None
        last = res.result.path.poses[-1].pose.position
        return (last.x, last.y), None

    def agent_suggestion(self):
        px, py, pth = self.pose
        gx, gy = self.goal
        base = math.atan2(gy-py, gx-px)
        ang = base + math.radians(30)
        return (px + STEP_DIST*math.cos(ang), py + STEP_DIST*math.sin(ang))

    def drive_step(self, target):
        px, py, pth = self.pose
        rel = wrap(math.atan2(target[1]-py, target[0]-px) - pth)
        t = Twist()
        t.linear.x = STEP_SPEED
        t.angular.z = max(-0.6, min(0.6, rel*1.5))
        end = time.time() + STEP_TIME
        while time.time() < end:
            self.cmd_pub.publish(t)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

    def log(self, chosen, nav2, agent, note=''):
        row = {'t': round(time.time(),3), 'step': self.step, 'pose': self.pose,
               'goal': self.goal, 'nav2_suggestion': nav2, 'agent_suggestion': agent,
               'chosen': chosen, 'note': note}
        self.log_f.write(json.dumps(row) + '\n')
        self.log_f.flush()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is None or self.goal is None:
                continue
            dist = math.hypot(self.goal[0]-self.pose[0], self.goal[1]-self.pose[1])
            if dist < 0.15:
                print('\n*** GOAL REACHED ***\n'); self.goal = None; continue
            nav2, err = self.nav2_suggestion()
            agent = self.agent_suggestion()
            self.step += 1
            print('\n--- step %d --- (%.2f m to goal)' % (self.step, dist))
            if nav2:
                px, py, pth = self.pose
                rel = wrap(math.atan2(nav2[1]-py, nav2[0]-px) - pth)
                print('  [1] Nav2 : %s  (%.2f, %.2f)' % (bearing_words(rel), nav2[0], nav2[1]))
            else:
                print('  [1] Nav2 : unavailable (%s)' % err)
            px, py, pth = self.pose
            rel = wrap(math.atan2(agent[1]-py, agent[0]-px) - pth)
            print('  [2] Agent: %s  (%.2f, %.2f)' % (bearing_words(rel), agent[0], agent[1]))
            ans = input('1=Nav2 drive, 2=agent(you joystick), s=skip, q=quit: ').strip()
            if ans == 'q': break
            elif ans == '1' and nav2:
                self.log('nav2', nav2, agent); self.drive_step(nav2)
            elif ans == '2':
                self.log('agent', nav2, agent, 'human joystick'); print('  -> drive with joystick, Enter when done'); input()
            elif ans == 's':
                self.log('skip', nav2, agent)
            else:
                print('  (invalid)')
        self.cmd_pub.publish(Twist()); self.log_f.close()

def main():
    rclpy.init(); n = Rec()
    try: n.run()
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

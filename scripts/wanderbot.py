#!/usr/bin/env python3
"""
WanderBot for TurtleBot3 (ROS Noetic, Python3)
- Subscribes to /scan (sensor_msgs/LaserScan)
- Publishes to /cmd_vel (geometry_msgs/Twist)
- Simple states: MOVE_FORWARD, ROTATE
- If obstacle closer than min_obstacle_dist in front sector -> rotate (random direction)
- Otherwise move forward with small random angular jitter
"""

import rospy
import random
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class WanderBot:
    def __init__(self):
        rospy.init_node('wanderbot', anonymous=False)

        # Parameters (tune these)
        self.forward_speed = rospy.get_param('~forward_speed', 0.18)      # m/s
        self.rotate_speed  = rospy.get_param('~rotate_speed', 0.6)       # rad/s during escape turn
        self.min_obstacle_dist = rospy.get_param('~min_obstacle_dist', 0.45)  # m - closer than this triggers avoidance
        self.front_angle_deg = rospy.get_param('~front_angle_deg', 30)   # consider +- this angle as "front"
        self.jitter_angle_rad = rospy.get_param('~jitter_angle_rad', 0.15)   # small jitter while moving
        self.rotate_time_min = rospy.get_param('~rotate_time_min', 0.6)  # seconds
        self.rotate_time_max = rospy.get_param('~rotate_time_max', 1.4)  # seconds
        self.rate_hz = rospy.get_param('~rate_hz', 10)

        # State
        self.state = 'MOVE_FORWARD'
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.last_scan = None
        self.rotation_end_time = None
        self.rotation_direction = 1  # 1 = left, -1 = right

        rospy.loginfo("WanderBot started (forward_speed=%.2f m/s, min_obstacle_dist=%.2f m)",
                      self.forward_speed, self.min_obstacle_dist)

        self.r = rospy.Rate(self.rate_hz)
        self.spin_loop()

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def front_obstacle_distance(self):
        """Return min distance in the front sector. If no scan available return +inf."""
        if self.last_scan is None:
            return float('inf')

        scan = self.last_scan
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        # front sector bounds in radians
        half = math.radians(self.front_angle_deg)
        start_angle = -half
        end_angle = half

        # convert angles to indices
        start_i = int(round((start_angle - angle_min) / angle_increment))
        end_i = int(round((end_angle - angle_min) / angle_increment))
        start_i = max(0, start_i)
        end_i = min(len(ranges)-1, end_i)

        # find minimum valid range in sector (ignore NaN/inf/0)
        min_dist = float('inf')
        for r in ranges[start_i:end_i+1]:
            if r == 0.0:
                continue
            if r is None:
                continue
            if math.isfinite(r) and r > 0.0:
                if r < min_dist:
                    min_dist = r
        return min_dist

    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        t = Twist()
        t.linear.x = linear_x
        t.angular.z = angular_z
        self.cmd_pub.publish(t)

    def start_rotation(self):
        # pick rotation direction randomly and rotation duration
        self.rotation_direction = random.choice([-1, 1])
        dur = random.uniform(self.rotate_time_min, self.rotate_time_max)
        self.rotation_end_time = time.time() + dur
        self.state = 'ROTATE'
        rospy.logdebug("Starting rotate dir=%d for %.2fs", self.rotation_direction, dur)

    def spin_loop(self):
        while not rospy.is_shutdown():
            try:
                min_front = self.front_obstacle_distance()

                if self.state == 'MOVE_FORWARD':
                    # obstacle detected? switch to rotate
                    if min_front < self.min_obstacle_dist:
                        rospy.loginfo_throttle(1, "Obstacle in front: %.2fm -> rotating", min_front)
                        self.start_rotation()
                        # immediately publish a rotating command for responsiveness
                        self.publish_twist(0.0, self.rotation_direction * self.rotate_speed)
                    else:
                        # normal forward with slight random jitter (makes motion less dead-straight)
                        jitter = random.uniform(-self.jitter_angle_rad, self.jitter_angle_rad)
                        self.publish_twist(self.forward_speed, jitter)
                elif self.state == 'ROTATE':
                    # keep rotating until rotation_end_time
                    if self.rotation_end_time and time.time() < self.rotation_end_time:
                        self.publish_twist(0.0, self.rotation_direction * self.rotate_speed)
                    else:
                        # rotation finished; go forward again
                        self.state = 'MOVE_FORWARD'
                        self.rotation_end_time = None
                        rospy.logdebug("Rotation finished, moving forward")
                        self.publish_twist(self.forward_speed, 0.0)

                self.r.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr("WanderBot exception: %s", e)
                # safety stop
                self.publish_twist(0.0, 0.0)
                rospy.sleep(0.5)

        # On shutdown stop the robot
        self.publish_twist(0.0, 0.0)
        rospy.loginfo("WanderBot node shutdown - robot stopped.")


if __name__ == '__main__':
    try:
        WanderBot()
    except rospy.ROSInterruptException:
        pass

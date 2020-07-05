#!/usr/bin/env python
import rospy
import numpy as np
import csv
from utils import get_actuation, nearest_point_on_trajectory_py2, first_point_on_trajectory_intersecting_circle
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import tf
import rospkg

class PPA(object):
    def __init__(self, csv_path):
        self.lad = 1.0
        self.max_reacquire = 10
        with open(csv_path) as f:
            wpts = [tuple(line) for line in csv.reader(f)]
            self.waypoints = np.array([(float(pt[0]), float(pt[1]), float(pt[2]), float(pt[3]), float(pt[4]), float(pt[5])) for pt in wpts])

        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.plan, queue_size=10)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

    def _get_current_waypoint(self, waypoints, lad, position, theta):
        wpts = waypoints[:, 0:2]
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory_py2(position, wpts)
        if nearest_dist < lad:
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(position, lad, wpts, i+t, wrap=True)
            if i2 == None:
                return None
            current_waypoint = np.empty(waypoints[i2, :].shape)
            # x, y
            current_waypoint[0:2] = waypoints[i2, 0:2]
            # theta
            current_waypoint[3] = waypoints[i2, 3]
            # speed
            current_waypoint[2] = waypoints[i2, 2]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return waypoints[i, :]
        else:
            return None

    def plan(self, odom_msg):

        q = np.array([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
        _, _, pose_theta = tf.transformations.euler_from_quaternion(q)
        pose_x = odom_msg.pose.pose.position.x
        pose_y = odom_msg.pose.pose.position.y

        position = np.array([pose_x, pose_y])
        lookahead_point = self._get_current_waypoint(self.waypoints, self.lad, position, pose_theta)
        if lookahead_point is None:
            return self.safe_speed, 0.0
        speed, steering_angle = get_actuation(pose_theta, lookahead_point, position, self.lad, 0.25)
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    rospy.init_node('pp')
    rospack = rospkg.RosPack()
    path = rospack.get_path('benchmark1')
    ppa = PPA(path + '/src/berlin.csv')
    rospy.spin()
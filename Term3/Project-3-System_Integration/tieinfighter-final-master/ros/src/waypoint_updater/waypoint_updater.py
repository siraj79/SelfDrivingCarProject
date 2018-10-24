#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
from scipy.spatial import KDTree
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
DECEL_RATE = 0.5
ACCEL_RATE = 0.5
debug = False

class DriveState():
    Drive = 1
    Stop = 2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_index = None
        self.vehicle_velocity = None

        self.drive_state = DriveState.Drive
        self.previous_velocity = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and \
               self.base_waypoints and \
               self.waypoint_tree and \
               self.vehicle_velocity:

                # get the closest waypoint
                #closest_waypoint_index = self.get_closest_waypoint_index()
                self.publish_waypoints()
            self.previous_velocity = self.vehicle_velocity
            rate.sleep()

    def get_closest_waypoint_index(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # figure out if closest coordinate is infront or behind vehicle
        closest_coord_np = np.array(closest_coord)
        previous_coord_np = np.array(prev_coord)
        position_np = np.array([x, y])

        val = np.dot(closest_coord_np - previous_coord_np,
                     position_np - closest_coord_np)

        # ensure closest index is for the closest waypoint ahead of the car
        if (val > 0):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
        # generate waypoints
        start_idx = self.get_closest_waypoint_index()
        if start_idx is not None:
            lane = Lane()
            end_index = start_idx + LOOKAHEAD_WPS
            lane_waypoints = self.base_waypoints.waypoints[start_idx:end_index]
            if self.stopline_wp_index != None and self.stopline_wp_index >= start_idx and self.stopline_wp_index <= end_index:
                self.drive_state = DriveState.Stop
                lane_waypoints = self.decel_waypoints(lane_waypoints, start_idx)
            elif self.drive_state == DriveState.Stop:
                self.drive_state = DriveState.Drive

            if self.drive_state == DriveState.Drive:
                if abs(self.vehicle_velocity - self.get_waypoint_velocity(lane_waypoints[0])) > 1.0:
                    if self.previous_velocity == None:
                        start_velocity = self.vehicle_velocity
                    else:
                        start_velocity = max(self.previous_velocity+0.2, self.vehicle_velocity)
                    lane_waypoints = self.accel_waypoints(lane_waypoints, start_velocity)
                else:
                    self.acceleration_start_velocity = None

            lane.waypoints = lane_waypoints
            self.final_waypoints_pub.publish(lane)
    

    def decel_waypoints(self, waypoints, closest_index):
        new_waypoints = []
        for i, waypoint in enumerate(waypoints):
            p = Waypoint()
            p.pose = waypoint.pose

            # stop_index is a bit further back from the stop line
            stop_index = max(self.stopline_wp_index - closest_index - 2, 0)
            dist = self.distance(waypoints, i, stop_index)
            vel = math.sqrt(2 * DECEL_RATE * dist)
            if vel < 1:
                vel = 0

            target_speed = min(vel, self.get_waypoint_velocity(waypoint))
            p.twist.twist.linear.x = target_speed
            new_waypoints.append(p)
        return new_waypoints

    def accel_waypoints(self, waypoints, start_velocity):
        new_waypoints = []
        for i, waypoint in enumerate(waypoints):
            p = Waypoint()
            p.pose = waypoint.pose

            dist = self.distance(waypoints, 0, i)
            vel = start_velocity + dist * ACCEL_RATE
            if vel < 0.5:
                vel = 0.5

            target_speed = min(vel, self.get_waypoint_velocity(waypoint))
            p.twist.twist.linear.x = target_speed
            new_waypoints.append(p)
        return new_waypoints

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if debug:
            rospy.logwarn('Waypoint -- loading base waypoints')
        self.base_waypoints = waypoints
        if debug:
            rospy.logwarn(str(self.waypoints_2d))
        if not self.waypoints_2d:
            self.waypoints_2d = []
            for waypoint in waypoints.waypoints:
                self.waypoints_2d.append(
                    [waypoint.pose.pose.position.x, waypoint.pose.pose.position.y])
            self.waypoint_tree = KDTree(self.waypoints_2d)
            if debug:
                rospy.logwarn('Waypoint -- wauypoint_tree' +
                              str(self.waypoint_tree))

    def traffic_cb(self, msg):
        if msg.data == -1:
            self.stopline_wp_index = None
        else:
            self.stopline_wp_index = msg.data

    def velocity_cb(self, velocity):
        self.vehicle_velocity = velocity.twist.linear.x

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

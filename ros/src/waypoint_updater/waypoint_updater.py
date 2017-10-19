#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math



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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
CORRIDOR = math.degrees(60.) # within this angle to the left and right of the ego we accept base_waypoints
INVERSE_WP_DENSITY = 5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None # pose.position, pose.orientation (quaternion)
        self.base_waypoints = None # list of pose's.
        self.closest_before = 0
        self.frame_id = None

        self.frequency = 10
        # TODO: Needs to be adjusted based on target speed
        self.tl_slow_down_dst = 40
        self.tl_stopp_dst = 3
        self.tl_idx = -1 #nearest traffic light with red status

        #rospy.spin()
        self.loop()


    def loop(self):
        # publish updates on fixed frequency as in dbw_node
        rate = rospy.Rate(self.frequency) # 50Hz
        while not rospy.is_shutdown():
            if self.base_waypoints != None and self.pose != None:
                #(x,y,z) = self.quaternion_to_euler_angle()

                closest = self.closest_wp(self.closest_before)

                # TODO this is a very basic setup of the Lane
                finalwps = Lane()
                finalwps.header.stamp = rospy.Time.now()
                finalwps.header.frame_id = self.frame_id

                i = 0
                idx = closest

                while i < LOOKAHEAD_WPS:
                    if idx >= len(self.base_waypoints) - 1:
                        idx = 0

                    velocity = self.base_waypoints[idx].twist.twist.linear.x

                    self.set_waypoint_velocity(self.base_waypoints, idx, velocity)
                    finalwps.waypoints.append(self.base_waypoints[idx])

                    i = i + 1
                    idx = idx + 1

                self.closest_before = closest
                self.final_waypoints_pub.publish(finalwps)

            rate.sleep()
    



    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints
        self.frame_id = waypoints.header.frame_id

        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data != self.tl_idx:
            self.tl_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def closest_wp(self, closest_before):
        dist = 999999999.
        closest = None

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        #alphal = lambda a, b: math.atan2((a.y - b.y), (a.x - b.x))

        # limit the search space
        upper = min(len(self.base_waypoints), closest_before + 700)
        lower = max(0, closest_before - 50)

        lower = 0
        upper = len(self.base_waypoints)

        for i in range(lower, upper):
            d = dl(self.base_waypoints[i].pose.pose.position, self.pose.position)
            #a = alphal(self.base_waypoints[i].pose.pose.position, self.pose.position)

            # TODO: check if there is an offset
            #if d < dist and ((z - CORRIDOR) < a) and ((z+CORRIDOR) > a):

            if d < dist:
                dist = d
                closest = i

        return closest


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

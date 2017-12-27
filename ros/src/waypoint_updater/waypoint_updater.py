#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import copy
import numpy as np

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
SLOW_DOWN_DIST = 100
STOP_DIST = 5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)


        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None # Waypoints from the course
        self.current_pose = None # Vehicls current position
        self.closest_idx = None # index to the closest waypoint
        self.frame_id = None # frame id??
        self.new_waypoints = None # New waypoints to send to the vehicle
        self.upcoming_traffic_light = None # Waypoint of any upcoming red traffic light
        self.current_velocity = None # Current Velocity of the vehicles
        self.max_velocity = None
        self.state = None # State of the vehicle ('GO', 'SLOW', 'STOP')
        self.velocity_curve = None # Polyfit for deacceleration

        rospy.loginfo('~~:Starting Waypoint Updater Loop')
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.current_pose is None or self.base_waypoints is None or self.upcoming_traffic_light is None:
                continue

            # rospy.loginfo('~~:Current Position-  x:{}, y:{}'.format(self.current_pose.position.x, self.current_pose.position.y))
            self.closest_idx = self.get_closest_waypoint(self.current_pose, self.base_waypoints)
            self.new_waypoints = self.load_new_waypoints(self.closest_idx)

            # rospy.loginfo('~~:Closest Waypoint - x:{}, y:{}'.format(self.new_waypoints[0].pose.pose.position.x, self.new_waypoints[0].pose.pose.position.y))
            lane = self.create_new_lane(self.frame_id, self.new_waypoints)
            self.final_waypoints_pub.publish(lane)


            rate.sleep()
        
#region Helper Functions

    #### HELPER FUNCTIONS ####

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

    def get_closest_waypoint(self, current_pose, waypoints):
        # loop through the base waypoints and find the closest one
        min_dist = float("inf")
        closest_idx = 0

        for idx, waypoint in enumerate(waypoints):
            dist = self.distance_between_two_points(current_pose.position, waypoint.pose.pose.position)

            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        return closest_idx

    def distance_between_two_points(self, p1, p2):
        """ Returns the distance between two points """
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return dx*dx + dy*dy

    def load_new_waypoints(self, start_idx):
        """ Creates a list of new waypoints starting from the idx of the closest waypoint """

        # Check that there are enough points left
        # end_idx = min(len(self.base_waypoints), start_idx + LOOKAHEAD_WPS)
        # new_waypoints = copy.deepcopy(self.base_waypoints[start_idx:end_idx])

        # if self.upcoming_traffic_light is None:
        #     return new_waypoints

        # Decision Making
        if self.upcoming_traffic_light == -1:
            self.state = 'GO'
        else:
            # Get distance to traffic light
            dist_to_tl = self.distance(self.base_waypoints, start_idx, self.upcoming_traffic_light)

            rospy.loginfo('~~:dist to tl: {}'.format(dist_to_tl))

            if dist_to_tl > SLOW_DOWN_DIST: # dont need to slow down till within 100m
                self.state = 'GO'
            elif dist_to_tl > STOP_DIST: # Slow down when more than 5m away
                self.state = 'SLOW'
            else: # Stop when within 5m of the TL
                self.state = 'STOP'

        # Waypoint creation
        new_waypoints = []
        for idx in range(LOOKAHEAD_WPS):
            waypoint = copy.deepcopy(self.base_waypoints[start_idx+idx])

            if self.state == 'GO':
                new_velocity = self.get_waypoint_velocity(waypoint)
                # rospy.loginfo('~~:1')
            
            elif self.state == 'SLOW':
                dist_to_tl = self.distance(self.base_waypoints, start_idx+idx, self.upcoming_traffic_light)
                new_velocity = self.velocity_profile(dist_to_tl)
                # rospy.loginfo('~~:2')

            else:
                new_velocity = 0.0
                # rospy.loginfo('~~:3')

            self.set_waypoint_velocity([waypoint], 0, new_velocity)

            new_waypoints.append(waypoint)

        # if self.upcoming_traffic_light != -1:
        #     dist_to_slow = (self.upcoming_traffic_light - start_idx) - 5 # Want to be 0 at least 5 steps before

        #     if dist_to_slow > 0.5:
        #         speed_decrease = self.current_velocity / dist_to_slow
        #     else:
        #         speed_decrease = 0

        #     rospy.loginfo('~~:dist_to_slow: {} | speed_decrease: {} | current_speed: {}'.format(dist_to_slow, speed_decrease, self.current_velocity))

        #     speed = self.current_velocity
        #     for i in range(len(new_waypoints)):
        #         speed = max(0, speed-speed_decrease)
        #         if speed <= 1:
        #             speed = 0
        #         # rospy.loginfo('~~:Speed: {}'.format(speed))
        #         self.set_waypoint_velocity(new_waypoints, i, speed)

        return new_waypoints

    def create_new_lane(self, frame_id, waypoints):
        new_lane = Lane()
        new_lane.header.frame_id = frame_id
        new_lane.waypoints = waypoints
        new_lane.header.stamp = rospy.Time.now()
        return new_lane

    def velocity_profile(self, distance):
        """ defines the velocity profile for slowing down """

        # Create curve if not already done
        if self.velocity_curve is None:
            self.create_curve()

        if distance < STOP_DIST: # Want to be stopped within 5m of the light
            return 0.0
        else:
            return self.velocity_curve(-distance) # negative distance for the curve. No reason, justs looks better in plot


    def create_curve(self):
        x = [-SLOW_DOWN_DIST,-5,-4,-3,-2,-1,0,1,2,3,4,5] # distances
        y = [self.max_velocity,0,0,0,0,0,0,0,0,0,0,0] # speeds

        # fit curve to the profile
        coeffs = np.polyfit(x,y,2)
        self.velocity_curve = np.poly1d(coeffs)


#endregion

#region Callbacks

    #### CALLBACKS ####

    def pose_cb(self, msg):
        # TODO: Implement
        
        # pos has stucture:
        # geometry_msgs/Point position
        #     float64 x
        #     float64 y
        #     float64 z
        # geometry_msgs/Quaternion orientation
        #     float64 x
        #     float64 y
        #     float64 z
        #     float64 w
        self.current_pose = msg.pose
        self.frame_id = msg.header.frame_id
        pass


    def waypoints_cb(self, msg):
        # TODO: Implement        
        # Called once, save the waypoints
        self.base_waypoints = msg.waypoints
        self.max_velocity = self.get_waypoint_velocity(self.base_waypoints[1000])

    def traffic_waypoint_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.upcoming_traffic_light = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


#endregion


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
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
        self.upcoming_traffic_light = None
        self.current_velocity = None

        rospy.loginfo('~~:Starting Waypoint Updater Loop')
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.current_pose is None:
                continue

            # rospy.loginfo('~~:Current Position-  x:{}, y:{}'.format(self.current_pose.position.x, self.current_pose.position.y))
            self.closest_idx = self.get_closest_waypoint(self.current_pose, self.base_waypoints)
            self.new_waypoints = self.load_new_waypoints(self.closest_idx, self.base_waypoints)

            # rospy.loginfo('~~:Closest Waypoint - x:{}, y:{}'.format(self.new_waypoints[0].pose.pose.position.x, self.new_waypoints[0].pose.pose.position.y))
            lane = self.create_new_lane(self.frame_id, self.new_waypoints)
            self.final_waypoints_pub.publish(lane)


            rate.sleep()
        
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

    def load_new_waypoints(self, start_idx, waypoints):
        """ Creates a list of new waypoints starting from the idx of the closest waypoint """

        # Check that there are enough points left
        end_idx = min(len(waypoints), start_idx + LOOKAHEAD_WPS)

        new_waypoints = waypoints[start_idx:end_idx]

        if self.upcoming_traffic_light is None:
            return new_waypoints

        rospy.loginfo('~~:current_vel: {}'.format(self.convert_ms_to_kph(self.current_velocity)))

        if self.upcoming_traffic_light != -1:
            dist_to_slow = self.upcoming_traffic_light - start_idx

            if dist_to_slow > 1:
                speed_decrease = self.convert_ms_to_kph(self.current_velocity) / dist_to_slow
            else:
                speed_decrease = 0

            rospy.loginfo('~~:dist_to_slow: {} | speed_decrease: {}'.format(dist_to_slow, speed_decrease))
            # speed = self.convert_ms_to_kph(self.current_velocity)
            # for i in range(dist_to_slow):
            #     new_waypoints[i].twist.twist.linear.x = speed
            #     speed = max(0, speed-speed_decrease)
            #     rospy.loginfo('~~:speed:{}'.format(speed))

        return new_waypoints

    def create_new_lane(self, frame_id, waypoints):
        new_lane = Lane()
        new_lane.header.frame_id = frame_id
        new_lane.waypoints = waypoints
        new_lane.header.stamp = rospy.Time.now()
        return new_lane

    def convert_ms_to_kph(self, speed_ms):
        return speed_ms*3.6


#endregion

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

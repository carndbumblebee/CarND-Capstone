
import rospy


from yaw_controller import YawController
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # Yaw Controller for the steering control
        self.controller_yaw = YawController(wheel_base, steer_ratio, 0.0, max_lat_accel, max_steer_angle)

    def control(self, twist_cmd_msg, current_vel_msg):
        # TODO: Change the arg, kwarg list to suit your needs
        
        linear_vel_desired = twist_cmd_msg.twist.linear.x
        angular_vel_desired = twist_cmd_msg.twist.angular.z

        linear_vel_current = current_vel_msg.twist.linear.x
        angular_vel_current = current_vel_msg.twist.angular.z
        
        steer = self.controller_yaw.get_steering( linear_vel_desired,
                                                    angular_vel_desired,
                                                    linear_vel_current)
        

        throttle = 0.3
        brake = 0.0
        
        # Return throttle, brake, steer
        return throttle, brake, steer

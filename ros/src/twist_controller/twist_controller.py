
import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # Yaw Controller for the steering control
        self.controller_steer = YawController(self.wheel_base, self.steer_ratio, 0.0, self.max_lat_accel, self.max_steer_angle)

        # PID Controller for the throttle and brake
        self.controller_throttle = PID(5.0,0.01,0.0)
        self.controller_brake = PID(100.0, 0.01, 0.0)

        self.prev_time = None

    def control(self, twist_cmd_msg, current_vel_msg, dbw_enabled):

        throttle = 0.0
        brake = 0.0
        steer = 0.0

        # Return if the commands are not loaded
        if not all((twist_cmd_msg, current_vel_msg)):
            return throttle, brake, steer

        if not self.prev_time:
            self.prev_time = rospy.get_time()
            return throttle, brake, steer

        if not dbw_enabled:
            self.controller_throttle.reset()
            return throttle, brake, steer

        # Grab the velocities and targets
        linear_vel_desired = twist_cmd_msg.twist.linear.x
        angular_vel_desired = twist_cmd_msg.twist.angular.z

        linear_vel_current = current_vel_msg.twist.linear.x
        angular_vel_current = current_vel_msg.twist.angular.z
        
        # Calc the steering angle using the inbuilt yaw controller
        steer = self.controller_steer.get_steering( linear_vel_desired,
                                                    angular_vel_desired,
                                                    linear_vel_current)
        
        ts = rospy.get_time() - self.prev_time
        self.prev_time = rospy.get_time()

        rospy.loginfo('~~:desired: {} current: {}'.format(linear_vel_desired, linear_vel_current))

        error = linear_vel_desired - linear_vel_current

        if linear_vel_desired == 0.0:
            self.controller_brake.reset()
            self.controller_throttle.reset()
            brake = 1000
        elif error > -self.brake_deadband:
            throttle = self.controller_throttle.step(error, ts)
            self.controller_brake.reset()
        else:
            brake = self.controller_brake.step(abs(error), ts)
            self.controller_throttle.reset()

        rospy.loginfo('~~:error: {} | throttle: {}, brake: {}, steer: {}'.format(error, throttle, brake, steer))

        # Calc the throttle and brake
        # output = self.controller_throttle.step(error, ts)

        # rospy.loginfo('~~:output: {}'.format(output))

        # if output > 0:
        #     throttle = output
        # elif output < -self.brake_deadband:
        #     brake = output
        # else:
        #     self.controller_throttle.reset()
        
        # Return throttle, brake, steer
        return throttle, brake, steer


import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
GAS_DENSITY_KG_M3 = 0.77
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
        self.max_throttle_percentage = rospy.get_param('~max_throttle_percentage', 0.1)
        self.max_braking_percentage = rospy.get_param('~max_braking_percentage', -0.1)

        # Yaw Controller for the steering control
        self.controller_steer = YawController(self.wheel_base, self.steer_ratio, 0.0, self.max_lat_accel, self.max_steer_angle)

        # PID Controller for the throttle and brake
        self.controller_throttle = PID(3.0,0.1,0.0)
        self.controller_brake = PID(4.0, 0.0, 0.0)

        # Low Pass Filter for Brake
        self.filter_brake = LowPassFilter(0.5, 0.02)
        self.filter_error = LowPassFilter(2, 0.5)

        # Brake Values
        # self.max_braking = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY_KG_M3) * self.decel_limit * self.wheel_radius
        # self.min_braking = 0

        self.prev_time = None

    def control(self, twist_cmd_msg, current_vel_msg, dbw_enabled):

        throttle = 0.0
        brake = 0.0
        steer = 0.0

        # See https://carnd.slack.com/archives/C6NVDVAQ3/p1506389845000004

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


        # Calculate the throttle and brake based upon a proportional amount
        # See https://carnd.slack.com/archives/C6NVDVAQ3/p1506389845000004

        error = abs(linear_vel_desired) - abs(linear_vel_current)
        ts = rospy.get_time() - self.prev_time
        self.prev_time = rospy.get_time()
        
        # If speeding up
        if error > 0.01:
            # First check if desired is 0, if so stop the car completely
            if linear_vel_desired <= 0.0:
                throttle = 0.0
                brake = 1.0
                self.controller_throttle.reset()
            else: # Calculate the throttle as a proportion of the desired vel
                throttle = min(self.controller_throttle.step(error/linear_vel_desired, ts), self.max_throttle_percentage)
                self.controller_brake.reset()
        
        # If slowing down but moving calc braking as a proportion of the current vel
        elif linear_vel_current > 0.1:
            brake = min(self.controller_brake.step(abs(error)/linear_vel_current,ts), 1.0)
            self.controller_throttle.reset()

        else: # Slowing down and slow moving
            throttle = 0.0
            brake = 1.0
            self.controller_throttle.reset()


        rospy.loginfo('~~:desired: {} current: {}'.format(linear_vel_desired, linear_vel_current))

        # error = linear_vel_desired - linear_vel_current
        # error = self.filter_error.filt(error)

        # if linear_vel_desired == 0.0:
        #     self.controller_brake.reset()
        #     self.controller_throttle.reset()
        #     brake = self.max_braking
        # elif error > -self.brake_deadband:
        #     throttle = self.controller_throttle.step(error, ts)
        #     self.controller_brake.reset()
        # else:
            # brake = self.controller_brake.step(abs(error), ts)
            # brake = self.filter_brake.filt(brake)

            # self.controller_throttle.reset()

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

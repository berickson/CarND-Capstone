import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, 
                 brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel,
                 max_steer_angle):
        
        # init members
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        # init controllers
        self.velocity_pid = PID(1.5, 0.001, 0.,
                                mn=decel_limit, mx=accel_limit)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 1, 
                                            max_lat_accel, max_steer_angle)
        
        pass

    def control(self, twist, current_velocity, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        velocity_cte = twist.twist.linear.x - current_velocity.twist.linear.x
        throttle = self.velocity_pid.step(velocity_cte, dt)
        steer = self.yaw_controller.get_steering(twist.twist.linear.x,
                                                 twist.twist.angular.z,
                                                 current_velocity.twist.linear.x)

        brake = 0.0
        
        #rospy.logwarn("twist_conroller throttle: %s", throttle)
        #rospy.logwarn("twist_conroller throttle: %s", steer)
        
        return throttle, brake, steer
        

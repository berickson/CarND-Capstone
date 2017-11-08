
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
        
        pass

    def control(self, twist, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.

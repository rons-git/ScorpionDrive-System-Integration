
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        kp = 0.5
        ki = 0.3
        kd = 0.2
        mn = 0.0    # minimum throttle value
        mx = 0.4    # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        tau = 0.5  # cutoff frequency: 1 / 2pi*tau
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        self.vehicle_mass = vehicle_mass + GAS_DENSITY*fuel_capacity
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_vel = None

        self.prev_time = rospy.get_time()

    def control(self, cur_vel, dbw_enabled, linear_vel, angular_vel):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        cur_vel = self.vel_lpf.filt(cur_vel)
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, cur_vel)

        vel_error = linear_vel - cur_vel
        self.last_vel = cur_vel

        cur_time = rospy.get_time()
        delta_time = cur_time - self.prev_time
        self.prev_time = cur_time

        throttle = self.throttle_controller.step(vel_error, delta_time)
        brake = 0

        if linear_vel == 0.0 and cur_vel < 0.1:
            throttle = 0.0
            # to prevent Carla from moving if we are stopped applying approximately 700 Nm of torque
            brake = 700.0

        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # torque N*m

        return throttle, brake, steering
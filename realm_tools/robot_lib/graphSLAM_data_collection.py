import numpy as np
import math
from realm_tools.robot_lib.my_robot import MyRobot


"""
This function gets the row in a 2d array where each index corresponds to
each landmark and the data stored is the distance to each landmark
"""
def get_landmark_row(robot, n_landmarks=8):
    row = np.full((n_landmarks, 2), np.nan, dtype=np.float32)  # <-- shape fix

    for obj in robot.camera.getRecognitionObjects():
        colors = obj.getColors()  # returns a flat list, e.g. [r, g, b] per color
        if not colors:
            continue
        color_key = tuple(colors[:3])
        rel_x, rel_y, _ = obj.getPosition()
        landmark_index = get_landmark_index(color_key)
        dist = math.hypot(rel_x, rel_y)
        bearing = math.atan2(rel_y, rel_x)
        row[landmark_index] = (dist, bearing)
    return row


LANDMARK_COLOR_MAP = {
    (1.0, 0.0, 0.0): 0,
    (0.0, 1.0, 0.0): 1,
    (0.0, 0.0, 1.0): 2,
    (1.0, 1.0, 0.0): 3,
    (0.0, 1.0, 1.0): 4,
    (1.0, 0.5, 0.0): 5,
    (0.5, 0.0, 0.5): 6,
    (0.0, 0.5, 0.5): 7,
}

def get_landmark_index(color_key):
    return LANDMARK_COLOR_MAP[color_key]

def get_odometry(robot, prev_encoder_readings, dt):
    """
    Compute linear and angular velocity from encoder deltas since the last call.

    Parameters
    ----------
    robot : Robot
    prev_encoder_readings : tuple/list (left, right) — encoder values from the previous timestep
    dt : float — elapsed time since the previous timestep (seconds)

    Returns
    -------
    v     : float — linear velocity (m/s)
    omega : float — angular velocity (rad/s)
    current_encoder_readings : tuple (left, right) — pass this in as prev_encoder_readings next call
    """
    current_encoder_readings = robot.get_encoder_readings()

    delta_left = current_encoder_readings[0] - prev_encoder_readings[0]
    delta_right = current_encoder_readings[1] - prev_encoder_readings[1]

    delta_s_left = delta_left * robot.wheel_radius
    delta_s_right = delta_right * robot.wheel_radius

    delta_s = (delta_s_left + delta_s_right) / 2.0
    delta_theta = (delta_s_right - delta_s_left) / robot.axel_length

    v = delta_s / dt
    omega = delta_theta / dt

    return v, omega, current_encoder_readings

def rotate_step(self, degrees=90, Kp=1, Ki=0, Kd=0, margin_error=0.01, max_accel=None):
    """
    Generator version of rotate(). Does ONE PID update per call to next().
    Caller is responsible for calling robot.experiment_supervisor.step(timestep)
    between each next() call.

    max_accel slew-rate limits the commanded velocity so it ramps up over
    a few steps instead of jumping straight to the PID's (often saturated)
    output on the very first step. Without this, a large initial heading
    error commands near-max velocity immediately, but the wheels can't
    actually reach that speed in one ~32ms timestep -- the encoder-derived
    odometry for that step ends up reflecting the abrupt commanded jump
    rather than the robot's actual smooth motion, showing up as a spurious
    velocity spike right at the move -> rotate transition. Defaults to
    max_motor_velocity / 10 (full speed reached after ~10 steps).

    Yields True while still rotating, then yields False once and stops
    (StopIteration) when the rotation is complete.
    """
    I = 0.0
    prev_error = 0.0
    dt = 0.032
    if max_accel is None:
        max_accel = self.max_motor_velocity / 10.0

    setpoint = (self.get_compass_reading() + degrees) % 360
    applied_velocity = 0.0

    while True:
        current_heading = self.get_compass_reading()
        error = (setpoint - current_heading + 180) % 360 - 180
        P = Kp * error
        I = Ki * (I + error * dt)
        D = Kd * ((error - prev_error) / dt)

        desired_velocity = abs(self.sat(P + I + D))

        if -margin_error <= error <= margin_error:
            self.stop()
            return  # done — generator ends here

        #ramp applied_velocity toward desired_velocity instead of jumping
        #straight to it, so the commanded velocity changes smoothly step
        #to step
        velocity_step = max(-max_accel, min(max_accel, desired_velocity - applied_velocity))
        applied_velocity += velocity_step
        out_signal = applied_velocity

        if error < 0:
            self.set_right_motor_velocity(-out_signal)
            self.set_left_motor_velocity(out_signal)
        elif error > 0:
            self.set_right_motor_velocity(out_signal)
            self.set_left_motor_velocity(-out_signal)

        prev_error = error
        yield  # give control back to the caller after one PID step

def move_forward_step(self, distance, Kp=20, margin_error=0.01):
    """
    Generator version of move_forward(). Does ONE control update per call to next().
    Caller is responsible for calling robot.experiment_supervisor.step(timestep)
    between each next() call.
    """
    starting_encoder_position = self.get_encoder_readings()

    while True:
        error = distance - self.calculate_wheel_distance_traveled(starting_encoder_position)

        if error <= margin_error:
            self.stop()
            return  # done — generator ends here

        self.go_forward(velocity=self.sat(Kp * error))
        yield  # give control back to the caller after one control step
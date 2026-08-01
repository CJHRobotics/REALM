from concurrent.futures import ThreadPoolExecutor
from realm_tools.robot_lib.hambot import HamBot
from realm_tools.image_lib.image_feature_lib import *
from realm_tools.robot_lib.navigation_tools import *
import operator
import numpy as np
import math
class MyRobot(HamBot):
    def __init__(self,action_length=0.5,enable_cnn_features=False,cnn_extractor_model=None):
        HamBot.__init__(self)
        self.action_length = action_length
        self.action_set = {
            0: [0, self.action_length],
            1: [45, self.action_length],
            2: [90, self.action_length],
            3: [135, self.action_length],
            4: [180, self.action_length],
            5: [225, self.action_length],
            6: [270, self.action_length],
            7: [315, self.action_length],
        }

        # Experiment Variables
        self.previous_action_index = -1

        # Enable CNN-based feature extraction if required
        self.enable_cnn_features = enable_cnn_features
        self.cnn_extractor_model = cnn_extractor_model
        if self.enable_cnn_features:
            from realm_tools.image_lib.feature_extractor import FeatureExtractor
            self.cnn_feature_extractor = FeatureExtractor(self.cnn_extractor_model)
        else:
            self.cnn_feature_extractor = None

    def get_robot_feature_pose(self):
        while self.experiment_supervisor.step(self.timestep) != -1:
            current_x, current_y, current_z = self.robot_translation_field.getSFVec3f()
            break
        return current_x, current_y, self.get_closest_action_heading()

    def get_robot_pov_features(self):
        """
        Get combined feature vector including CNN features, multimodal features, and robot pose.

        Returns:
            np.ndarray: Combined feature vector suitable for clustering.
        """
        pov, landmark_mask, landmark_azimuths = self.get_pov_image()
        x, y, theta = self.get_robot_pose()

        # Extract CNN features if enabled
        if self.enable_cnn_features:
            cnn_features = self.cnn_feature_extractor.get_cnn_features(pov)
        else:
            cnn_features = np.array([])  # Empty array if CNN features are not enabled

        multimodal_features = extract_combined_features(pov, landmark_mask, theta)

        return multimodal_features, cnn_features, landmark_mask, landmark_azimuths

    def capture_pov_images(self, thetas):
        """
        Capture one image per heading without extracting features.
        Use this for batch collection — accumulate images across many positions
        then extract features in parallel in the controller.
        Caller must have already teleported to the target position.

        Returns
        -------
        images            : list of POV images, one per heading
        landmark_masks    : list of (n_landmarks,) arrays — visibility per heading
        landmark_azimuths : list of (n_landmarks,) arrays — egocentric bearing (rad)
                            to each landmark, NaN where not visible
        """
        images = []
        landmark_masks = []
        landmark_azimuths = []
        for theta in thetas:
            self.robot_rotation_field.setSFRotation([0, 0, 1, theta])
            pov, mask, azimuths = self.get_pov_image()
            images.append(pov)
            landmark_masks.append(mask)
            landmark_azimuths.append(azimuths)
        return images, landmark_masks, landmark_azimuths

    def get_full_robot_pov_features(self, thetas):
        robot_x, robot_y, robot_theta = self.get_robot_feature_pose()

        # Phase 1 — capture all images sequentially (requires simulation steps)
        images = []
        for theta in thetas:
            self.robot_rotation_field.setSFRotation([0, 0, 1, theta])
            images.append(self.get_pov_image()[0])

        # Phase 2 — extract features in parallel (pure CPU, no Webots dependency)
        with ThreadPoolExecutor() as executor:
            multimodal_features = list(executor.map(extract_combined_features, images))
            if self.enable_cnn_features:
                cnn_features = list(executor.map(
                    self.cnn_feature_extractor.get_cnn_features, images))
            else:
                cnn_features = None

        return np.concatenate(multimodal_features), cnn_features

    def get_pov_image(self):
        while self.experiment_supervisor.step(self.timestep) != -1:
            # getImage() returns raw BGRA bytes — much faster than getImageArray()
            # which returns a Python list of lists that must be converted to numpy.
            w = self.camera.getWidth()
            h = self.camera.getHeight()
            img_bgra = np.frombuffer(self.camera.getImage(), dtype=np.uint8).reshape(h, w, 4)
            # Make contiguous RGB array — .copy() ensures cv2 operations downstream
            # don't pay a stride penalty from the reversed-channel view.
            pov = np.ascontiguousarray(img_bgra[:, :, 2::-1])

            landmark_mask, landmark_azimuths = self.get_landmark_observations()
            return pov, landmark_mask, landmark_azimuths

    def get_landmark_observations(self):
        """
        Determine which landmarks (from self.maze.landmarks) are visible in the
        current camera frame, and the egocentric bearing to each.

        Each landmark is identified by its unique recognitionColors. Bearings come
        from CameraRecognitionObject.getPosition(), which Webots already reports in
        the camera's own coordinate frame (x = forward, y = lateral) — so atan2(y, x)
        is the angle relative to the robot's current heading, with no global-frame
        math needed.

        Returns
        -------
        landmark_mask      : np.ndarray, shape (n_landmarks,) — 1 if visible, else 0
        landmark_azimuths  : np.ndarray, shape (n_landmarks,) — egocentric bearing
                             (radians) to each landmark, NaN where not visible
        """
        n_landmarks = len(self.maze.landmarks)
        landmark_mask = np.zeros(n_landmarks, dtype=np.float32)
        landmark_azimuths = np.full(n_landmarks, np.nan, dtype=np.float32)
        color_to_id = {tuple(round(c, 2) for c in lm.color): lm.id for lm in self.maze.landmarks}

        for obj in self.camera.getRecognitionObjects():
            color = tuple(round(c, 2) for c in obj.getColors()[:3])
            landmark_id = color_to_id.get(color)
            if landmark_id is None:
                continue
            rel_x, rel_y, rel_z = obj.getPosition()
            landmark_mask[landmark_id] = 1
            landmark_azimuths[landmark_id] = math.atan2(rel_y, rel_x)

        return landmark_mask, landmark_azimuths
    def get_closest_action_index(self):
        return int((self.get_compass_reading() // 45))

    def get_closest_action_heading(self):
        """
        Rounds the given heading (in degrees) to the nearest increment of 45 degrees.
        Handles headings in the range [0, 360] and ensures 360 is treated as 0.

        Parameters:
        - heading: float or int, the heading in degrees (e.g., 314, 316, 5, 360, 355).

        Returns:
        - int: The closest heading that is a multiple of 45 (0, 45, 90, 135, 180, 225, 270, 315).
        """
        # Normalize heading to [0, 360)
        heading = self.get_compass_reading() % 360

        # Divide by 45, round to nearest integer, and multiply back by 45
        closest_heading = round(heading / 45) * 45

        # Ensure 360 is returned as 0
        if closest_heading == 360:
            closest_heading = 0

        return closest_heading

    # Calculates the vector needed to move the robot to the point (x,y)
    def calculate_robot_motion_vector(self, x, y):
        self.sensor_calibration()
        current_position = self.gps.getValues()[0:2]
        return calculate_motion_vector(current_position[0], current_position[1], x, y)

    # Caps the motor velocities to ensure PID calculations do no exceed motor speeds
    def velocity_saturation(self, motor_velocity):
        if motor_velocity > self.max_motor_velocity:
            return self.max_motor_velocity
        elif motor_velocity < -1 * self.max_motor_velocity:
            return -1 * self.max_motor_velocity
        else:
            return motor_velocity

    # Sets motor speeds using PID to turn robot to desired bearing
    def rotation_PID(self, end_bearing, K_p=1):
        delta = end_bearing - self.get_compass_reading()
        velocity = self.velocity_saturation(K_p * abs(delta))
        if -180 <= delta <= 0 or 180 < delta <= 360:
            self.left_motor.setVelocity(1 * velocity)
            self.right_motor.setVelocity(-1 * velocity)

        elif 0 < delta <= 180 or -360 <= delta < -180:
            self.left_motor.setVelocity(-1 * velocity)
            self.right_motor.setVelocity(1 * velocity)

    # Sets motor speeds using PID to move the robot forward a desired distance in mm
    def forward_motion_with_encoder_PID(self, travel_distance, starting_encoder_position, K_p=100):
        delta = travel_distance - self.calculate_wheel_distance_traveled(starting_encoder_position)
        velocity = self.velocity_saturation(K_p * delta)
        for motor in self.all_motors:
            motor.setVelocity(velocity)

    # Sets the motor speeds using PID to move the robot to the point (x,y)
    def forward_motion_with_xy_PID(self, x, y, K_p=10):
        current_position = self.gps.getValues()[0:2]
        delta_x = x - current_position[0]
        delta_y = y - current_position[1]
        delta = (delta_x + delta_y) / 2
        velocity = self.velocity_saturation(K_p * delta)
        for motor in self.all_motors:
            motor.setVelocity(velocity)

    # Rotates the robot in place to face end_bearing and stops within margin_error (DEFAULT: +-.001)
    def rotate_to(self, end_bearing, margin_error=.0001):
        counter = 0
        while self.experiment_supervisor.step(self.timestep) != -1:
            self.rotation_PID(end_bearing)
            counter += 1
            if end_bearing - margin_error <= self.get_compass_reading() <= end_bearing + margin_error:
                self.stop()
                break

    # Moves the robot forward in a straight line by the amount distance (in mm)
    def move_forward_with_PID(self, distance, margin_error=.01, safty=True):
        forward_lidar_window = 15
        starting_encoder_position = self.get_encoder_readings()
        while self.experiment_supervisor.step(self.timestep) != -1:
            self.forward_motion_with_encoder_PID(distance, starting_encoder_position)
            if (distance - margin_error <=
                    self.calculate_wheel_distance_traveled(starting_encoder_position) <= distance + margin_error):
                self.stop()
                break
            if safty:
                if (min(self.lidar.getRangeImage()[180 - forward_lidar_window:180 + forward_lidar_window]) < .4):
                    self.stop()
                    break

    # Moves the robot forward in a straight line by the amount distance (in mm)
    def move_forward_no_PID(self, distance, velocity=20, margin_error=.01):
        forward_lidar_window = 15
        starting_encoder_position = self.get_encoder_readings()
        while self.experiment_supervisor.step(self.timestep) != -1:
            self.go_forward(velocity=velocity)
            if (distance - margin_error <=
                    self.calculate_wheel_distance_traveled(starting_encoder_position) <= distance + margin_error):
                self.stop()
                break
            if (min(self.lidar.getRangeImage()[180 - forward_lidar_window:180 + forward_lidar_window]) < .25):
                self.stop()
                break

    # Moves the robot to the point (x,y) by rotating and then moving in a straight line
    def move_to_xy_with_PID(self, x, y, margin_error=.1):
        motion_vector = self.calculate_robot_motion_vector(x, y)
        if not (motion_vector[0] - margin_error <=
                self.get_compass_reading() <=
                motion_vector[0] + margin_error):
            self.rotate_to(motion_vector[0])
        motion_vector = self.calculate_robot_motion_vector(x, y)
        self.move_forward_with_PID(motion_vector[1])

    # Moves the robot to the point (x,y) by rotating and then moving in a straight line
    def move_to_xy_no_PID(self, x, y, velocity=20, margin_error=.01):
        motion_vector = self.calculate_robot_motion_vector(x, y)
        print(motion_vector)
        if not (motion_vector[0] - margin_error <=
                self.get_compass_reading() <=
                motion_vector[0] + margin_error):
            self.rotate_to(motion_vector[0])

        self.move_forward_no_PID(motion_vector[1])

    def slow_stop(self):
        while self.experiment_supervisor.step(self.timestep) != -1:
            current_velocity = self.left_motor.getVelocity()
            if current_velocity > .1:
                for motor in self.all_motors:
                    motor.setVelocity(current_velocity / 4)
            else:
                self.stop()
                break

    def rotate(self, degrees = 90,  Kp=1, Ki=0, Kd=0, margin_error = 0.01):
        I = 0.0
        prev_error = 0.0
        dt = 0.032

        # degrees > 0 -> ccw
        # degrees < 0 -> cw

        setpoint = (self.get_compass_reading() + degrees) % 360

        while self.experiment_supervisor.step(self.timestep) != -1:
            current_heading =  self.get_compass_reading()
            error = (setpoint - current_heading+180)%360 -180 # (-180,180)
            P = Kp * error

            I = Ki * (I + error * dt)

            D = Kd * ((error - prev_error) / dt)

            out_signal = abs(self.sat(P + I + D))
            if -margin_error <= error <= margin_error:
                self.stop()
                break
            elif error < 0:
                self.set_right_motor_velocity(-out_signal)
                self.set_left_motor_velocity(out_signal)
            elif error > 0:
                self.set_right_motor_velocity(out_signal)
                self.set_left_motor_velocity(-out_signal)

            prev_error = error
    def calculate_wheel_distance_traveled(self, starting_encoder_position):
        current_encoder_readings = self.get_encoder_readings()
        differences = list(map(operator.sub, current_encoder_readings, starting_encoder_position))
        average_differences = sum(differences) / len(differences)
        average_distance = average_differences * self.wheel_radius
        return average_distance
    def move_forward(self, distance, Kp=20, margin_error=0.01):
        starting_encoder_position = self.get_encoder_readings()
        while self.experiment_supervisor.step(self.timestep) != -1:
            error = distance - self.calculate_wheel_distance_traveled(starting_encoder_position)
            if error <= margin_error:
                self.stop()
                break
            self.go_forward(velocity=self.sat(Kp * error))

    def perform_random_action(self, bias=True):

        available_actions = [int(i) for i in self.get_possible_actions()]
        # Add motion Bias and normalize
        if bias:
            action_distribution = apply_softmax(add_motion_bias(available_actions, self.previous_action_index))
        else:
            action_distribution = apply_softmax(available_actions)
        random_action_index = np.random.choice(8, 1, p=action_distribution)[0]

        if self.check_if_action_is_possible(random_action_index):
            random_action = self.action_set.get(random_action_index)
            self.rotate_to(random_action[0])
            self.move_forward_with_PID(random_action[1])
        else:
            random_action_index = np.argmax(action_distribution)
            random_action = self.action_set.get(random_action_index)
            self.rotate_to(random_action[0])
            self.move_forward_with_PID(random_action[1])

        self.previous_action_index = random_action_index
        return random_action_index

    def perform_habituation_action(self, bias=True):
        self.sensor_calibration()
        available_actions = [int(i) for i in self.get_possible_actions()]
        # Add motion Bias and normalize
        if bias:
            action_distribution = apply_softmax(add_motion_bias(available_actions, self.previous_action_index))
        else:
            action_distribution = apply_softmax(available_actions)
        random_action_index = np.random.choice(8, 1, p=action_distribution)[0]
        if self.check_if_action_is_possible(random_action_index, teleport=True):
            self.perform_training_action_teleport(random_action_index)
        else:
            # Try the remaining actions sorted by probability
            sorted_indices = np.argsort(action_distribution)[::-1]  # descending order
            for idx in sorted_indices:
                if self.check_if_action_is_possible(idx, teleport=True):
                    self.perform_training_action_teleport(idx)
                    random_action_index = idx
                    break

        self.previous_action_index = random_action_index
        return random_action_index

    def perform_action_with_PID(self, action_index):
        action = self.action_set.get(action_index)
        if self.check_if_action_is_possible(action_index=action_index):
            self.rotate_to(action[0])
            self.move_forward_with_PID(action[1])
        else:
            print("cant preform action")

    def perform_action_no_PID(self, action_index):
        action = self.action_set.get(action_index)
        if self.check_if_action_is_possible(action_index=action_index):
            self.rotate_to(action[0])
            self.move_forward_no_PID(500 * action[1])
        else:
            print("cant preform action")

    def perform_training_action(self, action_index):
        action = self.action_set.get(action_index)
        self.rotate_to(action[0])
        self.move_forward_with_PID(action[1])

    def perform_training_action_teleport(self, action_index):
        action = self.action_set.get(action_index)
        curr_x, curr_y, curr_theta = self.get_robot_pose()
        action_theta = math.radians(action[0])
        new_x = curr_x + self.action_length * math.cos(action_theta)
        new_y = curr_y + self.action_length * math.sin(action_theta)
        self.teleport_robot(x=new_x, y=new_y, theta=action_theta)
        return self.get_robot_pose()

    def get_possible_actions(self):
        min_action_distance = self.action_length + 0.25
        while self.experiment_supervisor.step(self.timestep) != -1:
            relative_distances = RelativeDistances(lidar_range_image=self.lidar.getRangeImage())
            available_actions = [0] * 8
            bin_index = 0
            front_action_index = self.get_closest_action_index()
            for bin in relative_distances.distance_bins:
                action_index = (front_action_index - bin_index) % 8
                available_actions[action_index] = min(bin) > min_action_distance
                bin_index += 1

            return available_actions

    def get_possible_training_actions(self):
        available_actions = self.get_possible_actions()
        return [i for i in range(len(available_actions)) if available_actions[i]]

    def get_possible_training_action_mask(self):
        available_actions = np.array(self.get_possible_actions())
        return np.array(np.multiply(available_actions, 1), dtype=np.float32)

    def check_if_action_is_possible(self, action_index=-1, teleport=False):
        forward_lidar_window = 45
        min_action_distance = .5
        if action_index == -1:
            if min(self.lidar.getRangeImage()[
                   180 - forward_lidar_window:180 + forward_lidar_window]) > min_action_distance:
                return True
            else:
                return False
        else:
            action = self.action_set.get(action_index)
            if teleport:
                curr_x, curr_y, curr_theta = self.get_robot_pose()
                self.teleport_robot(curr_x, curr_y, theta=math.radians(action[0]))
            else:
                self.rotate_to(action[0])
            if min(self.lidar.getRangeImage()[
                   180 - forward_lidar_window:180 + forward_lidar_window]) > min_action_distance:
                return True
            else:
                return False

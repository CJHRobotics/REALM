from realm_tools.robot_lib.hambot import HamBot
from realm_tools.image_lib.image_feature_lib import *
from realm_tools.robot_lib.robot_tools import *
from realm_tools.image_lib.feature_extractor import FeatureExtractor
import operator
import numpy as np
import math
class ChanceRobot(HamBot):
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
            self.cnn_feature_extractor = FeatureExtractor(self.cnn_extractor_model)
        else:
            self.cnn_feature_extractor = None

    def get_robot_feature_pose(self):
        while self.experiment_supervisor.step(self.timestep) != -1:
            current_x, current_y, current_z = self.robot_translation_field.getSFVec3f()
            break
        return current_x, current_y, self.get_closest_action_heading()

    def get_robot_pov_features(self, landmark_dictionary):
        """
        Get combined feature vector including CNN features, multimodal features, and robot pose.

        Args:
            landmark_dictionary (dict): Dictionary to identify landmarks in the POV image.

        Returns:
            np.ndarray: Combined feature vector suitable for clustering.
        """
        pov, landmark_mask = self.get_pov_image(landmark_dictionary)
        x, y, theta = self.get_robot_pose()

        # Extract CNN features if enabled
        if self.enable_cnn_features:
            cnn_features = self.cnn_feature_extractor.get_cnn_features(pov)
        else:
            cnn_features = np.array([])  # Empty array if CNN features are not enabled

        multimodal_features = extract_combined_features(pov, landmark_mask, theta)

        return multimodal_features, cnn_features, landmark_mask

    def get_full_robot_pov_features(self,thetas):
        robot_x, robot_y, robot_theta = self.get_robot_feature_pose()
        multimodal_features = []
        cnn_features = []
        for theta in thetas:
            self.teleport_robot(x=robot_x, y=robot_y, theta=theta)
            pov = self.get_pov_image()
            multimodal_features.append(extract_combined_features(pov))
            # Extract CNN features if enabled
            if self.enable_cnn_features:
                cnn_features.append(self.cnn_feature_extractor.get_cnn_features(pov))
            else:
                cnn_features = np.array([])  # Empty array if CNN features are not enabled
        multimodal_features = np.concatenate(multimodal_features)
        cnn_features = None
        return multimodal_features, cnn_features

    def get_pov_image(self, landmark_dictionary=None):
        while self.experiment_supervisor.step(self.timestep) != -1:
            landmark_mask = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            pov = self.camera.getImageArray()
            if landmark_dictionary != None:
                landmarks = self.camera.getRecognitionObjects()
                for landmark in landmarks:
                    color = landmark.getColors()
                    color = [color[0], color[1], color[2]]
                    key_list = list(landmark_dictionary.keys())
                    val_list = list(landmark_dictionary.values())
                    position = val_list.index(color)
                    mask_index = key_list[position]
                    landmark_mask[mask_index] = 1
                return pov, landmark_mask
            return pov
    def get_closest_action_index(self):
        return int((self.get_bearing() // 45))

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
        heading = self.get_bearing() % 360

        # Divide by 45, round to nearest integer, and multiply back by 45
        closest_heading = round(heading / 45) * 45

        # Ensure 360 is returned as 0
        if closest_heading == 360:
            closest_heading = 0

        return closest_heading

        # Calculates the average mm all the wheels have traveled from a relative starting encoder reading
    def calculate_wheel_distance_traveled(self, starting_encoder_position):
        current_encoder_readings = self.get_encoder_readings()
        differences = list(map(operator.sub, current_encoder_readings, starting_encoder_position))
        average_differences = sum(differences) / len(differences)
        average_distance = average_differences * self.wheel_radius
        return average_distance

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
        delta = end_bearing - self.get_bearing()
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
            if end_bearing - margin_error <= self.get_bearing() <= end_bearing + margin_error:
                self.stop()
                break

    # Rotates the robot by the amount degree. Only rotates until robot reaches the calculated end_bearing
    def rotate(self, degree, margin_error=.0001):
        start_bearing = self.get_bearing()
        end_bearing = start_bearing - degree
        if end_bearing > 360:
            end_bearing -= 360
        elif end_bearing < 0:
            end_bearing += 360
        self.rotate_to(end_bearing, margin_error=margin_error)

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
                self.get_bearing() <=
                motion_vector[0] + margin_error):
            self.rotate_to(motion_vector[0])
        motion_vector = self.calculate_robot_motion_vector(x, y)
        self.move_forward_with_PID(motion_vector[1])

    # Moves the robot to the point (x,y) by rotating and then moving in a straight line
    def move_to_xy_no_PID(self, x, y, velocity=20, margin_error=.01):
        motion_vector = self.calculate_robot_motion_vector(x, y)
        print(motion_vector)
        if not (motion_vector[0] - margin_error <=
                self.get_bearing() <=
                motion_vector[0] + margin_error):
            self.rotate_to(motion_vector[0])

        self.move_forward_no_PID(motion_vector[1])

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

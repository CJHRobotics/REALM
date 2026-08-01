import math
import numpy as np
from scipy.special import softmax
class RelativeDistances:
    def __init__(self, lidar_range_image, window=22):
        self.window = window
        self.lidar_range_image = lidar_range_image
        self.bin_indices = {}

        def get_range(center):
            start = (center - window) % 360
            end = (center + window) % 360
            self.bin_indices[center] = (start, end)
            if start <= end:
                return lidar_range_image[start:end + 1]
            else:
                return lidar_range_image[start:] + lidar_range_image[:end + 1]

        # Define directional bins centered at 45° increments
        self.front_distances = get_range(180)
        self.front_right_distances = get_range(225)
        self.right_distances = get_range(270)
        self.rear_right_distances = get_range(315)
        self.rear_distances = get_range(0)
        self.rear_left_distances = get_range(45)
        self.left_distances = get_range(90)
        self.front_left_distances = get_range(135)

        # Clockwise ordering from front
        self.distance_bins = [
            self.front_distances,
            self.front_right_distances,
            self.right_distances,
            self.rear_right_distances,
            self.rear_distances,
            self.rear_left_distances,
            self.left_distances,
            self.front_left_distances
        ]

    def __str__(self):
        label_map = {
            180: "Front",
            225: "Front Right",
            270: "Right",
            315: "Rear Right",
            0: "Rear",
            45: "Rear Left",
            90: "Left",
            135: "Front Left"
        }
        output = ["Distance Bin Ranges (index-based):"]
        for center in [180, 225, 270, 315, 0, 45, 90, 135]:
            start, end = self.bin_indices[center]
            output.append(f"{label_map[center]:<13}: start = {start}, end = {end}")
        return "\n".join(output)


# Function to calculate the angle and distance between two points (x1,y1) and (x2,y2)
def calculate_motion_vector(x1, y1, x2, y2):
    theta = int(math.degrees(math.atan2((y2 - y1), (x2 - x1))))
    if theta < 0:
        theta += 360
    magnitude = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return np.array([theta, magnitude])

def add_motion_bias(available_actions, previous_action_index):
    if previous_action_index == -1:
        return available_actions
    else:
        action_distribution = [i for i in available_actions]
        # Previous action bias
        action_distribution[previous_action_index] += 3
        # Adjacent action bias
        action_distribution[(previous_action_index - 1) % 8] += 2
        action_distribution[(previous_action_index + 1) % 8] += 2
        # Orthogonal action bias
        action_distribution[(previous_action_index - 2) % 8] += 1
        action_distribution[(previous_action_index + 2) % 8] += 1
        # Eliminate not available actions
        action_distribution = np.multiply(action_distribution, available_actions)
    return action_distribution

def apply_softmax(action_distrabution):
    return softmax(action_distrabution)
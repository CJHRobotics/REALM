import math
import random
from collections import deque

import matplotlib.pyplot as plt
from matplotlib import collections as pycol
from matplotlib import patches

from realm_tools.simulation_lib.environment_parser import parse_environment


class RandomStartPositionGenerator:
    def __init__(self, starting_positions):
        self.starting_positions = deque(starting_positions)
        self.used_positions = []

    def get_random_start_position(self):
        if not self.starting_positions:
            self.starting_positions.extend(self.used_positions)
            self.used_positions.clear()
        position = random.choice(list(self.starting_positions))
        self.starting_positions.remove(position)
        self.used_positions.append(position)
        return position


class Environment:
    def __init__(self, environment_file, display_width=1000, display_height=1000):
        self.boundary_walls = []
        self.obstacles = []
        self.landmarks = []
        self.training_starting_location = []
        self.testing_start_location = []
        self.goal_locations = []
        self._wall_lines = []

        walls, landmarks, goals, train_starts, test_starts = parse_environment(environment_file)

        for i, w in enumerate(walls):
            wall = Wall(w['x1'], w['y1'], w['x2'], w['y2'],
                        height=w['height'], width=w['width'],
                        wall_type=w['wall_type'], texture=w['texture'], id=i)
            if w['wall_type'] == 'boundary':
                self.boundary_walls.append(wall)
            else:
                self.obstacles.append(wall)
            self._wall_lines.append([(w['x1'], w['y1']), (w['x2'], w['y2'])])

        for i, lm in enumerate(landmarks):
            self.landmarks.append(Landmark(lm, id=i))

        for p in train_starts:
            self.training_starting_location.append(StartingPosition(p['x'], p['y'], p['theta']))

        for p in test_starts:
            self.testing_start_location.append(StartingPosition(p['x'], p['y'], p['theta']))

        for g in goals:
            self.goal_locations.append(Goal(g['x'], g['y'], g['id']))

        self.make_environment_plot(display_width, display_height)

        self.random_training_start_position_generator = RandomStartPositionGenerator(self.training_starting_location)
        self.random_testing_starting_position_generator = RandomStartPositionGenerator(self.testing_start_location)

    def make_environment_plot(self, display_width, display_height):
        self.environment_figure, self.environment_figure_ax = plt.subplots(
            1, 1, figsize=(display_width / 100, display_height / 100))
        self.environment_figure_ax.add_collection(pycol.LineCollection(self._wall_lines, linewidths=2))
        for p in self.training_starting_location:
            self.environment_figure_ax.add_patch(patches.Circle((p.x, p.y), radius=0.05, color='green'))
        for p in self.testing_start_location:
            self.environment_figure_ax.add_patch(patches.Circle((p.x, p.y), radius=0.05, color='blue'))
        for g in self.goal_locations:
            self.environment_figure_ax.add_patch(patches.Circle((g.x, g.y), radius=0.05, color='red'))
        self.environment_figure_ax.set_ylim(-4.25, 4.25)
        self.environment_figure_ax.set_xlim(-4.25, 4.25)
        self.environment_figure_ax.margins(0.1)

    def close_environment_figure(self):
        plt.close(self.environment_figure)

    def get_random_training_starting_position(self):
        return self.random_training_start_position_generator.get_random_start_position()

    def get_random_testing_starting_position(self):
        return self.random_testing_starting_position_generator.get_random_start_position()

    def get_environment_figure(self):
        return self.environment_figure, self.environment_figure_ax

    def get_goal_location(self):
        return self.goal_locations[0].x, self.goal_locations[0].y


class EnvironmentPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to_point(self, robot_position):
        return math.dist((self.x, self.y), (robot_position[0], robot_position[1])) - robot_position[2]


class Goal(EnvironmentPoint):
    def __init__(self, x, y, id):
        super().__init__(x, y)
        self.goal_id = id


class StartingPosition(EnvironmentPoint):
    def __init__(self, x, y, theta):
        super().__init__(x, y)
        self.theta = theta


class Wall:
    def __init__(self, x1, y1, x2, y2, height=0.3, width=0.012, wall_type='obstacle', texture=None, id=0):
        self.x1, self.y1 = x1, y1
        self.x2, self.y2 = x2, y2
        self.height = height
        self.width = width
        self.wall_type = wall_type
        self.texture = texture
        self.id = id
        self.length = math.dist((x1, y1), (x2, y2))
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        self.translation = [cx, cy, height / 2]
        theta = math.atan2(x1 - x2, y2 - y1)
        self.rotation = [0, 0, 1, theta]

    def get_webots_node_string(self):
        prefix = 'boundary_wall' if self.wall_type == 'boundary' else 'obstacle'
        node = (
            f'DEF {prefix}_{self.id} wall {{ '
            f'translation {self.translation[0]:.2f} {self.translation[1]:.2f} {self.translation[2]:.2f} '
            f'rotation {self.rotation[0]:.2f} {self.rotation[1]:.2f} {self.rotation[2]:.2f} {self.rotation[3]:.2f} '
            f'size {self.width:.3f} {self.length:.3f} {self.height:.3f}'
        )
        if self.texture:
            node += (
                f' appearance PBRAppearance {{ '
                f'baseColorMap ImageTexture {{ url ["{self.texture}"] }} '
                f'metalness 0 roughness 0.5 }}'
            )
        return node + ' }'


class Landmark:
    def __init__(self, data, id=0):
        self.landmark_type = data['type']
        self.x = data['x']
        self.y = data['y']
        self.theta = data.get('theta', 0.0)
        self.height = data['height']
        self.color = [data['red'], data['green'], data['blue']]
        self.id = id

        if self.landmark_type == 'cylinder':
            self.radius = data.get('radius', 0.25)
        elif self.landmark_type == 'panel':
            self.width = data.get('width', 1.0)
            self.texture = data.get('texture', '')

        self.translation = [self.x, self.y, self.height / 2]
        self.rotation = [0, 0, 1, self.theta]

    def get_webots_node_string(self):
        if self.landmark_type == 'cylinder':
            return self._cylinder_node_string()
        elif self.landmark_type == 'panel':
            return self._panel_node_string()

    def _cylinder_node_string(self):
        r, g, b = self.color
        return (
            f'DEF landmark_{self.id} landmark {{ '
            f'translation {self.translation[0]:.2f} {self.translation[1]:.2f} {self.translation[2]:.2f} '
            f'color {r:.2f} {g:.2f} {b:.2f} '
            f'recognitionColors [{r:.2f} {g:.2f} {b:.2f}] '
            f'size {self.height:.2f} {self.radius:.2f} {self.radius - 0.01:.2f} }}'
        )

    def _panel_node_string(self):
        r, g, b = self.color
        return (
            f'DEF panel_{self.id} RectangularPanel {{ '
            f'translation {self.translation[0]:.2f} {self.translation[1]:.2f} {self.translation[2]:.2f} '
            f'rotation {self.rotation[0]:.2f} {self.rotation[1]:.2f} {self.rotation[2]:.2f} {self.rotation[3]:.2f} '
            f'color {r:.2f} {g:.2f} {b:.2f} '
            f'recognitionColors [{r:.2f} {g:.2f} {b:.2f}] '
            f'size {self.width:.2f} {self.height:.2f} '
            f'signImage ["{self.texture}"] }}'
        )

import math
import random
from collections import deque

import matplotlib.pyplot as plt
from matplotlib import collections as pycol
from matplotlib import patches

from fairis_lib.simulation_lib.maze_parser import parse_maze


class RandomStartPositionGenerator:
    def __init__(self, starting_positions):
        self.starting_positions = deque(starting_positions)
        self.used_positions = []

    def get_random_start_position(self):
        if not self.starting_positions:
            # All starting positions have been used once, reset the list.
            self.starting_positions.extend(self.used_positions)
            self.used_positions.clear()

        position = random.choice(self.starting_positions)
        self.starting_positions.remove(position)
        self.used_positions.append(position)
        return position


def make_maze_plot_with_pc(display_width, display_height, maze_file="simulation/worlds/mazes/Experiment1/WM00.xml"):
    maze = Maze(maze_file)

    fig, axs = plt.subplots(figsize=(display_width / 100, display_height / 100))

    axs.add_collection(pycol.LineCollection(maze.walls, linewidths=2))

    for point in maze.experiment_starting_location:
        new_crc = patches.Circle((point.x, point.y), radius=.05, color='green')
        axs.add_patch(new_crc)
    for point in maze.habituation_start_location:
        new_crc = patches.Circle((point.x, point.y), radius=.05, color='blue')
        axs.add_patch(new_crc)
    for point in maze.goal_locations:
        new_crc = patches.Circle((point.x, point.y), radius=.05, color='red')
        axs.add_patch(new_crc)

    axs.set_ylim(-4.25, 4.25)
    axs.set_xlim(-4.25, 4.25)
    axs.margins(0.1)

    return fig, axs


class Maze:
    def __init__(self, maze_file, display_width=1000, display_height=1000):

        self.length = 0
        self.width = 0
        self.boundary_walls = []
        self.experiment_starting_location = []
        self.habituation_start_location = []
        self.goal_locations = []
        self.obstacles = []
        self.walls = []
        self.cylinder_landmarks = []
        self.tag_landmarks = []

        walls, goals, experiment_start_positions, habituation_start_positions, cylinder_landmarks, tag_landmarks = parse_maze(maze_file)
        for index, row in walls.iterrows():
            if index <= 3:
                self.boundary_walls.append(BoundaryWall(row['x1'], row['y1'], row['x2'], row['y2'], width=row['width'], id=index))
            else:
                self.obstacles.append(Obstacle(row['x1'], row['y1'], row['x2'], row['y2'], width=row['width'], id=index - 4))
            self.walls.append([(row['x1'], row['y1']), (row['x2'], row['y2'])])

        for index, row in experiment_start_positions.iterrows():
            self.experiment_starting_location.append(StartingPosition(row['x'], row['y'], row['theta']))

        for index, row in habituation_start_positions.iterrows():
            self.habituation_start_location.append(StartingPosition(row['x'], row['y'], row['theta']))

        for index, row in goals.iterrows():
            self.goal_locations.append(Goal(row['x'], row['y'], row['id']))

        for index, row in cylinder_landmarks.iterrows():
            self.cylinder_landmarks.append(CylinderLandmark(row['x'], row['y'], color=[row['red'], row['green'], row['blue']], id=index))

        for index, row in tag_landmarks.iterrows():
            self.tag_landmarks.append(TagLandmark(row['x'],
                                                  row['y'],
                                                  row['theta'],
                                                  row['tag_id'],
                                                  row['height'],
                                                  row['width'],
                                                  color=[row['red'], row['green'], row['blue']]))

        self.make_maze_plot(display_width, display_height)

        self.random_experiment_starting_position_generator = RandomStartPositionGenerator(
            self.experiment_starting_location)
        self.random_habituation_starting_position_generator = RandomStartPositionGenerator(
            self.habituation_start_location)
        self.random_testing_starting_position_generator = RandomStartPositionGenerator(self.habituation_start_location)

    def make_maze_plot(self, display_width, display_height):

        self.maze_figure, self.maze_figure_ax = plt.subplots(1, 1, figsize=(display_width / 100, display_height / 100))

        self.maze_figure_ax.add_collection(pycol.LineCollection(self.walls, linewidths=2))

        for point in self.experiment_starting_location:
            new_crc = patches.Circle((point.x, point.y), radius=.05, color='green')
            self.maze_figure_ax.add_patch(new_crc)
        for point in self.habituation_start_location:
            new_crc = patches.Circle((point.x, point.y), radius=.05, color='blue')
            self.maze_figure_ax.add_patch(new_crc)
        for point in self.goal_locations:
            new_crc = patches.Circle((point.x, point.y), radius=.05, color='red')
            self.maze_figure_ax.add_patch(new_crc)

        self.maze_figure_ax.set_ylim(-4.25, 4.25)
        self.maze_figure_ax.set_xlim(-4.25, 4.25)
        self.maze_figure_ax.margins(0.1)

    def close_maze_figure(self):
        plt.close(self.maze_figure)

    # Returns random starting positions
    def get_random_experiment_starting_position(self):
        return self.random_experiment_starting_position_generator.get_random_start_position()

    # Returns random starting positions
    def get_random_experiment_testing_starting_position(self):
        return self.random_testing_starting_position_generator.get_random_start_position()

    def get_random_habituation_starting_position(self):
        return self.random_habituation_starting_position_generator.get_random_start_position()

    # Creates a matplotlib plot of the maze
    def get_maze_figure(self):
        return self.maze_figure, self.maze_figure_ax

    def get_goal_location(self):
        return self.goal_locations[0].x, self.goal_locations[0].y


class MazePoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to_point(self, robot_position):
        return math.dist((self.x, self.y), (robot_position[0], robot_position[1])) - robot_position[2]


class Goal(MazePoint):
    def __init__(self, x, y, id):
        super().__init__(x, y)
        self.goal_id = id


class StartingPosition(MazePoint):
    def __init__(self, x, y, theta):
        super().__init__(x, y)
        self.theta = theta


class BoundaryWall:
    def __init__(self, x1, y1, x2, y2, height=0.3, width=0.012, id=0):
        self.end_point1 = MazePoint(x1, y1)
        self.end_point2 = MazePoint(x2, y2)
        self.height = height
        self.width = width
        self.length = math.dist((x1, y1), (x2, y2))
        self.id = id
        self.center_mass = MazePoint((self.end_point1.x + self.end_point2.x) / 2,
                                     (self.end_point1.y + self.end_point2.y) / 2)
        self.dimensions = [self.width, self.length, self.height]
        self.translation = [(self.end_point1.x + self.end_point2.x) / 2,
                            (self.end_point1.y + self.end_point2.y) / 2,
                            self.height / 2]
        theta = math.atan2(self.end_point1.x - self.end_point2.x, self.end_point1.y - self.end_point2.y)
        self.rotation = [0, 0, 1, theta]

    def get_webots_translation_string(self):
        txt = 'translation {x:.2f} {y:.2f} {z:.2f}'
        return txt.format(x=self.translation[0], y=self.translation[1], z=self.translation[2])

    def get_webots_rotation_string(self):
        txt = 'rotation {x:.2f} {y:.2f} {z:.2f} {theta:-.2f}'
        return txt.format(x=self.rotation[0], y=self.rotation[1], z=self.rotation[2], theta=self.rotation[3])

    def get_webots_size_string(self):
        txt = 'size {width:.2f} {length:.2f} {height:.2f}'
        return txt.format(width=self.width, length=self.length, height=self.height)

    def get_webots_node_string(self):
        node_string = "{translation} {rotation} {size}".format(translation=self.get_webots_translation_string(),
                                                               rotation=self.get_webots_rotation_string(),
                                                               size=self.get_webots_size_string())
        return 'DEF Boundary_Wall{id} Obstacle '.format(id=self.id) + '{ ' + node_string + ' }'


class Obstacle:
    def __init__(self, x1, y1, x2, y2, height=0.3, width=0.012, id=0):
        self.end_point1 = MazePoint(x1, y1)
        self.end_point2 = MazePoint(x2, y2)
        self.height = height
        self.width = width
        self.length = math.dist((x1, y1), (x2, y2))
        self.id = id
        self.center_mass = MazePoint((self.end_point1.x + self.end_point2.x) / 2,
                                     (self.end_point1.y + self.end_point2.y) / 2)
        self.dimensions = [self.width, self.length, self.height]
        self.translation = [(self.end_point1.x + self.end_point2.x) / 2,
                            (self.end_point1.y + self.end_point2.y) / 2,
                            self.height / 2]
        theta = math.atan2(self.end_point1.x - self.end_point2.x, self.end_point1.y - self.end_point2.y)
        self.rotation = [0, 0, 1, math.pi - theta]

    def get_webots_translation_string(self):
        txt = 'translation {x:.2f} {y:.2f} {z:.2f}'
        return txt.format(x=self.translation[0], y=self.translation[1], z=self.translation[2])

    def get_webots_rotation_string(self):
        txt = 'rotation {x:.2f} {y:.2f} {z:.2f} {theta:-.2f}'
        return txt.format(x=self.rotation[0], y=self.rotation[1], z=self.rotation[2], theta=self.rotation[3])

    def get_webots_size_string(self):
        txt = 'size {width:.2f} {length:.2f} {height:.2f}'
        return txt.format(width=self.width, length=self.length, height=self.height)

    def get_webots_node_string(self):
        node_string = "{translation} {rotation} {size}".format(translation=self.get_webots_translation_string(),
                                                               rotation=self.get_webots_rotation_string(),
                                                               size=self.get_webots_size_string())
        return 'DEF Obstacle_{id} Obstacle '.format(id=self.id) + '{ ' + node_string + ' }'

class CylinderLandmark:
    def __init__(self, x, y, height=1.5, radius=.25, color=[1, 1, 1], id=0):
        self.height = height
        self.radius = radius

        self.x = x
        self.y = y
        self.z = height/2
        self.translation = [x, y, self.z]
        self.id = id
        self.color = color

    def get_webots_translation_string(self):
        txt = 'translation {x:.2f} {y:.2f} {z:.2f}'
        return txt.format(x=self.translation[0], y=self.translation[1], z=self.translation[2])

    def get_webots_size_string(self):
        txt = 'size {width:.2f} {length:.2f} {height:.2f}'
        return txt.format(width=self.height, length=self.radius, height=self.radius-.01)

    def get_webots_color_string(self):
        txt = 'color {red:.2f} {green:.2f} {blue:.2f}'
        return txt.format(red=self.color[0], green=self.color[1], blue=self.color[2])
    def get_webots_recognition_color_string(self):
        txt = 'recognitionColors [{red:.2f} {green:.2f} {blue:.2f}]'
        return txt.format(red=self.color[0], green=self.color[1], blue=self.color[2])

    def get_webots_node_string(self):
        node_string = "{translation} {color} {recognitionColors} {size}".format(translation=self.get_webots_translation_string(),
                                                            color=self.get_webots_color_string(),
                                                            recognitionColors=self.get_webots_recognition_color_string(),
                                                            size=self.get_webots_size_string())
        return 'DEF Landmark_{id} Landmark '.format(id=self.id) + '{ ' + node_string + ' }'

class TagLandmark:
    def __init__(self, x, y, theta, tag_id, height, width, color=[1, 1, 1]):
        self.height = height
        self.width = width

        self.x = x
        self.y = y
        self.theta = theta

        self.z = self.height/2

        self.translation = [x, y, self.z]
        self.tag_id = tag_id
        self.color = color
        self.rotation = [0, 0, 1, theta]

    def get_webots_translation_string(self):
        txt = 'translation {x:.2f} {y:.2f} {z:.2f}'
        return txt.format(x=self.translation[0], y=self.translation[1], z=self.translation[2])

    def get_webots_rotation_string(self):
        txt = 'rotation {x:.2f} {y:.2f} {z:.2f} {theta:-.2f}'
        return txt.format(x=self.rotation[0], y=self.rotation[1], z=self.rotation[2], theta=self.rotation[3])

    def get_webots_size_string(self):
        txt = 'size {width:.2f} {height:.2f}'
        return txt.format(width=self.width, height=self.height)

    def get_webots_color_string(self):
        txt = 'color {red:.2f} {green:.2f} {blue:.2f}'
        return txt.format(red=self.color[0], green=self.color[1], blue=self.color[2])
    def get_webots_recognition_color_string(self):
        txt = 'recognitionColors [{red:.2f} {green:.2f} {blue:.2f}]'
        return txt.format(red=self.color[0], green=self.color[1], blue=self.color[2])

    def get_webots_tag_string(self):
        txt = 'signImage ["{path_to_tag_image}"]'
        return txt.format(path_to_tag_image='../protos/WorldObjects/LandMarkTags/landmark_tag_'+str(int(self.tag_id))+'.png')

    def get_webots_node_string(self):
        node_string = "{translation} {rotation} {color} {recognitionColors} {size} {signImage}".format(translation=self.get_webots_translation_string(),
                                                            rotation=self.get_webots_rotation_string(),
                                                            color=self.get_webots_color_string(),
                                                            recognitionColors=self.get_webots_recognition_color_string(),
                                                            size=self.get_webots_size_string(),
                                                            signImage = self.get_webots_tag_string())
        return 'DEF Tag_Landmark_{id} RectangularPanel '.format(id=int(self.tag_id)) + '{ ' + node_string + ' }'
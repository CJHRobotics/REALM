from realm_tools.simulation_lib.environment import Maze
from controller import Supervisor
from matplotlib import patches
import math

class HamBot(Supervisor):

    # Initiilize an instance of Webots Harrison's RosBot
    def __init__(self):

        # Inherent from Webots Robot Class: https://cyberbotics.com/doc/reference/robot
        self.experiment_supervisor = Supervisor()

        # Add a display to plot the place cells as they are generated
        self.pc_display = self.experiment_supervisor.getDevice('Place Cell Display')
        self.pc_display.setOpacity(1.0)

        # Sets Supervisor Root Nodes
        self.root_node = self.experiment_supervisor.getRoot()
        self.children_field = self.root_node.getField('children')
        self.robot_node = self.experiment_supervisor.getSelf()
        self.robot_translation_field = self.robot_node.getField('translation')
        self.robot_rotation_field = self.robot_node.getField('rotation')

        # Physical Robot Specifications
        self.wheel_radius = .045  # meters
        self.axel_length = .205  # meters
        self.robot_radius = .3086  # m

        # Define all systems and makes them class attributes
        self.timestep = int(self.experiment_supervisor.getBasicTimeStep())

        # Webots Rotational Motors: https://cyberbotics.com/doc/reference/motor
        self.left_motor = self.experiment_supervisor.getDevice('left motor')
        self.right_motor = self.experiment_supervisor.getDevice('right motor')
        self.all_motors = [self.left_motor, self.right_motor]
        self.max_motor_velocity = self.left_motor.getMaxVelocity()

        # Initialize robot's motors
        for motor in self.all_motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Webots Wheel Positional Sensors: https://www.cyberbotics.com/doc/reference/positionsensor
        self.left_encoder = self.experiment_supervisor.getDevice('left wheel encoder')
        self.right_encoder = self.experiment_supervisor.getDevice('right wheel encoder')
        self.all_encoders = [self.left_encoder, self.right_encoder]

        # Initialize robot's encoders
        for encoder in self.all_encoders:
            encoder.enable(self.timestep)


        # Webots Camera: https://cyberbotics.com/doc/reference/camera
        self.camera = self.experiment_supervisor.getDevice('camera')
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)

        # Webots RpLidarA2: https://www.cyberbotics.com/doc/guide/lidar-sensors#slamtec-rplidar-a2
        self.lidar = self.experiment_supervisor.getDevice('lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        # Webots IMU: https://www.cyberbotics.com/doc/guide/imu-sensors#mpu-9250
        # Webots IMU Accelerometer: https://www.cyberbotics.com/doc/reference/accelerometer
        self.imu = self.experiment_supervisor.getDevice('imu')
        self.imu.enable(self.timestep)

        # Webots GPS:
        self.gps = self.experiment_supervisor.getDevice('gps')
        self.gps.enable(self.timestep)

        self.sensor_calibration()

    # Preforms one timestep to update all sensors should be used when initializing robot and after teleport
    def sensor_calibration(self,stop_num =1):
        counter = 0
        while self.experiment_supervisor.step(self.timestep) != -1:
            counter += 1
            if counter > stop_num:
                break

    def get_robot_pose(self):
        while self.experiment_supervisor.step(self.timestep) != -1:
            current_x, current_y, current_z = self.robot_translation_field.getSFVec3f()
            break
        return current_x, current_y, self.get_bearing()

    # Reads the robot's IMU compass and return bearing in degrees
    #   North   -> 90
    #   East    -> 0 or 360
    #   South   -> 270
    #   West    -> 180
    def get_bearing(self):
        compass_reading = self.imu.getRollPitchYaw()
        bearing = math.degrees(compass_reading[-1])
        if bearing < 0.0:
            bearing += 360.0
        return round(bearing)

    def get_min_lidar_reading(self):
        self.sensor_calibration()
        return min(self.lidar.getRangeImage())

    # Reads current encoder readings and return an array of encoder positions:
    #   [left, right]
    def get_encoder_readings(self):
        return [readings.getValue() for readings in self.all_encoders]

    # Sets Front Left Motor Velocity (rad/sec)
    def set_left_motor_velocity(self, velocity, suppress=False):
        self.left_motor.setVelocity(self.velocity_saturation(velocity, suppress=suppress))

    # Sets Front Right Motor Velocity (rad/sec)
    def set_right_motor_velocity(self, velocity, suppress=False):
        self.right_motor.setVelocity(self.velocity_saturation(velocity, suppress=suppress))

    # Sets all motors speed to 0
    def stop(self):
        for motor in self.all_motors:
            motor.setVelocity(0)

    def slow_stop(self):
        while self.experiment_supervisor.step(self.timestep) != -1:
            current_velocity = self.left_motor.getVelocity()
            if current_velocity > .1:
                for motor in self.all_motors:
                    motor.setVelocity(current_velocity / 4)
            else:
                self.stop()
                break

    # Sets all motors speed to velocity
    def go_forward(self, velocity=1):
        for motor in self.all_motors:
            motor.setVelocity(self.velocity_saturation(velocity))

    # Gets Current Front Left Motor Encoder Reading (meters)
    def get_left_motor_encoder_reading(self):
        return self.left_encoder.getValue()

    # Gets Current Front Right Motor Encoder Reading (meters)
    def get_right_motor_encoder_reading(self):
        return self.right_encoder.getValue()

    # Get Lidar Range Image (meters)
    def get_lidar_range_image(self):
        return self.lidar.getRangeImage()




    # Supervisor Functions: allows robot to control the simulation
    # DO NOT MODIFY: unless you are attempting to manipulate the webots world simulations!!!

    # Takes in a xml maze file and creates the walls, starting locations, and goal locations
    def load_environment(self, maze_file):
        self.maze = Maze(maze_file, display_width=self.pc_display.getWidth(),
                         display_height=self.pc_display.getHeight())
        self.pc_figure, self.pc_figure_ax = self.maze.get_maze_figure()
        self.pc_figure.savefig('data/DataCache/temp.png')

        while self.experiment_supervisor.step(self.timestep) != -1:
            ir = self.pc_display.imageLoad('data/DataCache/temp.png')
            self.pc_display.imagePaste(ir, 0, 0, True)
            break

        self.obstical_nodes = []
        self.boundry_wall_nodes = []
        self.cylinder_landmark_nodes = []
        self.tag_landmark_nodes = []

        for obstacles in self.maze.obstacles:
            self.children_field.importMFNodeFromString(-1, obstacles.get_webots_node_string())
            self.obstical_nodes.append(self.experiment_supervisor.getFromDef('Obstacle'))
        for boundary_wall in self.maze.boundary_walls:
            self.children_field.importMFNodeFromString(-1, boundary_wall.get_webots_node_string())
            self.boundry_wall_nodes.append(self.experiment_supervisor.getFromDef('Obstacle'))
        for cylinder_landmark in self.maze.cylinder_landmarks:
            self.children_field.importMFNodeFromString(-1, cylinder_landmark.get_webots_node_string())
            self.cylinder_landmark_nodes.append(self.experiment_supervisor.getFromDef('Landmark'))
        for tag_landmark in self.maze.tag_landmarks:
            self.children_field.importMFNodeFromString(-1, tag_landmark.get_webots_node_string())
            self.tag_landmark_nodes.append(self.experiment_supervisor.getFromDef('RectangularPanel'))

    def reset_environment(self):
        self.teleport_robot(theta=math.pi / 2)
        total_nodes = len(self.obstical_nodes) + len(self.boundry_wall_nodes) + len(self.landmark_nodes)
        for i in range(total_nodes):
            self.children_field.removeMF(-1)
        self.maze.close_maze_figure()
        self.sensor_calibration()

    # Teleports the robot to the point (x,y,z)
    def teleport_robot(self, x=0.0, y=0.0, z=0.09, theta=math.pi):
        self.robot_translation_field.setSFVec3f([x, y, z])
        self.robot_rotation_field.setSFRotation([0, 0, 1, theta])
        self.sensor_calibration()

    def move_to_training_start(self):
        starting_position = self.maze.experiment_starting_location[0]
        self.teleport_robot(starting_position.x, starting_position.y, theta=starting_position.theta)

    # Moves the robot to a random starting position
    def move_to_testing_start(self, index=-1):
        if index == -1:
            starting_position = self.maze.get_random_experiment_testing_starting_position()
        else:
            starting_position = self.maze.experiment_starting_location[index]
        self.teleport_robot(starting_position.x, starting_position.y, theta=starting_position.theta)
        return self.get_robot_pose()

    # Moves the robot to a random starting position
    def move_to_random_experiment_start(self):
        starting_position = self.maze.get_random_experiment_starting_position()
        self.teleport_robot(starting_position.x, starting_position.y, theta=starting_position.theta)
        return self.get_robot_pose()

    # Moves the robot to a random starting position
    def move_to_habituation_start(self, index=-1):
        if index == -1:
            starting_position = self.maze.get_random_habituation_starting_position()
        else:
            starting_position = self.maze.habituation_start_location[index]
        self.teleport_robot(starting_position.x, starting_position.y, theta=starting_position.theta)
        return self.get_robot_pose()

    def check_at_goal(self):
        current_x, current_y, currentcurrent_z = self.robot_translation_field.getSFVec3f()
        goal_x, goal_y = self.maze.get_goal_location()
        distance_to_goal = math.sqrt((current_x - goal_x) ** 2 + (current_y - goal_y) ** 2)
        return distance_to_goal < 0.8

    def get_dist_to_goal(self):
        current_x, current_y, currentcurrent_z = self.robot_translation_field.getSFVec3f()
        goal_x, goal_y = self.maze.get_goal_location()
        distance_to_goal = math.sqrt((current_x - goal_x) ** 2 + (current_y - goal_y) ** 2)
        return distance_to_goal

    def show_loaded_pc_network(self, pc_network):
        for pc in pc_network.pc_list:
            self.update_pc_display(pc)

    # Plots Place cells and shows them on the Display
    def update_pc_display(self, place_cell):
        new_pc = patches.Circle((place_cell.center_x, place_cell.center_y), radius=place_cell.radius, fill=False)
        self.pc_figure_ax.add_patch(new_pc)
        self.pc_figure_ax.set_ylim(-4.25, 4.25)
        self.pc_figure_ax.set_xlim(-4.25, 4.25)
        self.pc_figure.savefig('data/DataCache/temp.png')
        while self.experiment_supervisor.step(self.timestep) != -1:
            ir = self.pc_display.imageLoad('data/DataCache/temp.png')
            self.pc_display.imagePaste(ir, 0, 0, True)
            break

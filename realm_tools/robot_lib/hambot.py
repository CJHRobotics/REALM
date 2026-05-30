from realm_tools.simulation_lib.environment import Environment
from controller import Supervisor, Display
from PIL import Image
import math

class HamBot(Supervisor):

    # Initiilize an instance of HamBot
    def __init__(self):

        # Inherent from Webots Robot Class: https://cyberbotics.com/doc/reference/robot
        self.experiment_supervisor = Supervisor()

        # Add a display to plot the place cells as they are generated
        self.robot_display = self.experiment_supervisor.getDevice('Robot Display')
        self.robot_display.setOpacity(1.0)

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
        self.update_robot_display()

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
        return current_x, current_y, self.get_compass_reading()

    # Reads the robot's IMU compass and return bearing in degrees
    #   North   -> 90
    #   East    -> 0 or 360
    #   South   -> 270
    #   West    -> 180
    def get_compass_reading(self):
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

    def sat(self, velocity):
        return max(-self.max_motor_velocity, min(self.max_motor_velocity, velocity))

    # Sets Front Left Motor Velocity (rad/sec)
    def set_left_motor_velocity(self, velocity):
        self.left_motor.setVelocity(self.sat(velocity))

    # Sets Front Right Motor Velocity (rad/sec)
    def set_right_motor_velocity(self, velocity):
        self.right_motor.setVelocity(self.sat(velocity))

    # Sets all motors speed to 0
    def stop(self):
        for motor in self.all_motors:
            motor.setVelocity(0)

    # Sets all motors speed to velocity
    def go_forward(self, velocity=1):
        for motor in self.all_motors:
            motor.setVelocity(self.sat(velocity))

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
    def load_environment(self, environment_file, display=False):
        self.maze = Environment(environment_file, display_width=self.robot_display.getWidth(),
                                display_height=self.robot_display.getHeight())
        if display:
            self.maze_figure, self.maze_figure_ax = self.maze.get_environment_figure()
            self.maze_figure.savefig('data/data_cache/maze.png')
            self.update_robot_display(name='maze')

        self.environment_nodes = []

        for wall in self.maze.boundary_walls + self.maze.obstacles:
            self.children_field.importMFNodeFromString(-1, wall.get_webots_node_string())
            self.environment_nodes.append(self.experiment_supervisor.getFromDef(f'{wall.wall_type}_{wall.id}'))

        for landmark in self.maze.landmarks:
            self.children_field.importMFNodeFromString(-1, landmark.get_webots_node_string())
            self.environment_nodes.append(self.experiment_supervisor.getFromDef(f'{landmark.landmark_type}_{landmark.id}'))

    def reset_environment(self):
        self.teleport_robot(theta=math.pi / 2)
        for _ in range(len(self.environment_nodes)):
            self.children_field.removeMF(-1)
        self.environment_nodes = []
        self.maze.close_environment_figure()
        self.sensor_calibration()

    # Teleports the robot to the point (x,y,z)
    def teleport_robot(self, x=0.0, y=0.0, z=0.09, theta=math.pi):
        self.robot_translation_field.setSFVec3f([x, y, z])
        self.robot_rotation_field.setSFRotation([0, 0, 1, theta])
        self.robot_node.resetPhysics()
        self.sensor_calibration()

    def move_to_start(self, mode='testing', index=0):
        if mode == 'training':
            locations = self.maze.training_starting_location
            get_random = self.maze.get_random_training_starting_position
        else:
            if mode != 'testing':
                print(f"Warning: Invalid mode '{mode}', defaulting to 'testing'.")
            locations = self.maze.testing_start_location
            get_random = self.maze.get_random_testing_starting_position

        if index == -1 or not (0 <= index < len(locations)):
            if index != -1:
                print(f"Warning: Index {index} out of range for {mode} starts (0-{len(locations) - 1}), using random.")
            starting_position = get_random()
        else:
            starting_position = locations[index]

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

    # Plots Place cells and shows them on the Display
    def update_robot_display(self, name='default'):
        display_width = self.robot_display.getWidth()
        display_height = self.robot_display.getHeight()
        img = Image.open('data/data_cache/' + name + '.png').convert('RGB')
        img = img.resize((display_width, display_height))
        data = img.tobytes()
        while self.experiment_supervisor.step(self.timestep) != -1:
            ir = self.robot_display.imageNew(data, Display.RGB, display_width, display_height)
            self.robot_display.imagePaste(ir, 0, 0, False)
            self.robot_display.imageDelete(ir)
            break

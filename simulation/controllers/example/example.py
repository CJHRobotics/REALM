import os
os.chdir("../../..")

"""ExperimentSupervisor controller."""
from realm_tools.robot_lib.hambot import HamBot


maze_file = 'simulation/worlds/mazes/samples/example.xml'

# create the robot/supervisor instance.
robot = HamBot()

# Loads the environment from the maze file
robot.load_environment(maze_file)

# Show basic robot/supervisor functions
robot.move_to_start()

start_time = robot.experiment_supervisor.getTime()

# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:

    print("Max rotational motor velocity: ", robot.max_motor_velocity)


    # Reads and Prints Robot's Encoder Readings
    print("Motor Encoder Readings: ", robot.get_encoder_readings())

    # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    print("Lidar Front Reading", robot.get_lidar_range_image()[180])
    print("Lidar Left Reading", robot.get_lidar_range_image()[90])
    print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
    print("Lidar Right Reading", robot.get_lidar_range_image()[270])
    print("Simulation Time", robot.experiment_supervisor.getTime())

    # Sets the robot's motor velocity to 18 rad/sec
    robot.set_right_motor_velocity(18)
    robot.set_left_motor_velocity(18)

    # Calculates distance the wheel has turned since beginning of simulation
    distance_left_wheel_traveled = robot.wheel_radius * robot.get_left_motor_encoder_reading()
    travel_time = robot.experiment_supervisor.getTime() - start_time

    # Stops the robot after the robot moves a distance of 1.5 meters
    if distance_left_wheel_traveled > 1.5:
        print(f"Time to travel 1.5 meters:{travel_time:.2f}")
        robot.stop()
        break
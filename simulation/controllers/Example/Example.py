import os
os.chdir("../../..")

"""ExperimentSupervisor controller."""
from realm_tools.robot_lib.hambot import HamBot


maze_file = 'simulation/worlds/mazes/Samples/LandmarkExample.xml'

# create the robot/supervisor instance.
robot = HamBot()

# Loads the environment from the maze file
robot.load_environment(maze_file)

# Show basic robot/supervisor functions
robot.move_to_random_experiment_start()


for i in range(8):
    robot.perform_action_with_PID(i)
    robot_x, robot_y, robot_theta = robot.get_robot_pose()


robot.experiment_supervisor.simulationReset()



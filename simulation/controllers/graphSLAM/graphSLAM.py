import os

from realm_tools.robot_lib.graph_slam_calculations import *
os.chdir("../../..")


from realm_tools.robot_lib.my_robot import MyRobot
from realm_tools.robot_lib.graphSLAM_data_collection import *

robot = MyRobot()
robot.load_environment('simulation/worlds/environments/samples/octagon.xml')
#starting position
robot.teleport_robot(0,0,0, 0)

# initialize (t=0)
action_gen = None
action_phase = 'rotate'
cycles_completed = 0
max_cycles = 5

landmark_log = []
odometry_log = []
truth_log = []

prev_encoder_readings = robot.get_encoder_readings()

t=0

while robot.experiment_supervisor.step(robot.timestep) != -1:

    # measure (distance to landmarks and odometry)
    ''' 
        finds landmarks and returns relative distance to them
        using get_landmark_observations()
    '''

    #landmark collection
    landmark_scan = get_landmark_row(robot)
    landmark_log.append(landmark_scan)

    #odometry collection
    dt = robot.timestep / 1000.0  # ms to seconds

    v, omega, prev_encoder_readings = get_odometry(robot,prev_encoder_readings, dt)
    odometry_log.append((v, omega, dt))

    #ground truth collection
    x, y, _ = robot.robot_translation_field.getSFVec3f()
    theta = robot.get_compass_reading()
    truth_log.append((x, y, theta))


    # move ()
    # Start a new generator if none is currently running
    if action_gen is None:
        if action_phase == 'rotate':
            action_gen = rotate_step(robot, 72)
        elif action_phase == 'move':
            action_gen = move_forward_step(robot, .5)

    try:
        next(action_gen)
    except StopIteration:
        action_gen = None
        if action_phase == 'rotate':
            action_phase = 'move'
        else:
            action_phase = 'rotate'
            cycles_completed += 1
            if cycles_completed >= max_cycles:
                break  # exit the main loop after 5 full rotate+move cycles

    print(f"Completed {cycles_completed} cycles, {len(landmark_log)} timesteps recorded")



#graphSLAM calculations

true_landmark_positions = [(lm.x, lm.y) for lm in robot.maze.landmarks]
mu_poses, mu_landmarks = graphSLAM_init(odometry_log, landmark_log, x0=(0.0, 0.0, 0.0))
print_landmark_triangulation_check(mu_landmarks, true_landmark_positions)

print_landmark_observation_spread(mu_poses, landmark_log)  # mu_poses from graphSLAM_init
mu_poses_final, mu_landmarks_final = graphSLAM_run(odometry_log, landmark_log, x0=(0.0, 0.0, 0.0))
print_landmark_triangulation_check(mu_landmarks_final, true_landmark_positions)
dead_reckoned_poses, _ = graphSLAM_init(odometry_log, landmark_log, x0=(0.0, 0.0, 0.0))
print_graphSLAM_accuracy(mu_poses_final, dead_reckoned_poses, truth_log)

print_worst_pose_errors(mu_poses_final, truth_log, odometry_log, top_n=10)

mu_poses_optimized, mu_landmarks_optimized = graphSLAM_run(odometry_log, landmark_log, x0=(0.0, 0.0, 0.0))
print_landmark_triangulation_check(mu_landmarks_optimized, true_landmark_positions)
print_landmark_observation_spread(mu_poses_optimized, landmark_log)



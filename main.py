from dataset import BasicCross
from basic_fun import Transform
import matplotlib.pyplot as plt
import numpy as np
from robots import Robot
from data_struct import Pose

# Press the green button in the gutter to run the script.
TIME_DURATION = 1000
TIME_STEP = 0.02
SHOW_ANIMATION = True
PLOT_WINDOW_SIZE_X = 12
PLOT_WINDOW_SIZE_Y = 17
PLOT_FONT_SIZE = 8

simulation_running = True
all_robots_are_at_target = False

def GenerateRefPath(a_begin, a_end, b_begin, b_end, num):
    cross = BasicCross(a_begin, a_end, b_begin, b_end, num)
    # pathA_ref_x, pathA_ref_y, pathB_ref_x, pathB_ref_y = cross.GetRefPathsPlot()
    # plt.plot(pathA_ref_x, pathA_ref_y)
    # plt.plot(pathB_ref_x, pathB_ref_y)
    # plt.show()
    return cross.GetRefPaths()

def Test():
    x0 = 1.0
    y0 = 1.0
    theta = 3.1415926 / 2.0
    x = 1.0
    y = 2.0
    x_, y_ = Transform.Global2Local(x, y, x0, y0, theta)
    print(x_, y_)

# if __name__ == '__main__':
#     GenerateRefPath(-20.0, 20.0, -30.0, 30.0, 100)
#     #Test()

def run_simulation(robots):
    """Simulates all robots simultaneously"""
    global all_robots_are_at_target
    global simulation_running

    robot_names = []
    for instance in robots:
        robot_names.append(instance.name)

    time = 0
    while simulation_running and time < TIME_DURATION:
        time += TIME_STEP
        robots_are_at_target = []

        for instance in robots:
            if not instance.is_at_target:
                instance.Move(TIME_STEP)
            robots_are_at_target.append(instance.is_at_target)

        if all(robots_are_at_target):
            simulation_running = False

        if SHOW_ANIMATION:
            plt.cla()
            plt.xlim(-PLOT_WINDOW_SIZE_X, PLOT_WINDOW_SIZE_X)
            plt.ylim(-PLOT_WINDOW_SIZE_Y, PLOT_WINDOW_SIZE_Y)

            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 1,
                     'Time: {:.2f}'.format(time),
                     fontsize=PLOT_FONT_SIZE)

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 2,
                     'Reached target: {} = '.format(robot_names)
                     + str(robots_are_at_target),
                     fontsize=PLOT_FONT_SIZE)

            for instance in robots:
                plt.arrow(instance.pose_start.x,
                          instance.pose_start.y,
                          np.cos(instance.pose_start.theta),
                          np.sin(instance.pose_start.theta),
                          color='r',
                          width=0.1)
                plt.arrow(instance.pose_target.x,
                          instance.pose_target.y,
                          np.cos(instance.pose_target.theta),
                          np.sin(instance.pose_target.theta),
                          color='g',
                          width=0.1)
                plot_vehicle(instance.pose_current.x,
                             instance.pose_current.y,
                             instance.pose_current.theta,
                             instance.x_traj,
                             instance.y_traj, instance.color, instance.x_pos_list, instance.y_pos_list)
                print("name = %s, color = %c" % (instance.name, instance.color))
            plt.pause(TIME_STEP)


def plot_vehicle(x, y, theta, x_traj, y_traj, color, x_pos, y_pos):
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = T @ p1_i
    p2 = T @ p2_i
    p3 = T @ p3_i

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color+'-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color+'-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color+'-')
    plt.plot(x_traj, y_traj, color+'-')
    plt.plot(x_pos, y_pos, 'b--')


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def main():
    pathA, pathB = GenerateRefPath(-10.0, 10.0, -15.0, 15.0, 100)
    pose_target_ego = Pose(10.0, 0.0, 0)
    pose_start_ego = Pose(-10.0, 2.0, 0)
    pose_target_obj1 = Pose(0.0, 15.0, np.pi / 2.0)
    pose_start_obj1 = Pose(0.0, -15.0, np.pi / 2.0)

    robot_1 = Robot("Ego", "y", 10.0, 5.0, pose_start_ego, pose_target_ego, pathA)
    robot_2 = Robot("Obj1", "r", 10.0, 5.0, pose_start_obj1, pose_target_obj1, pathB)

    robots: list[Robot] = [robot_1, robot_2]

    run_simulation(robots)


if __name__ == '__main__':
    main()



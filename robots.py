import numpy as np
import math
from data_struct import PathPoint
from basic_fun import Transform
import copy

AT_TARGET_ACCEPTANCE_THRESHOLD = 1.0
SPD_PROFILE_DT = 0.1
SPEED_LIMIT = 10.0
ACC_LIMIT = 5.0
DCC_LIMIT = -5.0
MAX_ANGLE_ACC = 1.0
MAX_VAL = 1E6
MAX_SUM = 10.0
MAX_D_ERR = 10.0

class Robot:
    def __init__(self, name, color, max_linear_speed, max_angular_speed, pose_start, pose_target, ref_path):
        self.name = name
        self.color = color
        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ANGULAR_SPEED = max_angular_speed
        self.ref_path = ref_path
        self.pose_target = pose_target
        self.pose_start = copy.copy(pose_start)
        self.x_traj = [] # path plan
        self.y_traj = []
        self.x_pos_list = [] # pos after control
        self.y_pos_list = []
        # current status
        self.pose_current = copy.copy(pose_start)
        self.vec = 0.0 # current velocity
        self.angle_vec = 0.0 # current angle velocity
        self.foresee_time = 1.0 # s, look forward time in controller
        self.is_at_target = False
        # used in motion plan
        self.max_to_stop_time = SPEED_LIMIT / math.fabs(DCC_LIMIT)  # the time it takes from max speed to stop
        # used in control
        self.lateral_kp = 0.2
        self.lateral_ki = 0.01
        self.lateral_kd = 50.0
        self.error_sum = 0.0  # for ki
        self.last_error = 0.0 # for kd
        self.vec_kp = 0.5
        self.vec_target_ratio = 0.5

    def SetStartTargetPoses(self, pose_start, pose_target):
        self.pose_start = copy.copy(pose_start)
        self.pose_target = pose_target

    def Control(self, path, speed_profile, pose_current):
        point_match = PathPoint()
        foresee_dist = self.foresee_time * self.vec
        dist_sum = 0.0
        speed_match = self.vec
        # look for match point
        min_dist_to_ego = MAX_VAL
        ego_index = -1
        for i in range(len(path)):
            dist_to_ego = np.hypot(path[i].x - self.pose_current.x, path[i].y - self.pose_current.y)
            if dist_to_ego < min_dist_to_ego:
                min_dist_to_ego = dist_to_ego
                ego_index = i

        for i in range(ego_index, len(path)):
            if i == 0:
                continue
            dist_temp = np.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y)
            dist_sum += dist_temp
            if (dist_sum >= foresee_dist or i == len(path) - 1):
                point_match.x, point_match.y = Transform.Global2Local(path[i].x, path[i].y, pose_current.x,
                                                                      pose_current.y, pose_current.theta)
                break
        # delay in vehicle model is not considered
        self.error_sum += point_match.y
        print("err = %lf, err_sum = %lf" % (point_match.y, self.error_sum))
        self.error_sum = max(-MAX_SUM, min(MAX_SUM, self.error_sum))
        if (self.last_error * point_match.y <= 0.0):  # when cross 0, sum should decrease
            self.error_sum = 0.0

        d_error = point_match.y - self.last_error
        d_error = max(-MAX_D_ERR, min(MAX_D_ERR, d_error))
        self.last_error = point_match.y

        angle_vec = self.lateral_kp * point_match.y + self.lateral_ki * self.error_sum + self.lateral_kd * d_error
        angle_vec = max(-MAX_ANGLE_ACC, min(MAX_ANGLE_ACC, angle_vec))

        print("name = %s, error = %lf, error_sum = %lf, d_error = %lf" % \
            (self.name, point_match.y, self.error_sum, d_error))
        print("p action = %lf, i action = %lf, d action = %lf" % (self.lateral_kp * point_match.y, \
            self.lateral_ki * self.error_sum, self.lateral_kd * d_error))

        if bool(speed_profile): #not empty
            index = int(self.foresee_time / SPD_PROFILE_DT)
            if speed_profile[index]:
                speed_match = speed_profile.get(index)
            else:
                speed_match = speed_profile[0][0]
        vec = self.vec + self.vec_kp * (speed_match - self.vec)
        return (vec, angle_vec)

    def PathPlan(self, ref_path):
        self.x_traj.clear()
        self.y_traj.clear()
        for pt in ref_path:
            self.x_traj.append(pt.x)
            self.y_traj.append(pt.y)
        return ref_path

    def SpeedPlan(self, path, pose_target):
        speed_profile = dict()
        # A simple speed plan

        if self.is_at_target:
            speed_profile[0.1] = 0.0
        else:
            min_dist = MAX_VAL
            match_index = 0
            for i in range(len(path)):
                dist = np.hypot(path[i].x - self.pose_current.x, path[i].y - self.pose_current.y)
                if dist < min_dist:
                    min_dist = dist
                    match_index = i

            dist_to_final = 0.0
            for i in range(match_index, len(path)):
                dist_temp = 0.0

                if i > 0:
                    dist_temp = np.hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y)
                    dist_to_final += dist_temp
            # if (dist_to_final > self.vec * self.vec / (2 * math.fabs(DCC_LIMIT))) and \
            #     (dist_to_final < self.vec * self.vec / (2 * math.fabs(DCC_LIMIT)) + 5.0):
            for i in range(20):
                speed_profile[i] = 5.0

        return speed_profile

    def MotionPlan(self, ref_path, pose_target):
        path = self.PathPlan(ref_path)
        speed_profile = self.SpeedPlan(path, pose_target)
        return (path, speed_profile)

    def Move(self, dt):
        path, speed_profile = self.MotionPlan(self.ref_path, self.pose_target)
        vec, angle_vec = self.Control(path, speed_profile, self.pose_current)

        if np.hypot(self.pose_target.x - self.pose_current.x, self.pose_target.y - self.pose_current.y) < \
                AT_TARGET_ACCEPTANCE_THRESHOLD:
            self.is_at_target = True

        '''
        if abs(linear_velocity) > self.MAX_LINEAR_SPEED:
            linear_velocity = (np.sign(linear_velocity)
                               * self.MAX_LINEAR_SPEED)

        if abs(angular_velocity) > self.MAX_ANGULAR_SPEED:
            angular_velocity = (np.sign(angular_velocity)
                                * self.MAX_ANGULAR_SPEED)
        '''
        self.pose_current.theta = self.pose_current.theta + angle_vec * dt
        self.pose_current.x = self.pose_current.x + vec * np.cos(self.pose_current.theta) * dt
        self.pose_current.y = self.pose_current.y + vec * np.sin(self.pose_current.theta) * dt
        self.x_pos_list.append(self.pose_current.x)
        self.y_pos_list.append(self.pose_current.y)
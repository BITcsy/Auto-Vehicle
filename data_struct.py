class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class PathPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.leftwidth = 2.0
        self.rightwidth = 2.0

class SpeedProflie:
    def __init__(self):
        self.speed_profile = dict()
        self.max_time = 5.0

class TrajPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t = 0.0
        self.veclocity = 0.0
        self.leftwidth = 2.0
        self.rightwidth = 2.0

class VehicleSize:
    def __init__(self):
        # f: front, b: back, l: left, r:right. x-axis: front, y-axis: left
        self.fl_pt = Pose(4.5, 1.0, 0.0)
        self.fr_pt = Pose(4.5, -1.0, 0.0)
        self.bl_pt = Pose(-1.3, 1.0, 0.0)
        self.br_pt = Pose(-1.3, -1.0, 0.0)



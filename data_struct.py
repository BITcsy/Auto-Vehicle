class SimplePoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class PathPoint:
    x = 0.0
    y = 0.0
    theta = 0.0
    leftwidth = 1.0
    rightwidth = 1.0

class TrajPoint:
    x = 0.0
    y = 0.0
    theta = 0.0
    t = 0.0
    leftwidth = 1.0
    rightwidth = 1.0

class VehicleSize:
    # f: front, b: back, l: left, r:right. x-axis: front, y-axis: left
    fl_pt = SimplePoint(4.5, 1.0)
    fr_pt = SimplePoint(4.5, -1.0)
    bl_pt = SimplePoint(-1.3, 1.0)
    br_pt = SimplePoint(-1.3, -1.0)



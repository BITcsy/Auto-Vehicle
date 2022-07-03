from math import cos, sin

class Transform:
    @classmethod
    def Global2Local(self, global_x, global_y, x0, y0, theta):
        local_x = global_x * cos(theta) + global_y * sin(theta) - x0 * cos(theta) - y0 * sin(theta)
        local_y = -global_x * sin(theta) + global_y * cos(theta) + x0 * sin(theta) - y0 * cos(theta)
        return (local_x, local_y)

    @classmethod
    def Local2Global(self, local_x, local_y, x0, y0, theta):
        global_x = local_x * cos(theta) - local_y * sin(theta) + x0
        global_y = local_x * sin(theta) + local_y * cos(theta) + y0
        return (global_x, global_y)

    @classmethod
    def Cartisian2Frenet(self):
        return

    @classmethod
    def Frenet2Cartisian(self):
        return
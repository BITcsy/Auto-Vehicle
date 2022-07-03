from data_struct import PathPoint

class BasicCross:
    def __init__(self, a_begin, a_end, b_begin, b_end, num):
        self.a_begin = a_begin
        self.a_end = a_end
        self.b_begin = b_begin
        self.b_end = b_end
        self.num = num
        self.reso_A = (a_end - a_begin) / num
        self.reso_B = (b_end - b_begin) / num
        self.carA_path = []
        self.carB_path = []
        for i in range(self.num):
            pt_A = PathPoint()
            pt_B = PathPoint()
            pt_A.x = self.a_begin + self.reso_A * float(i)
            pt_B.y = self.b_begin + self.reso_B * float(i)
            self.carA_path.append(pt_A)
            self.carB_path.append(pt_B)

    def GetRefPaths(self):
        refA_pts_x = []
        refA_pts_y = []
        refB_pts_x = []
        refB_pts_y = []
        for i in range(len(self.carA_path)):
            refA_pts_x.append(self.carA_path[i].x)
            refA_pts_y.append(self.carA_path[i].y)
        for i in range(len(self.carB_path)):
            refB_pts_x.append(self.carB_path[i].x)
            refB_pts_y.append(self.carB_path[i].y)
        return (refA_pts_x, refA_pts_y, refB_pts_x, refB_pts_y)

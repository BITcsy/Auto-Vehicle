from dataset import BasicCross
from basic_fun import Transform
import matplotlib.pyplot as plt

# Press the green button in the gutter to run the script.

def GenerateRefPath(a_begin, a_end, b_begin, b_end, num):
    cross = BasicCross(a_begin, a_end, b_begin, b_end, num)
    pathA_ref_x, pathA_ref_y, pathB_ref_x, pathB_ref_y = cross.GetRefPaths()
    plt.plot(pathA_ref_x, pathA_ref_y)
    plt.plot(pathB_ref_x, pathB_ref_y)
    plt.show()

def Test():
    x0 = 1.0
    y0 = 1.0
    theta = 3.1415926 / 2.0
    x = 1.0
    y = 2.0
    x_, y_ = Transform.Global2Local(x, y, x0, y0, theta)
    print(x_, y_)

if __name__ == '__main__':
    GenerateRefPath(-20.0, 20.0, -30.0, 30.0, 100)
    #Test()
# See PyCharm help at https://www.jetbrains.com/help/pycharm/

# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import math
from matplotlib import pyplot as plt


def calculate(path_plan):
    x = []
    y = []
    for a in path_plan:
        t = a[0]
        vr = a[1]
        vl = a[2]
        xo = a[3]
        yo = a[4]
        thetao = a[5]
        l = 0.160
        count = 0.1
        while count <= t:
            if vr != vl:
                x.append(xo + (l*(vr+vl) / (2 * (vr-vl)))* (math.sin(((vr-vl)*count) / l + thetao)-math.sin(thetao)))
                y.append(yo + (l * (vr + vl) / (2 * (vr - vl))) * (math.cos(((vr - vl) * count) / l + thetao) - math.cos(thetao)))
            else:
                x.append(vr*count * math.cos(thetao))
                y.append(vr * count * math.sin(thetao))



            count += 0.1
    plt.title("Path of Robot Problem 3")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.plot(x,y)
    plt.show()
    #




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    path_plan = [
        [15.3, 0.188, 0.22, 0, 0, 1.5709],
        [15.3, 0.22, 0.188, 2, 0, -1.5709],
        [9.1, 0.22, 0.22, 4, 0, 1.5709],
        [26.5, 0.15, 0.15, 4, 2, -3.14159],
        [9.1, 0.22, 0.22, 0, 2, -1.5709]
    ]

    calculate(path_plan)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/

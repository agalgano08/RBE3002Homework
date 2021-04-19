# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import math
from matplotlib import pyplot as plt


def calculate(path_plan):
    x3 = []
    y3 = []
    x4 = []
    y4 = []
    e = []

    time = []
    timer = 0
    for a in path_plan:
        t = a[0]
        vr = a[1]
        vl = a[2]
        vl4 = a[2] - 0.04
        xo = a[3]
        yo = a[4]
        thetao = a[5]
        l = 0.160
        count = 0.1

        while count <= t:
            if vr != vl:
                x3.append(xo + (l*(vr+vl) / (2 * (vr-vl)))* (math.sin(((vr-vl)*count) / l + thetao)-math.sin(thetao)))
                y3.append(yo + (l * (vr + vl) / (2 * (vr - vl))) * (math.cos(((vr - vl) * count) / l + thetao) - math.cos(thetao)))
            else:
                x3.append(vr*count * math.cos(thetao))
                y3.append(vr * count * math.sin(thetao))

            if vr != vl4:
                x4.append(xo + (l * (vr + vl4) / (2 * (vr - vl4))) * (math.sin(((vr - vl4) * count) / l + thetao) - math.sin(thetao)))
                y4.append(yo + (l * (vr + vl4) / (2 * (vr - vl4))) * (math.cos(((vr - vl4) * count) / l + thetao) - math.cos(thetao)))
            else:
                x4.append(vr*count * math.cos(thetao))
                y4.append(vr * count * math.sin(thetao))

            count += 0.1
            time.append(timer)
            timer += 0.1
    i = 0
    while i < len(x3):
        e.append(math.sqrt((x3[i] - x4[i])**2 + (y3[i] - y4[i])**2))
        i +=1

    plt.title("Error of Euclidean distance from Problem 3 and 4")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.plot(time,e)
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

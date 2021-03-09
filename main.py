
from math import cos,sin, pi

import numpy as np
import matplotlib.pyplot as plt


def data_reader():
    N = int(input())
    data = []
    for i in range(N):
        l, r = map(float, input().replace(',', '.').split(" "))
        data.append([l, r])
    return data


def coordinates_counter(odometry_data):
    R = 28  # радиус колеса, мм
    L = 170  # ширина колеи робота, мм
    N = len(odometry_data)
    x_coordinate = 0
    y_coordinate = 0
    d_angle = 0
    angle =0
    coordinates = np.zeros((N, 2))
    global_coordinates = np.array([[0],
                                   [0]])
    for i in range(1, N):
        dl = (odometry_data[i][0] - odometry_data[i - 1][0]) * pi / 180.0
        dr = (odometry_data[i][1] - odometry_data[i - 1][1]) * pi / 180.0
        l = odometry_data[i][0] * pi / 180.0
        r = odometry_data[i][1]* pi / 180.0
        d_angle = (r - l) * (R / L) - angle

        #Matrixes for Pose Exponentials
        transition_matrix = np.array([[cos(angle), -sin(angle)],
                                     [sin(angle), cos(angle)]])
        if d_angle!=0:
            arc_matrix = np.array([[sin(d_angle)/d_angle, (cos(d_angle)-1)/d_angle],
                                         [(1-cos(d_angle))/d_angle, sin(d_angle)/d_angle]])
        else: arc_matrix = np.array([[1,1],
                                    [1,1]])
        local_coordinates_matrix = np.array([[R * (dl + dr) / 2.0],
                                            [0]])
        angle = (r - l) * (R / L)

        global_coordinates = np.add(global_coordinates,np.matmul(np.matmul(transition_matrix, arc_matrix),local_coordinates_matrix))
        coordinates[i][0] = global_coordinates[0]
        coordinates[i][1] = global_coordinates[1]
    x, y = coordinates.T
    plt.scatter(x, y)
    plt.show()
    print(coordinates)
    return coordinates


def min_distance(x1, y1, x2, y2, trajectory):
    d_min = 1.7976931348623157e+308
    for point in trajectory:
        d = abs((y2-y1)*point[0] - (x2-x1)*point[1] +x2*y1 - y2*x1)/((y2-y2)**2 +(x2-x1)**2)**.5
        if (d < d_min):
            d_min = d

    return d_min


if __name__ == '__main__':
    x1, y1 = map(int, input().replace(',', '.').split(" "))
    x2, y2 = map(int, input().replace(',', '.').split(" "))
    #plt.plot([x1,x2],[y1, y2], color='k', marker='o')
    #plt.show()
    encoders_data = data_reader()
    trajectory = coordinates_counter(encoders_data)
    print(int(min_distance(x1, y1, x2, y2, trajectory)) -22)

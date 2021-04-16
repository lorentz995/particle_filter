import numpy as np


def change_ref_system(x, y, yaw, x0, y0, yaw0):
    yaw_ = np.radians(0)

    A = np.array([
            [np.cos(yaw_), np.sin(yaw_), 0, -x*np.cos(yaw_) - y*np.sin(yaw_)],
            [-np.sin(yaw_), np.cos(yaw_), 0, x*np.sin(yaw_) - y*np.cos(yaw_)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    [x_, y_, z_, _] = np.dot(A, np.array([x0, y0, 0, 1]))

    print(x_, y_, z_)

    R = np.array([
        [np.cos(yaw), np.sin(yaw), 0],
        [-np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    [x1, y1, z1] = np.dot(R, np.array([x_, y_, z_]))

    yaw_ = np.fmod((yaw0 - yaw), 2 * np.pi)

    return x1, y1, yaw_


if __name__ == '__main__':
    x0 = 0.2816253662109375
    y0 = 0.06523818969726562
    yaw0 = 0.5490823
    x = 0.26030029296875
    y = 0.0521905517578125
    yaw = 0.5490823

    # x = 1
    # y = 1
    # yaw = np.radians(45 - 90)
    # x0 = 3
    # y0 = 3
    # yaw0 = np.radians(0)

    x1, y1, yaw1 = change_ref_system(x, y, yaw, x0, y0, yaw0)
    print(x1, y1, yaw1)

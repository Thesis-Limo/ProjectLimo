# cython: language_level=3

from libc.math cimport cos, sin, atan2, sqrt, pi
import numpy as np
cimport numpy as cnp
import matplotlib.pyplot as plt

cdef double MAX_ACCEL = 1.0 # maximum acceleration [m/ss]
cdef double ROBOT_RADIUS = 0.2 # robot radius [m]
cdef double DT = 0.1 # default time tick [s]
cdef double DWA_V_MIN = 0.0
cdef double DWA_V_MAX = 0.25
cdef double DWA_OMEGA_MIN = -pi / 6
cdef double DWA_OMEGA_MAX = pi / 6
cdef double DWA_V_RESOLUTION = 0.1
cdef double DWA_OMEGA_RESOLUTION = pi / 90
cdef double PREDICT_TIME = 2.0  # Predict 2 seconds ahead
cdef double TO_GOAL_COST_GAIN = 1.0
cdef double SPEED_COST_GAIN = 1.0
cdef double OBSTACLE_COST_GAIN = 1.0

cdef class DWAPath:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = 0.0
        self.omega = 0.0
        self.cost = 0.0

def generate_trajectory(double v, double omega, double x, double y, double yaw, double current_speed, double dt):
    cdef DWAPath path = DWAPath()
    path.v = v
    path.omega = omega
    cdef double time = 0.0
    cdef int predict_steps = int(PREDICT_TIME / dt)
    cdef double x_, y_, yaw_

    for _ in range(predict_steps):
        time += dt
        v_t = current_speed + MAX_ACCEL * time
        x_ = x + v_t * cos(yaw) * dt
        y_ = y + v_t * sin(yaw) * dt
        yaw_ = yaw + omega * dt
        path.x.append(x_)
        path.y.append(y_)
        path.yaw.append(yaw_)
        x, y, yaw = x_, y_, yaw_

    return path

def calculate_cost(DWAPath path, cnp.ndarray[cnp.float64_t, ndim=2] ob, double gx, double gy):
    cdef double to_goal_cost = TO_GOAL_COST_GAIN * sqrt(pow(path.x[-1] - gx, 2) + pow(path.y[-1] - gy, 2))
    cdef double speed_cost = SPEED_COST_GAIN * (DWA_V_MAX - path.v)

    cdef double min_obstacle_distance = float("inf")
    for ix, iy in zip(path.x, path.y):
        for ob_x, ob_y in ob:
            distance = sqrt(pow(ix - ob_x, 2) + pow(iy - ob_y, 2))
            if distance < min_obstacle_distance:
                min_obstacle_distance = distance
    obstacle_cost = OBSTACLE_COST_GAIN / min_obstacle_distance

    path.cost = to_goal_cost + speed_cost + obstacle_cost
    return path.cost

def check_collision(DWAPath path, cnp.ndarray[cnp.float64_t, ndim=2] ob):
    cdef double d, ix, iy
    for ix, iy in zip(path.x, path.y):
        for ox, oy in ob:
            d = sqrt((ix - ox) ** 2 + (iy - oy) ** 2)
            if d <= ROBOT_RADIUS:
                return False
    return True

def dwa_planning(double x, double y, double yaw, double current_speed, cnp.ndarray[cnp.float64_t, ndim=2] ob, double gx, double gy, double dt=DT, bint debug_mode=False):
    cdef list[DWAPath] paths = []
    cdef DWAPath best_path = None
    cdef double min_cost = float("inf")

    # Generate and evaluate all trajectories
    for v in np.arange(DWA_V_MIN, DWA_V_MAX + DWA_V_RESOLUTION, DWA_V_RESOLUTION):
        for omega in np.arange(DWA_OMEGA_MIN, DWA_OMEGA_MAX + DWA_OMEGA_RESOLUTION, DWA_OMEGA_RESOLUTION):
            path = generate_trajectory(v, omega, x, y, yaw, current_speed, dt)
            if check_collision(path, ob):
                cost = calculate_cost(path, ob, gx, gy)
                paths.append(path)
                if cost < min_cost:
                    min_cost = cost
                    best_path = path

    if debug_mode:
        plt.figure(figsize=(10, 10))
        for path in paths:
            plt.plot(path.x, path.y, '-b', alpha=0.3)
        if best_path:
            plt.plot(best_path.x, best_path.y, '-g', linewidth=2, label="Best Path")
        plt.plot(ob[:, 0], ob[:, 1], "xk", label="Obstacles")
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Dynamic Window Approach Paths')
        plt.legend()
        plt.grid(True)
        plt.show()

    return best_path

# cython: language_level=3

from libc.math cimport cos, sin, sqrt, pi
import numpy as np
cimport numpy as cnp
import matplotlib.pyplot as plt
from functools import lru_cache

cdef double MAX_ACCEL = 1.0 # maximum acceleration [m/ss]
cdef double ROBOT_RADIUS = 0.2 # robot radius [m]
cdef double DT = 0.1 # default time tick [s]
cdef double DWA_V_MIN = 0.0
cdef double DWA_V_RESOLUTION = 0.05
cdef double DWA_OMEGA_RESOLUTION = pi / 90
cdef double PREDICT_TIME = 2.0  # Predict 2 seconds ahead
cdef double TO_GOAL_COST_GAIN = 1.0
cdef double SPEED_COST_GAIN = 1.0
cdef double OBSTACLE_COST_GAIN = 0.5
cdef double TURN_COST_GAIN = 0.05
cdef double TURN_RADIUS = 0.4 # turning radius [m]

cdef class DWAPath:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.omega = 0.0
        self.cost = 0.0

@lru_cache(None)
def generate_trajectory(double v, double omega, double x, double y, double yaw, double current_speed, double target_speed, double dt):
    cdef DWAPath path = DWAPath()
    path.omega = omega
    cdef double time = 0.0
    cdef int predict_steps = int(PREDICT_TIME / dt)
    cdef double x_ = x
    cdef double y_ = y
    cdef double yaw_ = yaw
    cdef double v_t = current_speed

    for _ in range(predict_steps):
        time += dt
        v_t = min(current_speed + MAX_ACCEL * time, target_speed)
        x_ = x + v_t * cos(yaw) * dt
        y_ = y + v_t * sin(yaw) * dt
        yaw_ = yaw + omega * dt
        path.x.append(x_)
        path.y.append(y_)
        path.yaw.append(yaw_)
        path.v.append(v_t)
        x, y, yaw = x_, y_, yaw_

    return path

cdef double calculate_cost(DWAPath path, cnp.ndarray[cnp.float64_t, ndim=2] ob, double gx, double gy, double target_speed, double current_speed, double current_omega):
    cdef double to_goal_cost = TO_GOAL_COST_GAIN * sqrt((path.x[-1] - gx) ** 2 + (path.y[-1] - gy) ** 2)
    cdef double speed_cost = SPEED_COST_GAIN * (target_speed - path.v[0])
    cdef double min_obstacle_distance = np.min(np.sqrt((np.array(path.x)[:, np.newaxis] - ob[:, 0]) ** 2 + (np.array(path.y)[:, np.newaxis] - ob[:, 1]) ** 2))
    cdef double obstacle_cost = OBSTACLE_COST_GAIN / min_obstacle_distance
    cdef double turn_cost = TURN_COST_GAIN * abs(path.omega - current_omega)

    path.cost = to_goal_cost + speed_cost + obstacle_cost + turn_cost
    return path.cost

cdef bint check_collision(DWAPath path, cnp.ndarray[cnp.float64_t, ndim=2] ob):
    cdef double d, ix, iy, ox, oy
    cdef cnp.ndarray[cnp.float64_t, ndim=1] path_x = np.array(path.x, dtype=np.float64)
    cdef cnp.ndarray[cnp.float64_t, ndim=1] path_y = np.array(path.y, dtype=np.float64)
    cdef int i, j
    cdef int path_len = path_x.shape[0]
    cdef int ob_len = ob.shape[0]
    for i in range(path_len):
        ix = path_x[i]
        iy = path_y[i]
        for j in range(ob_len):
            ox = ob[j, 0]
            oy = ob[j, 1]
            d = sqrt((ix - ox) ** 2 + (iy - oy) ** 2)
            if d <= ROBOT_RADIUS:
                return False
    return True

def dwa_planning(double x, double y, double yaw, double current_speed, double current_omega, cnp.ndarray[cnp.float64_t, ndim=2] ob, double gx, double gy, double target_speed, double dt=DT, bint debug_mode=False):
    cdef int v_steps = int((target_speed - DWA_V_MIN) / DWA_V_RESOLUTION + 1)
    cdef list[DWAPath] paths = []
    cdef DWAPath best_path = None
    cdef double min_cost = float("inf")
    cdef double v, omega, cost
    cdef DWAPath path

    for i in range(v_steps):
        v = DWA_V_MIN + i * DWA_V_RESOLUTION
        DWA_OMEGA_MIN = -v / TURN_RADIUS
        DWA_OMEGA_MAX = v / TURN_RADIUS
        omega_steps = int((DWA_OMEGA_MAX - DWA_OMEGA_MIN) / DWA_OMEGA_RESOLUTION + 1)
        
        for j in range(omega_steps):
            omega = DWA_OMEGA_MIN + j * DWA_OMEGA_RESOLUTION
            path = generate_trajectory(v, omega, x, y, yaw, current_speed, target_speed, dt)
            if check_collision(path, ob):
                cost = calculate_cost(path, ob, gx, gy, target_speed, current_speed, current_omega)
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

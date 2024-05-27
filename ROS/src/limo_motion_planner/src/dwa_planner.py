#!/usr/bin/env python3.6
import math
import time

import DWA.dwa as dp
import matplotlib.pyplot as plt
import numpy as np
import yaml
from Dubins.dubins_path_planner import plan_dubins_path

WHEELBASE = 0.2  # [m]
SIM_LOOP = 500
TARGET_SPEED = 0.2  # [m/s]


class Pose:
    def __init__(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = np.deg2rad(yaw)


class State:
    def __init__(self, x, y, yaw, speed):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed


class MotionPlanner:
    def __init__(
        self,
        goal_pose: Pose,
        start_pose: Pose = Pose(0.0, 0.0, 0.0),
        obstacleList: list = [],
        initial_state: State = State(0.0, 0.0, 0.0, 0.01),
    ):
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.obstacleList = obstacleList
        self.initial_state = initial_state
        self.motion_plan = []
        self.planning_done = False

    def get_dubins_path(self, curvature: float = 1.0 / 0.4):
        step_size = 1.0
        start = self.start_pose
        goal = self.goal_pose

        path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(
            start.x, start.y, start.yaw, goal.x, goal.y, goal.yaw, curvature, step_size
        )

        return zip(path_x, path_y)

    def calculate_dwa(self, path, state=None):
        print("Calculating DWA path")
        csp, tx, ty = self.generate_course_and_state_initialization(path)
        state = state or self.initial_state

        start = time.time()
        for _ in range(SIM_LOOP):
            step_start = time.time()
            try:
                state, path, goal_reached = self.run_dwa_step(csp, state, tx, ty)
                if goal_reached:
                    self.planning_done = True
                    print("Goal Reached")
                    break
            except KeyboardInterrupt:
                break

        print("Total planning time: ", time.time() - start)
        return state

    def run_dwa_step(self, csp, state, tx, ty):
        ob = np.array(self.obstacleList)
        gx = tx[-1]
        gy = ty[-1]
        target_speed = TARGET_SPEED
        dwa_path = dp.dwa_planning(
            csp,
            state.x,
            state.y,
            state.yaw,
            state.speed,
            ob,
            target_speed,
            debug_mode=True,
        )
        if dwa_path is None:
            raise RuntimeError("No valid path found.")

        state = self.update(state, dwa_path.v, dwa_path.omega)
        goal_reached = math.hypot(state.x - gx, state.y - gy) <= 1.0
        return state, dwa_path, goal_reached

    def generate_course_and_state_initialization(self, path):
        wx, wy = map(list, zip(*path))
        tx, ty, tyaw, tc, csp = dp.generate_target_course(wx, wy)
        return csp, tx, ty

    def update(self, state, v, omega):
        state.x += v * math.cos(state.yaw) * 0.1
        state.y += v * math.sin(state.yaw) * 0.1
        state.yaw += omega * 0.1
        state.speed = v
        return state


def read_laser_scan_from_file(file_path):
    with open(file_path, "r") as file:
        scan_data_str = file.read()

    # Convert YAML string to Python dictionary
    scan_data_dict = yaml.safe_load(scan_data_str)

    return scan_data_dict


def callback(lidar_msg):

    print("Calculating for timestamp:", lidar_msg["header"]["stamp"])
    obstacles = []
    s = time.time()

    min_dist = lidar_msg["range_min"]
    max_dist = lidar_msg["range_max"]

    for i, distance in enumerate(lidar_msg["ranges"]):
        if min_dist < distance < max_dist:
            angle = lidar_msg["angle_min"] + i * lidar_msg["angle_increment"]
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            obstacles.append((x, y))
    obstacles = np.array(obstacles)

    goal_values = input("Enter goal (x y yaw): ")
    if goal_values == "":
        return

    goal_x, goal_y, goal_yaw = [float(num) for num in goal_values.split()]

    goal_pose = Pose(goal_x, goal_y, goal_yaw)
    planner = MotionPlanner(goal_pose, obstacleList=obstacles)
    dubins_path = list(planner.get_dubins_path())
    planner.calculate_dwa(dubins_path)


if __name__ == "__main__":
    lidar_msg = read_laser_scan_from_file("scan_data.txt")
    callback(lidar_msg)
    print("running planner")

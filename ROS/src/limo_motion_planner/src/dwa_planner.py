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
TARGET_SPEED = 0.25  # [m/s]


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
        dt=0.1,
    ):
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.obstacleList = obstacleList
        self.initial_state = initial_state
        self.motion_plan = []
        self.dt = dt
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
        gx, gy = self.goal_pose.x, self.goal_pose.y
        state = state or self.initial_state

        start = time.time()
        for _ in range(SIM_LOOP):
            step_start = time.time()
            try:
                distance_to_goal = math.hypot(state.x - gx, state.y - gy)
                target_speed = min(TARGET_SPEED, distance_to_goal / 2)
                state, path, goal_reached = self.run_dwa_step(
                    state, gx, gy, target_speed
                )
                if goal_reached:
                    self.planning_done = True
                    print("Goal Reached")
                    break
                self.motion_plan.append(path)
                step_end = time.time()
                print("Time for step: ", step_end - step_start)
            except KeyboardInterrupt:
                break

        print("Total planning time: ", time.time() - start)
        return state

    def run_dwa_step(self, state, gx, gy, target_speed):
        ob = np.array(self.obstacleList)
        dwa_path = dp.dwa_planning(
            state.x,
            state.y,
            state.yaw,
            state.speed,
            ob,
            gx,
            gy,
            target_speed,
            self.dt,
            debug_mode=False,
        )
        if dwa_path is None:
            raise RuntimeError("No valid path found.")

        state = self.update(state, dwa_path.v, dwa_path.omega)
        goal_reached = math.hypot(state.x - gx, state.y - gy) <= 0.3
        return state, dwa_path, goal_reached

    def update(self, state, v, omega):
        state.x += v * math.cos(state.yaw) * self.dt
        state.y += v * math.sin(state.yaw) * self.dt
        state.yaw += omega * self.dt
        state.speed = v
        return state

    def plot(self):
        plt.figure()
        for path in self.motion_plan:
            plt.plot(path.x, path.y, "-b")
        plt.plot(self.goal_pose.x, self.goal_pose.y, "xr")
        plt.grid(True)
        plt.axis("equal")
        plt.show()


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
    planner.plot()


if __name__ == "__main__":
    lidar_msg = read_laser_scan_from_file("scan_data.txt")
    callback(lidar_msg)
    print("running planner")

#!/usr/bin/env python3.6
import math
import random
import threading
import time

import FrenetOptimalTrajectory.frenet_optimal_trajectory as frenet_optimal_trajectory
import matplotlib.pyplot as plt
import numpy as np
import rospy
from Dubins.dubins_path_planner import plan_dubins_path
from sensor_msgs.msg import LaserScan

ROBOT_RADIUS = 0.2  # [m]
WHEELBASE = 0.2  # [m]
TARGET_SPEED = 0.1  # [m/s]


class FrenetPath:
    def __init__(self):
        self.t = []  # Time points along the path
        self.d = []  # Lateral offsets from the reference path
        self.d_d = []  # Lateral velocities
        self.d_dd = []  # Lateral accelerations
        self.d_ddd = []  # Lateral jerks
        self.s = []  # Arc lengths along the reference path
        self.s_d = []  # Longitudinal velocities
        self.s_dd = []  # Longitudinal accelerations
        self.s_ddd = []  # Longitudinal jerks
        self.cd = 0.0  # Lateral cost
        self.cv = 0.0  # Longitudinal cost
        self.cf = 0.0  # Total cost
        self.x = []  # X coordinates in the global frame
        self.y = []  # Y coordinates in the global frame
        self.yaw = []  # Yaw angles along the path
        self.ds = []  # Path segment lengths
        self.c = []  # Curvatures along the path


class Pose:
    def __init__(self, x: float, y: float, yaw: float):
        self.x = x  # X coordinate
        self.y = y  # Y coordinate
        self.yaw = yaw  # Yaw angle


class FrenetState:
    def __init__(self, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0, c_x=0.0, c_y=0.0):
        self.c_speed = c_speed  # Current speed
        self.c_accel = c_accel  # Current acceleration
        self.c_d = c_d  # Current lateral offset
        self.c_d_d = c_d_d  # Current lateral speed
        self.c_d_dd = c_d_dd  # Current lateral acceleration
        self.s0 = s0  # Current arc length along the reference path
        self.c_x = c_x  # Current X coordinate in the global frame
        self.c_y = c_y  # Current Y coordinate in the global frame


class MotionPlanner:
    def __init__(
        self,
        goal_pose: Pose,
        obstacleList: list = [],
        initial_state: FrenetState = FrenetState(0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
    ):
        self.goal_pose = goal_pose
        self.obstacleList = obstacleList
        self.initial_state = initial_state
        self.motion_plan = []
        self.planning_done = False
        self.goal_updated = False
        self.goal_reached = False
        self.obstacles_updated = False
        self.lock = threading.Lock()
        self.paths = []
        self.lidar_data = []

    def update_goal(self, new_goal_pose: Pose):
        with self.lock:
            current_position = Pose(
                self.motion_plan[-1].x[0],
                self.motion_plan[-1].y[0],
                self.motion_plan[-1].yaw[0],
            )

            self.goal_pose = Pose(
                new_goal_pose.x * math.cos(current_position.yaw)
                - new_goal_pose.y * math.sin(current_position.yaw)
                + current_position.x,
                new_goal_pose.x * math.sin(current_position.yaw)
                + new_goal_pose.y * math.cos(current_position.yaw)
                + current_position.y,
                new_goal_pose.yaw + current_position.yaw,
            )
            self.goal_updated = True
            self.goal_reached = False
            self.planning_done = False
            print(
                f"Goal updated to: ({self.goal_pose.x}, {self.goal_pose.y}, {np.rad2deg(self.goal_pose.yaw)})"
            )

    def update_obstacles(self):
        lidar_msg = self.lidar_data[-1]
        with self.lock:
            current_position = Pose(
                self.motion_plan[-1].x[0],
                self.motion_plan[-1].y[0],
                self.motion_plan[-1].yaw[0],
            )

            obstacles = []

            min_dist = lidar_msg.range_min
            max_dist = lidar_msg.range_max

            for i, distance in enumerate(lidar_msg.ranges):
                if min_dist < distance < max_dist:
                    angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)
                    x_global = (
                        x * math.cos(current_position.yaw)
                        - y * math.sin(current_position.yaw)
                        + current_position.x
                    )
                    y_global = (
                        x * math.sin(current_position.yaw)
                        + y * math.cos(current_position.yaw)
                        + current_position.y
                    )

                    obstacles.append((x_global, y_global))
            obstacles = np.array(obstacles)

            self.obstacleList = obstacles

    def store_obstacle_scan(self, lidar_msg):
        self.lidar_data.append((lidar_msg))
        self.obstacles_updated = True

    def get_dubins_path(
        self,
        start: Pose,
        curvature: float = 1.0 / 0.4,
        step_size: float = 1.0,
    ):
        goal = self.goal_pose
        path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(
            start.x, start.y, start.yaw, goal.x, goal.y, goal.yaw, curvature, step_size
        )
        self.paths.append((path_x, path_y))
        return zip(path_x, path_y)

    def calculate_frenet(self, path, state=None):
        csp, tx, ty = self.generate_course_and_state_initialization(path)
        state = state or self.initial_state

        start = time.time()
        while True:
            if self.obstacles_updated:
                self.update_obstacles()
                self.obstacles_updated = False

            step_start = time.time()
            try:
                state, path, goal_reached = self.run_frenet_iteration(
                    csp, state, tx, ty, self.obstacleList
                )
            except AttributeError:
                self.motion_plan = []
                self.planning_done = True
                return

            self.motion_plan.append(path)

            with self.lock:
                if self.goal_updated:
                    self.goal_updated = False
                    new_path = self.get_dubins_path(
                        Pose(path.x[1], path.y[1], path.yaw[1])
                    )
                    csp, tx, ty = self.generate_course_and_state_initialization(
                        new_path
                    )
                    state = FrenetState(
                        c_speed=path.s_d[1],
                        c_accel=path.s_dd[1],
                        c_d=0.0,
                        c_d_d=0.0,
                        c_d_dd=0.0,
                        s0=0,
                        c_x=path.x[1],
                        c_y=path.y[1],
                    )
                if goal_reached:
                    self.goal_reached = True
                    self.planning_done = True
                    print("Goal reached.")
                    break

            step_end = time.time()
            print("Time for step is ", step_end - step_start)

        end = time.time()
        print(f"Time taken to plan: {end - start:.2f} seconds")
        self.planning_done = True

    def run_frenet_iteration(self, csp, state, tx, ty, obstacles):
        goal_dist = np.hypot(tx[-1] - state.c_x, ty[-1] - state.c_y)

        path = frenet_optimal_trajectory.frenet_optimal_planning(
            csp,
            state.s0,
            state.c_speed,
            state.c_accel,
            state.c_d,
            state.c_d_d,
            state.c_d_dd,
            obstacles,
            TARGET_SPEED if goal_dist > 1 else TARGET_SPEED * (goal_dist / 1),
        )

        updated_state = FrenetState(
            c_speed=path.s_d[1],
            c_accel=path.s_dd[1],
            c_d=path.d[1],
            c_d_d=path.d_d[1],
            c_d_dd=path.d_dd[1],
            s0=path.s[1],
            c_x=path.x[1],
            c_y=path.y[1],
        )

        goal_reached = np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 0.2
        return updated_state, path, goal_reached

    def plan(self, start_pose: Pose = Pose(0.0, 0.0, 0.0)):
        path = self.get_dubins_path(start_pose)
        self.calculate_frenet(path)

    def plot(self, motion_plan, full_path, goal_pose=None, area=5.0):
        goal_pose = goal_pose or self.goal_pose
        for path in motion_plan:
            plt.cla()
            current_time = time.time()
            for obs, timestamp in self.obstacles_at_time:
                if current_time - timestamp < 1.0:  # Only show recent obstacles
                    for x, y in obs:
                        circle = plt.Circle((x, y), 0.2, color="k", fill=False)
                        plt.gca().add_patch(circle)
            for path_x, path_y in self.paths:
                plt.plot(path_x, path_y, "r--")
            plt.plot(full_path.x, full_path.y, "b")
            plt.plot(goal_pose.x, goal_pose.y, "xg")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[m/s]:" + str(path.s_d[1])[0:4])
            plt.grid(True)
            plt.pause(0.0001)
        plt.show()

    def generate_course_and_state_initialization(self, path):
        wx, wy = zip(*path)
        tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(
            list(np.array(wx)), list(np.array(wy))
        )
        return csp, tx, ty


def add_to_motion_plan(motion_plan, path, final=False):
    if not final:
        motion_plan.t.append(path.t[1])
        motion_plan.d.append(path.d[1])
        motion_plan.d_d.append(path.d_d[1])
        motion_plan.d_dd.append(path.d_dd[1])
        motion_plan.d_ddd.append(path.d_ddd[1])
        motion_plan.s.append(path.s[1])
        motion_plan.s_d.append(path.s_d[1])
        motion_plan.s_dd.append(path.s_dd[1])
        motion_plan.s_ddd.append(path.s_ddd[1])
        motion_plan.cd = path.cd
        motion_plan.cv = path.cv
        motion_plan.cf = path.cf
        motion_plan.x.append(path.x[1])
        motion_plan.y.append(path.y[1])
        motion_plan.yaw.append(path.yaw[1])
        motion_plan.ds.append(path.ds[1])
        motion_plan.c.append(path.c[1])
    else:
        motion_plan.t.extend(path.t[2:])
        motion_plan.d.extend(path.d[2:])
        motion_plan.d_d.extend(path.d_d[2:])
        motion_plan.d_dd.extend(path.d_dd[2:])
        motion_plan.d_ddd.extend(path.d_ddd[2:])
        motion_plan.s.extend(path.s[2:])
        motion_plan.s_d.extend(path.s_d[2:])
        motion_plan.s_dd.extend(path.s_dd[2:])
        motion_plan.s_ddd.extend(path.s_ddd[2:])
        motion_plan.cd = path.cd
        motion_plan.cv = path.cv
        motion_plan.cf = path.cf
        motion_plan.x.extend(path.x[2:])
        motion_plan.y.extend(path.y[2:])
        motion_plan.yaw.extend(path.yaw[2:])
        motion_plan.ds.extend(path.ds[2:])
        motion_plan.c.extend(path.c[2:])

    return motion_plan


def callback(planner):
    motion_plan = FrenetPath()
    idx = 0
    while not planner.planning_done or idx < len(planner.motion_plan):
        if idx < len(planner.motion_plan):
            path = planner.motion_plan[idx]
            motion_plan = add_to_motion_plan(motion_plan, path)
            idx += 1

    if planner.motion_plan:
        path = planner.motion_plan[-1]
        motion_plan = add_to_motion_plan(motion_plan, path, final=True)

    plan = [
        (motion_plan.s_d[i], math.atan2(WHEELBASE * motion_plan.c[i], 1.0))
        for i in range(len(motion_plan.t) - 1)
    ]
    plan = [(speed, 0 if math.isnan(angle) else angle) for speed, angle in plan]
    print(plan)

    print(
        "Final position: ",
        motion_plan.x[-1],
        motion_plan.y[-1],
        np.rad2deg(motion_plan.yaw[-1]),
    )
    # print("x: ", motion_plan.x)
    # print("y: ", motion_plan.y)
    planner.plot(planner.motion_plan, motion_plan)


def goal_update_listener(planner):
    while True:
        goal_values = input("Enter new goal (x y yaw): ")
        if len(goal_values.split()) == 3:
            goal_x, goal_y, goal_yaw = [float(num) for num in goal_values.split()]
            new_goal_pose = Pose(goal_x, goal_y, np.deg2rad(goal_yaw))
            planner.update_goal(new_goal_pose)
        elif goal_values == "":
            break


def update_obstacles(lidar_msg, planner):
    planner.store_obstacle_scan(lidar_msg)


if __name__ == "__main__":
    print("running")
    planner = MotionPlanner(Pose(0, 0, np.deg2rad(0)))
    rospy.init_node("motion_planner")
    s = rospy.Subscriber("/scan", LaserScan, lambda msg: update_obstacles(msg, planner))
    rospy.spin()

    goal_thread = threading.Thread(target=goal_update_listener, args=(planner,))
    goal_thread.start()

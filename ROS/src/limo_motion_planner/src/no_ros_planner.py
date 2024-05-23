#!/usr/bin/env python3.6
import math
import threading
import time

import FrenetOptimalTrajectory.frenet_optimal_trajectory as frenet_optimal_trajectory
import matplotlib.pyplot as plt
import numpy as np
from Dubins.dubins_path_planner import plan_dubins_path

ROBOT_RADIUS = 0.2  # [m]
WHEELBASE = 0.2  # [m]
SIM_LOOP = 100
TARGET_SPEED = 0.1  # [m/s]


class FrenetPath:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0
        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


class Pose:
    def __init__(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = np.deg2rad(yaw)


class FrenetState:
    def __init__(self, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0, c_x=0.0, c_y=0.0):
        self.c_speed = c_speed
        self.c_accel = c_accel
        self.c_d = c_d
        self.c_d_d = c_d_d
        self.c_d_dd = c_d_dd
        self.s0 = s0
        self.c_x = c_x
        self.c_y = c_y


class MotionPlanner:
    def __init__(
        self,
        goal_pose: Pose,
        start_pose: Pose = Pose(0.0, 0.0, 0.0),
        obstacleList: list = [],
        initial_state: FrenetState = FrenetState(0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
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

    def calculate_frenet(self, path, state=None):
        start = time.time()
        csp, tx, ty = self.generate_course_and_state_initialization(
            [[0, 0], [self.goal_pose.x, self.goal_pose.y]]
        )
        stop = time.time()
        print(f"Time taken to generate course: {stop - start:.2f} seconds")
        state = state or self.initial_state

        start = time.time()
        for _ in range(SIM_LOOP):
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

            if goal_reached:
                break
            step_end = time.time()
            print("Time for step is ", step_end - step_start)

        self.planning_done = True
        end = time.time()
        print(f"Time taken to plan: {end - start:.2f} seconds")

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
        if goal_reached:
            print("Goal reached")
            print(np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]))
        return updated_state, path, goal_reached

    def plan(self):
        path = self.get_dubins_path()
        threading.Thread(target=self.calculate_frenet, args=(path,)).start()

    def plot(self, motion_plan, goal_pose=None, area=5.0):
        goal_pose = goal_pose or self.goal_pose
        for path in motion_plan:
            plt.cla()
            for x, y in self.obstacleList:
                circle = plt.Circle((x, y), 0.2, color="k", fill=False)
                plt.gca().add_patch(circle)
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


tick = True


def callback():
    global tick
    if tick:
        print("Calculating for timestamp:", time.time())
        tick = False

        goal_values = input("Enter goal (x y yaw): ")
        if goal_values == "":
            tick = True
            return

        goal_x, goal_y, goal_yaw = [float(num) for num in goal_values.split()]

        obstacles = np.array([(10.0, 10.0)])

        goal_pose = Pose(goal_x, goal_y, goal_yaw)
        planner = MotionPlanner(goal_pose, obstacleList=obstacles)
        planner.plan()
        time.sleep(1.0)

        motion_plan = FrenetPath()
        idx = 0
        while not planner.planning_done or idx < len(planner.motion_plan):
            if idx < len(planner.motion_plan):
                path = planner.motion_plan[idx]
                motion_plan = add_to_motion_plan(motion_plan, path)
                idx += 1

        motion_plan = add_to_motion_plan(motion_plan, path, final=True)
        plan = [
            (motion_plan.s_d[i], math.atan2(WHEELBASE * motion_plan.c[i], 1.0))
            for i in range(len(motion_plan.t) - 1)
        ]
        plan = [(speed, 0 if math.isnan(angle) else angle) for speed, angle in plan]
        print(plan)

        print("Final position: ", motion_plan.x[-1], motion_plan.y[-1])
        planner.plot(planner.motion_plan)

        input("Press Enter to continue...")

        tick = True


if __name__ == "__main__":
    print("running planner")
    while True:
        callback()

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
from limo_motion_controller.msg import MotionPlan, MovementController
from limo_yolo.msg import map

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

    def __str__(self):
        return f"({self.x}, {self.y}, {self.yaw})"


class FrenetState:
    def __init__(
        self,
        c_speed,
        c_accel,
        c_d=0.0,
        c_d_d=0.0,
        c_d_dd=0.0,
        s0=0.0,
        c_x=0.0,
        c_y=0.0,
        yaw=0.0,
    ):
        self.c_speed = c_speed  # Current speed
        self.c_accel = c_accel  # Current acceleration
        self.c_d = c_d  # Current lateral offset
        self.c_d_d = c_d_d  # Current lateral speed
        self.c_d_dd = c_d_dd  # Current lateral acceleration
        self.s0 = s0  # Current arc length along the reference path
        self.c_x = c_x  # Current X coordinate in the global frame
        self.c_y = c_y  # Current Y coordinate in the global frame
        self.yaw = yaw  # Relative yaw


class MotionPlanner:
    def __init__(
        self,
        publisher: rospy.Publisher,
    ):
        self.goal_pose = None
        self.goal_reached = False
        self.goal_set = False
        self.queued_goal = None

        self.obstacleList = None
        self.obstacles_updated = False
        self.queued_obstacles = None

        self.initial_state = FrenetState(0.01, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.lock = threading.Lock()
        self.publisher = publisher

    def get_distance(self, pose: Pose):
        return math.sqrt(pose.x**2 + pose.y**2)

    def extract_points(self, points):
        x = []
        y = []
        for point in points:
            position = point.point
            x.append(position.x - 0.2)
            y.append(position.y)
        return x, y

    def update_obstacles(self, obstacles: map.obstacles):
        x, y = self.extract_points(obstacles)
        obstacles = np.array(list(zip(x, y)))
        self.set_obstacles(obstacles)

    def set_obstacles(self, obstacles: np.array):
        self.obstacleList = obstacles
        self.obstacles_updated = False

    def set_goal(self, goal: Pose):
        self.goal_pose = goal
        self.goal_set = True
        if self.get_distance(goal) > 0.35:
            self.goal_reached = False

    def update_map(self, map: map):
        if len(map.obstacles):
            self.queued_obstacles = map.obstacles
            self.obstacles_updated = True

    def get_dubins_path(
        self,
        curvature: float = 1.0 / 0.4,
        step_size: float = 1.0,
    ):
        goal = self.goal_pose
        path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(
            0, 0, 0, goal.x, goal.y, goal.yaw, curvature, step_size
        )
        return zip(path_x, path_y)

    def publish_motion(self, motion_plan: FrenetPath):
        plan = MotionPlan()
        cont = MovementController()
        cont.speed = motion_plan.s_d[1]
        cont.angle = motion_plan.c[1]
        cont.duration = 0.5
        plan.sequence.append(cont)
        print("Sending command: ", cont.speed, cont.angle, cont.duration)
        self.publisher.publish(plan)

    def main_loop(self):
        state = self.initial_state
        while True:
            start = time.time()
            if not self.goal_set:
                continue

            new_x, new_y = (
                self.goal_pose.x - state.c_x,
                self.goal_pose.y - state.c_y,
            )
            new_yaw = self.goal_pose.yaw - state.yaw
            if math.isnan(new_x) or math.isnan(new_y) or math.isnan(new_yaw):
                print("Goal reached")
                time.sleep(1)
                continue
            else:
                self.set_goal(Pose(new_x, new_y, new_yaw))
                print("Calculated Goal: ", self.goal_pose)

            if self.obstacles_updated:
                self.update_obstacles(self.queued_obstacles)
            else:
                new_obstacles = []
                for obstacle in self.obstacleList:
                    new_x, new_y = obstacle[0] - state.c_x, obstacle[1] - state.c_y
                    new_obstacles.append([new_x, new_y])
                self.set_obstacles(np.array(new_obstacles))

            csp, tx, ty = self.generate_course_and_state_initialization()

            state = FrenetState(
                c_speed=state.c_speed,
                c_accel=state.c_accel,
            )

            try:
                state, path, goal_reached = self.run_frenet_iteration(
                    csp, state, tx, ty, self.obstacleList
                )
            except AttributeError as e:
                print(e)
                continue
            if goal_reached:
                print("Goal Reached")
            else:
                self.publish_motion(path)

            end = time.time()
            print(f"Time taken to plan: {end - start:.2f} seconds")

    def run_frenet_iteration(self, csp, state, tx, ty, obstacles):
        goal_dist = self.get_distance(self.goal_pose)

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
            yaw=path.yaw[1],
        )

        goal_reached = np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 0.3
        return updated_state, path, goal_reached

    def generate_course_and_state_initialization(self):
        goal = self.goal_pose
        path = self.get_dubins_path()
        wx, wy = zip(*path)
        tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(
            list(np.array(wx)), list(np.array(wy))
        )
        return csp, tx, ty


def update_goal(planner):
    while True:
        goal = input("Set goal (x, y, yaw): ").split()
        if len(goal) != 3:
            print("Invalid input")
            continue
        goal_x, goal_y, goal_yaw = [float(x) for x in goal]
        planner.set_goal(Pose(goal_x, goal_y, np.deg2rad(goal_yaw)))
        time.sleep(1)


if __name__ == "__main__":
    print("running")
    rospy.init_node("motion_planner")

    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    planner = MotionPlanner(pub)
    s = rospy.Subscriber("/map", map, lambda map: planner.update_map(map))

    goal_thread = threading.Thread(target=update_goal, args=(planner,))
    goal_thread.start()

    planner_thread = threading.Thread(target=planner.main_loop)
    planner_thread.start()
    rospy.spin()
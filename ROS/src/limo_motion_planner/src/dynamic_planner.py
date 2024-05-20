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

ROBOT_RADIUS = 0.2  # [m]
WHEELBASE = 0.2  # [m]
TARGET_SPEED = 0.1  # [m/s]
TIME_STEP = 0.7  # [s]


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
        publisher: rospy.Publisher,
    ):
        self.goal_pose = None
        self.goal_updated = False
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
            x.append(position.x)
            y.append(position.y)
        return x, y

    def update_obstacles(self, obstacles: map.obstacles):
        x, y = self.extract_points(obstacles)
        obstacles = np.array(list(zip(x, y)))
        self.set_obstacles(obstacles)

    def set_obstacles(self, obstacles: np.array):
        self.obstacleList = obstacles
        self.obstacles_updated = False

    def update_goal(self, goal: map.goal):
        x, y = self.extract_points(goal)
        x = sum(x) / len(x)
        y = sum(y) / len(y)
        goal_pose = Pose(x, y, math.atan2(y, x))
        self.set_goal(goal_pose)

    def set_goal(self, goal: Pose):
        self.goal_pose = goal
        self.goal_set = True
        self.goal_updated = False
        if self.get_distance(goal) > 0.25:
            self.goal_reached = False

    def update_map(self, map: map):
        if len(map.goal):
            print("setting goal")
            self.queued_goal = map.goal
            self.goal_updated = True
            self.goal_set = True
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
        cont.duration = TIME_STEP
        plan.sequence.append(cont)
        print("Sending command: ", cont.speed, cont.angle, cont.duration)
        self.publisher.publish(plan)

    def main_loop(self):
        state = self.initial_state
        while True:
            start = time.time()
            if not self.goal_set:
                continue

            if self.goal_updated:
                self.update_goal(self.queued_goal)
            else:
                new_x, new_y = (
                    self.goal_pose.x - state.c_x,
                    self.goal_pose.y - state.c_y,
                )
                new_yaw = math.atan2(new_y, new_x)
                self.set_goal(Pose(new_x, new_y, new_yaw))

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
                c_d=0.0,
                c_d_d=0.0,
                c_d_dd=0.0,
                s0=0.0,
                c_x=0.0,
                c_y=0.0,
            )

            try:
                state, path, goal_reached = self.run_frenet_iteration(
                    csp, state, tx, ty, self.obstacleList
                )
            except AttributeError:
                print("No path found.")
                continue

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
        )

        goal_reached = np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 0.2
        return updated_state, path, goal_reached

    def generate_course_and_state_initialization(self):
        goal = self.goal_pose
        path = self.get_dubins_path()
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


if __name__ == "__main__":
    print("running")
    rospy.init_node("motion_planner")

    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    planner = MotionPlanner(pub)
    s = rospy.Subscriber("/map", map, lambda map: planner.update_map(map))

    planner_thread = threading.Thread(target=planner.main_loop)
    planner_thread.start()
    rospy.spin()

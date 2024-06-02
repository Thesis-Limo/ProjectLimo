#!/usr/bin/env python3.6
import math
import random
import threading
import time

import DWA.dwa as dp
import matplotlib.pyplot as plt
import numpy as np
import rospy
from limo_motion_controller.msg import MotionPlan, MovementController
from limo_yolo.msg import map

SIM_LOOP = 500
TARGET_SPEED = 0.1  # [m/s]


class Pose:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self):
        return f"({self.x}, {self.y})"


class State:
    def __init__(self, x, y, yaw, speed, omega):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.omega = omega


class MotionPlanner:
    def __init__(
        self,
        publisher: rospy.Publisher,
        dt=0.1,
    ):
        self.goal_pose = None
        self.goal_updated = False

        self.obstacleList = None

        self.initial_state = State(0.0, 0.0, 0.0, 0.0, 0.0)

        self.lock = threading.Lock()
        self.publisher = publisher

        self.dt = dt

    def get_distance(self, pose: Pose):
        return math.sqrt(pose.x**2 + pose.y**2)

    def extract_points(self, points):
        x = []
        y = []
        for point in points:
            position = point.point
            x.append(position.x - 0.1)
            y.append(position.y)
        return x, y

    def update_obstacles(self, obstacles: map.obstacles):
        x, y = self.extract_points(obstacles)
        obstacles = np.array(list(zip(x, y)))
        self.set_obstacles(obstacles)

    def set_obstacles(self, obstacles: np.array):
        self.obstacleList = obstacles

    def update_goal(self, goal: map.goal):
        x, y = self.extract_points(goal)
        x = sum(x) / len(x)
        y = sum(y) / len(y)
        goal_pose = Pose(x, y)
        self.set_goal(goal_pose)

    def set_goal(self, goal: Pose):
        self.goal_pose = goal
        self.goal_updated = True

    def update_map(self, map: map):
        if len(map.goal):
            self.update_goal(map.goal)
        if len(map.obstacles):
            self.update_obstacles(map.obstacles)

    def publish_motion(self, planner):
        executable_plan = MotionPlan()
        for plan in planner:
            cont = MovementController()
            cont.speed = plan.v
            cont.angle = plan.omega
            cont.duration = self.dt
            executable_plan.sequence.append(cont)
        self.publisher.publish(executable_plan)

    def run_dwa_step(self, state, gx, gy, target_speed):
        ob = np.array(self.obstacleList)
        dwa_path = dp.dwa_planning(
            state.x,
            state.y,
            state.yaw,
            state.speed,
            state.omega,
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
        state.omega = omega
        return state

    def main_loop(self):
        state = self.initial_state
        motion_plan = []
        while True:
            if not self.goal_pose:
                continue

            gx, gy = self.goal_pose.x, self.goal_pose.y

            start = time.time()
            try:
                distance_to_goal = math.hypot(state.x - gx, state.y - gy)
                target_speed = (
                    TARGET_SPEED
                    if distance_to_goal > 1
                    else TARGET_SPEED * distance_to_goal
                )
                state, path, goal_reached = self.run_dwa_step(
                    state, gx, gy, target_speed
                )
                motion_plan.append(path)

                end = time.time()
                # print(f"Time taken to plan: {end - start:.2f} seconds")
            except Exception as e:
                print("Error in step:", e)
                motion_plan = []
                continue

            if goal_reached:
                print("Goal Reached")
                self.publish_motion(motion_plan)
                motion_plan = []
                continue

            if self.goal_updated:
                self.goal_updated = False
                print("Updated Goal: ", self.goal_pose)
                self.publish_motion(motion_plan)
                motion_plan = []
                state = State(0.0, 0.0, 0.0, state.speed, state.omega)
                continue


if __name__ == "__main__":
    print("running")
    rospy.init_node("motion_planner")

    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    planner = MotionPlanner(pub)
    s = rospy.Subscriber("/map", map, lambda map: planner.update_map(map))

    planner_thread = threading.Thread(target=planner.main_loop)
    planner_thread.start()
    rospy.spin()

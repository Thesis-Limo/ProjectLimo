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
    def __init__(self, x, y, yaw, speed):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed


class MotionPlanner:
    def __init__(
        self,
        publisher: rospy.Publisher,
        dt=0.2,
    ):
        self.goal_pose = None
        self.goal_updated = False
        self.goal_reached = False
        self.goal_set = False
        self.queued_goal = None

        self.obstacleList = None
        self.obstacles_updated = False
        self.queued_obstacles = None

        self.initial_state = State(0.0, 0.0, 0.0, 0.0)

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
        self.obstacles_updated = False

    def update_goal(self, goal: map.goal):
        x, y = self.extract_points(goal)
        x = sum(x) / len(x)
        y = sum(y) / len(y)
        goal_pose = Pose(x, y)
        self.set_goal(goal_pose)

    def set_goal(self, goal: Pose):
        self.goal_pose = goal
        self.goal_set = True
        self.goal_updated = False
        if self.get_distance(goal) > 0.35:
            self.goal_reached = False

    def update_map(self, map: map):
        if len(map.goal):
            self.queued_goal = map.goal
            self.goal_updated = True
            self.goal_set = True
        if len(map.obstacles):
            self.queued_obstacles = map.obstacles
            self.obstacles_updated = True

    def publish_motion(self, path):
        plan = MotionPlan()
        cont = MovementController()
        cont.speed = path.v
        cont.angle = path.omega
        cont.duration = self.dt
        plan.sequence.append(cont)
        print("Sending command: ", cont.speed, cont.angle, cont.duration)
        self.publisher.publish(plan)

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

    def main_loop(self):
        state = self.initial_state
        while True:
            start = time.time()
            if not self.goal_set:
                continue
            if self.goal_updated:
                self.update_goal(self.queued_goal)
                print("Updated Goal: ", self.goal_pose)
            else:
                new_x, new_y = (
                    self.goal_pose.x - state.x,
                    self.goal_pose.y - state.y,
                )
                if math.isnan(new_x) or math.isnan(new_y):
                    print("Goal reached")
                    time.sleep(1)
                    continue
                else:
                    self.set_goal(Pose(new_x, new_y))
                    print("Calculated Goal: ", self.goal_pose)

            if self.obstacles_updated:
                self.update_obstacles(self.queued_obstacles)
            else:
                new_obstacles = []
                for obstacle in self.obstacleList:
                    new_x, new_y = obstacle[0] - state.x, obstacle[1] - state.y
                    new_obstacles.append([new_x, new_y])
                self.set_obstacles(np.array(new_obstacles))

            gx, gy = self.goal_pose.x, self.goal_pose.y

            try:
                distance_to_goal = math.hypot(state.x - gx, state.y - gy)
                target_speed = (
                    TARGET_SPEED if distance_to_goal > 0.4 else TARGET_SPEED / 2
                )
                state, path, goal_reached = self.run_dwa_step(
                    state, gx, gy, target_speed
                )
                if goal_reached:
                    self.planning_done = True
                    print("Goal Reached")
                    time.sleep(0.5)
                    continue
            except Exception as e:
                print("Error in step:", e)
                continue

            if goal_reached:
                print("Goal Reached")
            else:
                self.publish_motion(path)

            end = time.time()
            print(f"Time taken to plan: {end - start:.2f} seconds")


if __name__ == "__main__":
    print("running")
    rospy.init_node("motion_planner")

    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    planner = MotionPlanner(pub)
    s = rospy.Subscriber("/map", map, lambda map: planner.update_map(map))

    planner_thread = threading.Thread(target=planner.main_loop)
    planner_thread.start()
    rospy.spin()

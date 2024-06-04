#!/usr/bin/env python3.6
import math
import time

import DWA.dwa as dp
import matplotlib.pyplot as plt
import numpy as np
import rospy
import yaml
from limo_motion_controller.msg import MotionPlan, MovementController
from sensor_msgs.msg import LaserScan

WHEELBASE = 0.2  # [m]
SIM_LOOP = 500
TARGET_SPEED = 0.1  # [m/s]


class Pose:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


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
        goal_pose: Pose,
        start_pose: Pose = Pose(0.0, 0.0),
        obstacleList: list = [],
        initial_state: State = State(0.0, 0.0, 0.0, 0.0, 0.0),
        dt=0.1,
    ):
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.obstacleList = obstacleList
        self.initial_state = initial_state
        self.motion_plan = []
        self.dt = dt
        self.planning_done = False

    def calculate_dwa(self, state=None):
        print("Calculating DWA path")
        gx, gy = self.goal_pose.x, self.goal_pose.y
        state = state or self.initial_state

        start = time.time()
        for _ in range(SIM_LOOP):
            step_start = time.time()
            try:
                distance_to_goal = math.hypot(state.x - gx, state.y - gy)
                target_speed = (
                    TARGET_SPEED
                    if distance_to_goal > 0.5
                    else TARGET_SPEED * distance_to_goal
                )
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
            except Exception as e:
                print("Error in step:", e)
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

        state = self.update(dwa_path)
        goal_reached = math.hypot(state.x - gx, state.y - gy) <= 0.2
        return state, dwa_path, goal_reached

    def update(self, dwa_path):
        state = State(
            dwa_path.x[0],
            dwa_path.y[0],
            dwa_path.yaw[0],
            dwa_path.v[0],
            dwa_path.omega,
        )
        return state

    def plot(self):
        plt.cla()
        for x, y in self.obstacleList:
            circle = plt.Circle((x, y), 0.2, color="k", fill=False)
            plt.gca().add_patch(circle)
        for path in self.motion_plan:
            plt.plot(path.x[:2], path.y[:2], "-b")
        plt.plot(self.goal_pose.x, self.goal_pose.y, "xr")
        plt.grid(True)
        plt.axis("equal")
        plt.show()

        plt.plot([plan.v[0] for plan in self.motion_plan])
        plt.show()


def read_laser_scan_from_file(file_path):
    with open(file_path, "r") as file:
        scan_data_str = file.read()

    # Convert YAML string to Python dictionary
    scan_data_dict = yaml.safe_load(scan_data_str)

    return scan_data_dict


tick = True


def callback(lidar_msg, publisher):
    global tick
    if tick:
        print("Calculating for timestamp:", lidar_msg.header.stamp)
        tick = False
        obstacles = []
        s = time.time()

        min_dist = lidar_msg.range_min
        max_dist = lidar_msg.range_max

        for i, distance in enumerate(lidar_msg.ranges):
            if min_dist < distance < max_dist:
                angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                obstacles.append((x, y))
        obstacles = np.array(obstacles)

        goal_values = input("Enter goal (x y yaw): ")
        if goal_values == "":
            tick = True
            return
        goal_x, goal_y = [float(num) for num in goal_values.split()]

        goal_pose = Pose(goal_x, goal_y)
        initial_state = State(0.0, 0.0, 0.0, 0.0, 0.0)
        planner = MotionPlanner(
            goal_pose, obstacleList=obstacles, initial_state=initial_state
        )
        planner.calculate_dwa()

        executable_plan = MotionPlan()
        for plan in planner.motion_plan:
            cont = MovementController()
            cont.speed = plan.v[0]
            cont.angle = plan.omega
            cont.duration = planner.dt
            executable_plan.sequence.append(cont)
        publisher.publish(executable_plan)

        input("Press Enter to continue...")

        tick = True

        planner.plot()


if __name__ == "__main__":
    print("running planner")
    rospy.init_node("motion_planner")
    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    s = rospy.Subscriber("/scan", LaserScan, lambda msg: callback(msg, pub))
    rospy.spin()

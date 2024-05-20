#!/usr/bin/env python3.6
import math
import threading
import time

import numpy as np
import rospy
from limo_motion_controller.msg import MotionPlan, MovementController
from limo_yolo.msg import map

ROBOT_RADIUS = 0.2  # [m]
WHEELBASE = 0.2  # [m]
TARGET_SPEED = 0.1  # [m/s]
TIME_STEP = 0.2  # [s]


class Pose:
    def __init__(self, x: float, y: float, yaw: float):
        self.x = x  # X coordinate
        self.y = y  # Y coordinate
        self.yaw = yaw  # Yaw angle

    def __str__(self):
        return f"({self.x}, {self.y}, {self.yaw})"


class Path:
    def __init__(self, u_th, u_v):
        self.x = None
        self.y = None
        self.th = None
        self.u_v = u_v
        self.u_th = u_th


class Obstacle:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size


class AckermannRobot:
    def __init__(self, init_x, init_y, init_th, wheelbase):
        self.x = init_x
        self.y = init_y
        self.th = init_th
        self.u_v = 0.0
        self.u_th = 0.0
        self.wheelbase = wheelbase
        self.traj_x = [init_x]
        self.traj_y = [init_y]
        self.traj_th = [init_th]

    def update_state(self, u_th, u_v, dt):
        self.u_th = u_th
        self.u_v = u_v
        delta_x = u_v * math.cos(self.th) * dt
        delta_y = u_v * math.sin(self.th) * dt
        delta_th = u_v / self.wheelbase * math.tan(u_th) * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.traj_x.append(self.x)
        self.traj_y.append(self.y)
        self.traj_th.append(self.th)
        return self.x, self.y, self.th


class DWARobot:
    def __init__(self):
        self.max_acceleration = 1
        self.max_ang_acceleration = 100 * math.pi / 180
        self.lim_max_velo = 0.2  # m/s
        self.lim_min_velo = 0.0  # m/s
        self.lim_max_ang_velo = math.pi
        self.lim_min_ang_velo = -math.pi

    def predict_state(self, ang_velo, velo, x, y, th, dt, pre_step, wheelbase):
        next_xs = []
        next_ys = []
        next_ths = []
        for _ in range(pre_step):
            delta_x = velo * math.cos(th) * dt
            delta_y = velo * math.sin(th) * dt
            delta_th = velo / wheelbase * math.tan(ang_velo) * dt
            x += delta_x
            y += delta_y
            th += delta_th
            next_xs.append(x)
            next_ys.append(y)
            next_ths.append(th)
        return next_xs, next_ys, next_ths


class DWA:
    def __init__(self, wheelbase):
        self.simu_robot = DWARobot()
        self.pre_step = 30
        self.delta_velo = 0.02
        self.delta_ang_velo = 0.02
        self.weight_angle = 0.04
        self.weight_velo = 0.2
        self.weight_obs = 0.1
        self.area_dis_to_obs = 5
        self.wheelbase = wheelbase

    def calc_input(self, g_x, g_y, state, obstacles):
        paths = self._make_path(state)
        opt_path = self._eval_path(paths, g_x, g_y, state, obstacles)
        return paths, opt_path

    def _make_path(self, state):
        min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_velos(state)
        paths = []
        for ang_velo in np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo):
            for velo in np.arange(min_velo, max_velo, self.delta_velo):
                path = Path(ang_velo, velo)
                path.x, path.y, path.th = self.simu_robot.predict_state(
                    ang_velo,
                    velo,
                    state.x,
                    state.y,
                    state.th,
                    TIME_STEP,
                    self.pre_step,
                    self.wheelbase,
                )
                paths.append(path)
        return paths

    def _calc_range_velos(self, state):
        range_ang_velo = TIME_STEP * self.simu_robot.max_ang_acceleration
        min_ang_velo = max(
            state.u_th - range_ang_velo, self.simu_robot.lim_min_ang_velo
        )
        max_ang_velo = min(
            state.u_th + range_ang_velo, self.simu_robot.lim_max_ang_velo
        )
        range_velo = TIME_STEP * self.simu_robot.max_acceleration
        min_velo = max(state.u_v - range_velo, self.simu_robot.lim_min_velo)
        max_velo = min(state.u_v + range_velo, self.simu_robot.lim_max_velo)
        return min_ang_velo, max_ang_velo, min_velo, max_velo

    def _eval_path(self, paths, g_x, g_y, state, obstacles):
        nearest_obs = self._calc_nearest_obs(state, obstacles)
        scores = []
        for path in paths:
            score = (
                self.weight_angle * self._heading_angle(path, g_x, g_y)
                + self.weight_velo * path.u_v
                + self.weight_obs * self._obstacle(path, nearest_obs)
            )
            scores.append(score)
        opt_path = paths[np.argmax(scores)]
        return opt_path

    def _heading_angle(self, path, g_x, g_y):
        angle_to_goal = math.atan2(g_y - path.y[-1], g_x - path.x[-1])
        angle_diff = angle_to_goal - path.th[-1]
        return abs(self._angle_range_corrector(angle_diff))

    def _angle_range_corrector(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _calc_nearest_obs(self, state, obstacles):
        return [
            obs
            for obs in obstacles
            if math.sqrt((state.x - obs.x) ** 2 + (state.y - obs.y) ** 2)
            < self.area_dis_to_obs
        ]

    def _obstacle(self, path, nearest_obs):
        score_obstacle = 2
        for i in range(len(path.x)):
            for obs in nearest_obs:
                dis_to_obs = math.sqrt(
                    (path.x[i] - obs.x) ** 2 + (path.y[i] - obs.y) ** 2
                )
                if dis_to_obs < score_obstacle:
                    score_obstacle = dis_to_obs
                if dis_to_obs < obs.size + 0.75:
                    return -float("inf")
        return score_obstacle


class MotionPlanner:
    def __init__(self, publisher: rospy.Publisher):
        self.goal_pose = None
        self.goal_updated = False
        self.goal_reached = False
        self.goal_set = False
        self.queued_goal = None
        self.obstacleList = None
        self.obstacles_updated = False
        self.queued_obstacles = None
        self.lock = threading.Lock()
        self.publisher = publisher
        self.robot = AckermannRobot(0.0, 0.0, 0.0, WHEELBASE)
        self.controller = DWA(WHEELBASE)

    def get_distance(self, pose: Pose):
        return math.sqrt(pose.x**2 + pose.y**2)

    def extract_points(self, points):
        x, y = [], []
        for point in points:
            position = point.point
            x.append(position.x)
            y.append(position.y)
        return x, y

    def update_obstacles(self, obstacles: map.obstacles):
        x, y = self.extract_points(obstacles)
        self.obstacleList = [Obstacle(x[i], y[i], 0.25) for i in range(len(x))]
        self.obstacles_updated = False

    def update_goal(self, goal: map.goal):
        x, y = self.extract_points(goal)
        x, y = sum(x) / len(x), sum(y) / len(y)
        self.goal_pose = Pose(x, y, math.atan2(y, x))
        self.goal_set = True
        self.goal_updated = False

    def update_map(self, map: map):
        print("updating")
        if len(map.goal):
            self.queued_goal = map.goal
            self.goal_updated = True
            self.goal_set = True
        if len(map.obstacles):
            self.queued_obstacles = map.obstacles
            self.obstacles_updated = True

    def publish_motion(self, path: Path):
        plan = MotionPlan()
        cont = MovementController()
        cont.speed = path.u_v
        cont.angle = path.u_th
        cont.duration = -1
        plan.sequence.append(cont)
        self.publisher.publish(plan)

    def main_loop(self):
        while True:
            start = time.time()
            if not self.goal_set:
                continue
            if self.goal_updated:
                self.update_goal(self.queued_goal)
            if self.obstacles_updated:
                self.update_obstacles(self.queued_obstacles)
            paths, opt_path = self.controller.calc_input(
                self.goal_pose.x, self.goal_pose.y, self.robot, self.obstacleList
            )
            self.robot.update_state(opt_path.u_th, opt_path.u_v, TIME_STEP)
            self.publish_motion(opt_path)
            end = time.time()
            print(f"Time taken to plan: {end - start:.2f} seconds")


if __name__ == "__main__":
    print("Starting motion planner")
    rospy.init_node("motion_planner")
    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    planner = MotionPlanner(pub)
    rospy.Subscriber("/map", map, lambda map: planner.update_map(map))
    planner_thread = threading.Thread(target=planner.main_loop)
    planner_thread.start()
    rospy.spin()

import math
import threading
import time

import FrenetOptimalTrajectory.frenet_optimal_trajectory as frenet_optimal_trajectory
import numpy as np
import rospy
from Dubins.dubins_path_planner import plan_dubins_path
from limo_motion_controller.msg import MotionPlan, MovementController
from limo_yolo.msg import map
from nav_msgs.msg import Odometry

TARGET_SPEED = 0.25  # [m/s]


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
        self.yaw = yaw

    def __str__(self):
        return f"({self.x}, {self.y}, {self.yaw})"


class MotionPlanner:
    def __init__(self, publisher: rospy.Publisher):
        self.speed = 0.0
        self.steering = 0.0
        self.acceleration = 0.0
        self.odom = None
        self.last_timestamp = rospy.Time(0)
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

    def get_obstacles(self, obstacles: map.obstacles):
        x, y = self.extract_points(obstacles)
        obstacles = np.array(list(zip(x, y)))
        return obstacles

    def get_goal(self, goal: map.goal):
        x, y = self.extract_points(goal)
        x = sum(x) / len(x)
        y = sum(y) / len(y)
        goal_pose = Pose(x, y, math.atan2(y, x))
        return goal_pose

    def get_map_data(self, map: map):
        goal = self.get_goal(map.goal)
        obstacles = self.get_obstacles(map.obstacles)
        return goal, obstacles

    def get_speed_and_steering(self, odom: Odometry):
        linear_velocity = odom.twist.twist.linear.x
        angular_velocity = odom.twist.twist.angular.z

        return linear_velocity, angular_velocity

    def set_odom(self, odom: Odometry):
        self.odom = odom

    def get_dubins_path(
        self, goal, curvature: float = 1.0 / 0.4, step_size: float = 1.0
    ):
        path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(
            0, 0, 0, goal.x, goal.y, goal.yaw, curvature, step_size
        )
        return zip(path_x, path_y)

    def publish_motion(self, motion_plan: FrenetPath):
        plan = MotionPlan()
        cont = MovementController()
        cont.speed = motion_plan.s_d[1]
        cont.angle = motion_plan.c[1]
        cont.duration = 1
        plan.sequence.append(cont)
        self.publisher.publish(plan)

    def run_frenet_iteration(self, map: map):
        if not self.odom:
            return

        start = time.time()

        speed, steering_angle = self.get_speed_and_steering(self.odom)
        speed = speed if speed > 0.01 else 0.01

        goal, obstacles = self.get_map_data(map)
        goal_dist = self.get_distance(goal)
        csp, tx, ty = self.generate_course_and_state_initialization(goal)

        path = frenet_optimal_trajectory.frenet_optimal_planning(
            csp,
            0.0,  # s0
            speed,  # c_speed
            self.acceleration,  # c_accel
            0.0,  # c_d
            speed * np.sin(steering_angle),  # c_d_d
            self.acceleration * np.sin(steering_angle),  # c_d_dd
            obstacles,  # ob
            (
                TARGET_SPEED if goal_dist > 2 else TARGET_SPEED * (goal_dist / 2)
            ),  # target_speed
        )

        if path is not None:
            current_timestamp = map.obstacles[0].header.stamp
            if current_timestamp <= self.last_timestamp:
                return

            self.acceleration = path.s_dd[1]
            print("Goal distance:", goal_dist)
            print("Goal pose:", goal)
            print("Acceleration:", self.acceleration)
            self.last_timestamp = current_timestamp
            print("Current speed:", speed)
            print("Current steering angle:", steering_angle)
            print("New speed:", path.s_d[1])
            print("New steering angle:", path.c[1])

            self.publish_motion(path)

        print("Iteration time: ", time.time() - start)
        print()

    def generate_course_and_state_initialization(self, goal):
        path = self.get_dubins_path(goal)
        wx, wy = zip(*path)
        tx, ty, tyaw, tc, csp = frenet_optimal_trajectory.generate_target_course(
            list(np.array(wx)), list(np.array(wy))
        )
        return csp, tx, ty


if __name__ == "__main__":
    print("Starting motion planner")
    rospy.init_node("motion_planner")
    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    planner = MotionPlanner(pub)
    rospy.Subscriber("/odom", Odometry, lambda odom: planner.set_odom(odom))
    rospy.Subscriber("/map", map, lambda map: planner.run_frenet_iteration(map))
    rospy.spin()

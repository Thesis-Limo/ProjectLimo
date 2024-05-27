#!/usr/bin/env python3.6
import time

import numpy as np
import rospy
from limo_motion_controller.msg import MotionPlan, MovementController

# import odom data
from nav_msgs.msg import Odometry


def publish_motion(speed, angle, duration, publisher):
    plan = MotionPlan()
    cont = MovementController()
    cont.speed = speed
    cont.angle = angle
    cont.duration = duration
    plan.sequence.append(cont)
    print("Sending command: ", cont.speed, cont.angle, cont.duration)
    publisher.publish(plan)


def odom_callback(odom):
    # print speed and angle
    print("Speed: ", odom.twist.twist.linear.x)
    print("Angle: ", odom.twist.twist.angular.z)


def main():
    rospy.init_node("motion_planner")

    pub = rospy.Publisher("/limo_motionplan", MotionPlan, queue_size=10)
    while True:
        params = input("Enter speed (m/s), angle (deg), and duration (s): ").split()
        if len(params) == 3:
            rospy.Subscriber("/odom", Odometry, odom_callback)
            speed, angle, duration = [float(x) for x in params]
            publish_motion(speed, np.deg2rad(angle), duration, pub)
            rospy.spin()


if __name__ == "__main__":
    main()

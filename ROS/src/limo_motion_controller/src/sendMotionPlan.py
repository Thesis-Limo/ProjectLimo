#!/usr/bin/env python3.6
import rospy
from limo_motion_controller.msg import MotionPlan, MovementController
if __name__ == '__main__':
    rospy.init_node('limo_motionplan_publisher')
    pub = rospy.Publisher('/limo_motionplan', MotionPlan, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    angle = 0.7
    while not rospy.is_shutdown():
        rate.sleep()
        rate.sleep()
        rate.sleep()
        rate.sleep()
        rate.sleep()
        plan = MotionPlan()
        cont = MovementController()
        cont.speed = 0.4
        cont.angle = 0
        cont.duration = 0.2
        plan.sequence.append(cont)

        cont = MovementController()
        cont.speed = 0.4
        cont.angle = 0
        cont.duration = 2.4
        plan.sequence.append(cont)

        cont = MovementController()
        cont.speed = 0
        cont.angle = 0
        cont.duration = 1
        plan.sequence.append(cont)
        
        # cont = MovementController()
        # cont.speed = 1
        # cont.angle = -angle
        # cont.duration = 0.3
        # plan.sequence.append(cont)

        # cont = MovementController()
        # cont.speed = 1
        # cont.angle = -angle
        # cont.duration = 3
        # plan.sequence.append(cont)
        
        # cont = MovementController()
        # cont.speed = 1
        # cont.angle = 0
        # cont.duration = 0.3
        # plan.sequence.append(cont)
        # cont = MovementController()
        # cont.speed = 1
        # cont.angle = 0
        # cont.duration = 2
        # plan.sequence.append(cont)
        # cont = MovementController()
        # cont.speed = 1
        # cont.angle = angle
        # cont.duration = 0.3
        # plan.sequence.append(cont)



        # cont = MovementController()
        # cont.speed = 0
        # cont.angle = 0
        # cont.duration = 1
        # plan.sequence.append(cont)
        pub.publish(plan)
        break

    print("send")
    
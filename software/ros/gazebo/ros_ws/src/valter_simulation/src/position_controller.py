#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs


#Define a RRBot joint positions publisher for joint controllers.
def valter_joint_positions_publisher():

    #Initiate node for controlling joint1 and joint2 positions.
    rospy.init_node('joint_positions_node', anonymous=True)

    #Define publishers for each joint position controller commands.
    pub1 = rospy.Publisher('/valter/TrunkJoint_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/valter/TorsoJoint_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(100) #100 Hz

    #While loop to have joints follow a certain position, while rospy is not shutdown.
    while not rospy.is_shutdown():

        #Publish current position to each joint.
        pub1.publish(0.3)
        pub2.publish(0.3)

        rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: valter_joint_positions_publisher()
    except rospy.ROSInterruptException: pass

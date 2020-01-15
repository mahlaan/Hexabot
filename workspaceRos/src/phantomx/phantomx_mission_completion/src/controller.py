#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def scanRead(data):
    """
        Callback fonction called when a Scan data from the Lidar is coming.
    """
    rospy.logwarn("SCAN DATA READ")
    cmd=rospy.Publisher("cmd_vel",Twist, queue_size=5)

if __name__ == '__main__':

    rospy.init_node("controller", anonymous=False, log_level=rospy.DEBUG)
    scan=rospy.Subscriber("scan", LaserScan, scanRead)

    while not rospy.is_shutdown():
        rospy.sleep(1)
#        rospy.logwarn("Test")
        pass
    

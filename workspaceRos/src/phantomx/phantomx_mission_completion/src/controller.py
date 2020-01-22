#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose

inf = float('inf')
vel_msg = Twist()
pose = Pose()

def lidarControllerBangBang(data):
    """
    While lidar doesn't detect wall from right or left, robot go straight. When a wall is detected, robot turns to the other side.

    Parameters:

    Input:
        data:
            sensor_msgs.msg: data from lidar
    Output:
        --- None ---
    """
    kp=1
    i_min,i_max = 0,len(data.ranges)-1
    dr_list,dl_list = data.ranges[i_min:i_min+100],data.ranges[i_max-100:i_max]
    dl,dr = np.median(dl_list)<1,np.median(dr_list)<1
    vel_msg.angular.z = - 0.6*dl + 0.6*dr
    cmd.publish(vel_msg)


if __name__ == '__main__':
    """
        Go Straight while there is any detection of wall closer than range 1 (meter) from left or right side.
    """

    rospy.init_node("controller", anonymous=False, log_level=rospy.DEBUG)
    vel_msg.linear.x = 0.4
    error = 0

    rospy.sleep(1)
    cmd = rospy.Publisher("/phantomx/cmd_vel",Twist, queue_size=5)
    scan = rospy.Subscriber("/phantomx/lidar", LaserScan, lidarControllerBangBang)


    while not rospy.is_shutdown():
        rospy.sleep(1)

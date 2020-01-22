#!/usr/bin/env python

import rospy
import rospkg
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose

inf = float('inf')
vel_msg = Twist()
pose = Pose()

def lidarRead(data):
    """
        Callback fonction called when a Scan data from the Lidar is coming.
    """
    i_min,i_max = 0,len(data.ranges)-1
    dr_list,dl_list = data.ranges[i_min:i_min+40],data.ranges[i_max-40:i_max]
    dl,dr = np.median(dl_list),np.median(dr_list)

    kp = 0.1
    ki = 0.01

    if (dl != inf) and (dr != inf):
        e = dl-dr
    else:
        if (dl == inf):
            if (dr == inf):
                e = 0
            else:
                e = 5
        else:
            e = -5

    global error

    if ((error <= 0) and (e >= 0)) or ((error >= 0) and (e <= 0)):
        error = 0

    error += e

    if (np.median(data.ranges[355:365]) <= 0.75) and vel_msg.linear.x == 0.6:
        vel_msg.linear.x = 0
    if (np.median(data.ranges[355:365]) >= 1.25) and vel_msg.linear.x == 0:
        vel_msg.linear.x = 0.6

    vel_msg.angular.z = e*kp + error*ki

    if vel_msg.angular.z >= 0.3:
        vel_msg.angular.z = 0.3
    if vel_msg.angular.z <= -0.3:
        vel_msg.angular.z = -0.3

    cmd.publish(vel_msg)

def lidarControllerBangBang(data):
    kp=1
    i_min,i_max = 0,len(data.ranges)-1
    dr_list,dl_list = data.ranges[i_min:i_min+100],data.ranges[i_max-100:i_max]
    dl,dr = np.median(dl_list)<1,np.median(dr_list)<1
    vel_msg.angular.z = - 0.6*dl + 0.6*dr
    cmd.publish(vel_msg)


if __name__ == '__main__':

    rospy.init_node("controller", anonymous=False, log_level=rospy.DEBUG)
    vel_msg.linear.x = 0.4
    error = 0

    rospy.sleep(1)
    cmd = rospy.Publisher("/phantomx/cmd_vel",Twist, queue_size=5)
    scan = rospy.Subscriber("/phantomx/lidar", LaserScan, lidarControllerBangBang)


    while not rospy.is_shutdown():
        rospy.sleep(1)

#!/usr/bin/env python3

import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest as ROI
import numpy as np

w = 360
h = 270
pid_w = [0.3, 0, 0]
pid_h = [0.5, 0, 0]
pid_f = [0.5, 0, 0]
zero_twist_published = False

def callback(msg):

    # tld_roi = msg
    # cx = tld_roi.x_offset + tld_roi.width // 2
    # cy = tld_roi.y_offset + tld_roi.height // 2
    # area = tld_roi.width * tld_roi.height
    global zero_twist_published
    circle = msg
    cx = circle.x_offset
    cy = circle.y_offset
    area = 3 * circle.width ** 2 
    
    yaw_speed, up_speed, forward_speed = pidtrack(cx, cy, area, w, h, pid_w, pid_h, pid_f)
    rc = "left: " + "0 " + "forward: " + str(forward_speed) + "up: " + str(up_speed) + "  yaw: " + str(yaw_speed)
    speed = Twist()
    if yaw_speed != 0 or up_speed != 0  or forward_speed != 0:
        speed.linear.x = -forward_speed
        speed.linear.y = 0.0
        speed.linear.z = -up_speed
        speed.angular.x = 0.0
        speed.angular.y = 0.0
        speed.angular.z = -yaw_speed
        rospy.loginfo(rc)
        pub.publish(speed)
        zero_twist_published = False
    else:
        if not zero_twist_published:
            pub.publish(speed)
            zero_twist_published = True
            rospy.loginfo("no circle detected")

def pidtrack(cx, cy, area, w, h, pid_w, pid_h, pid_f):
    error_w = cx - w // 2
    speed_w = pid_w[0] * error_w
    speed_w = int(np.clip(speed_w, -100, 100)) / 100.0

    error_h = cy - h // 4
    speed_h = pid_h[0] * error_h
    speed_h = int(np.clip(speed_h, -100, 100)) / 100.0

    error_f = np.sqrt(area) - 180
    speed_f = pid_f[0] * error_f
    speed_f = int(np.clip(speed_f, -100, 100)) / 100.0

    if cx != 0:
        yaw_speed = speed_w
    else:
        yaw_speed = 0
    if cy != 0:
        up_speed = speed_h
    else:
        up_speed = 0
    if area != 0:
        forward_speed = speed_f
    else:
        forward_speed = 0

    return yaw_speed, up_speed, forward_speed

if __name__ == '__main__':

    rospy.init_node('pid', anonymous=True)
    sub = rospy.Subscriber('circle_msg', ROI, callback)
    pub = rospy.Publisher('cmd_vel_circle', Twist, queue_size=1)
    rospy.spin()

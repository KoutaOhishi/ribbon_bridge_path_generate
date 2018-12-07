#!/usr/bin/env python
#coding: utf-8
import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
from nav_msgs.msg import Path
from std_msgs.msg import Int8

from ribbon_bridge_measurement.msg import *

ribbon_bridge_1 = Pose()
ribbon_bridge_2 = Pose()
ribbon_bridge_3 = Pose()

def CB1(msg):
    global ribbon_bridge_1
    ribbon_bridge_1 = msg

def CB2(msg):
    global ribbon_bridge_2
    ribbon_bridge_2 = msg

def CB3(msg):
    global ribbon_bridge_3
    ribbon_bridge_3 = msg


def img_cb(msg):
    try:
        #rospy.loginfo("Subscribed Image Topic !")
        cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        cv_height = cv_img.shape[0]
        cv_width = cv_img.shape[1]

        cv2.circle(cv_img, (int(ribbon_bridge_1.position.x),int(ribbon_bridge_1.position.y)), 20, (255,0,0), -1)
        cv2.circle(cv_img, (int(ribbon_bridge_2.position.x),int(ribbon_bridge_2.position.y)), 20, (0,255,0), -1)
        cv2.circle(cv_img, (int(ribbon_bridge_3.position.x),int(ribbon_bridge_3.position.y)), 20, (0,0,255), -1)





    except CvBridgeError, e:
        rospy.logerror("Failed to Subscribe Image Topic")




    # windowサイズの調整
    show_img_size = (cv_width/5, cv_height/5)
    show_img = cv2.resize(cv_img, show_img_size)



    cv2.imshow("ribbon_bridge", show_img)
    cv2.waitKey(1)




def main():
    rospy.init_node("img_subscriber", anonymous=True)

    sub_img = rospy.Subscriber("/aerial_camera/camera1/image_raw", Image, img_cb)

    sub_pose1 = rospy.Subscriber("/ribbon_bridge_path_generate/control_ribbon_bridge_pose_1", Pose, CB1)

    sub_pose2 = rospy.Subscriber("/ribbon_bridge_path_generate/control_ribbon_bridge_pose_2", Pose, CB2)

    sub_pose3 = rospy.Subscriber("/ribbon_bridge_path_generate/control_ribbon_bridge_pose_3", Pose, CB3)


if __name__ == "__main__":
    main()
    rospy.spin()

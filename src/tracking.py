#!/usr/bin/env python
#coding: utf-8
import rospy
import sys
import time
import math
import cv2
import numpy as np

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from ribbon_bridge_measurement.msg import *
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import *

class Tracking():
    def __init__(self):
        #Subscriber
        self.sub_img = rospy.Subscriber("/aerial_camera/camera1/image_raw", Image, self.sub_img_CB)
        self.sub_Result_data = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.sub_Result_data_CB)

        #imgのwidthとheightの設定(Subscribeの時に更新する)
        self.img_width = 1600#4096
        self.img_height = 900#2160

        #制御する浮体ごとに変数を用意する
        self.RibbonBridgePose_1 = Pose()
        self.RibbonBridgePose_2 = Pose()
        self.RibbonBridgePose_3 = Pose()

        #1つ前のタイミングの浮体の位置を保存しておく
        self.pastRibbonBridgePose_1 = Pose()
        self.pastRibbonBridgePose_2 = Pose()
        self.pastRibbonBridgePose_3 = Pose()

        #浮体の位置をpublishするためのPublisherを浮体の数だけ用意する
        self.pub_RibbonBridgePose_1 = rospy.Publisher("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, queue_size=1)
        self.pub_RibbonBridgePose_2 = rospy.Publisher("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, queue_size=1)
        self.pub_RibbonBridgePose_3 = rospy.Publisher("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, queue_size=1)

        self.RibbonBridges = RibbonBridges()#検出した浮体の配列

        self.Boat_diagonal = 1000 #浮体の対角線の長さ、浮体の追跡のしきい値に用いる。最初は大きめに設定しておく

    def sub_img_CB(self, msg):
        try:
            self.map_width = msg.width
            self.map_height = msg.height

            cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            #浮体の位置をマークする
            cv2.circle(cv_img, (int(self.RibbonBridgePose_1.position.x), int(self.RibbonBridgePose_1.position.y)), 20, (0,0,255), -1)
            cv2.circle(cv_img, (int(self.RibbonBridgePose_2.position.x), int(self.RibbonBridgePose_2.position.y)), 20, (255,0,255), -1)
            #cv2.circle(cv_img, (int(self.RibbonBridgePose_3.position.x), int(self.RibbonBridgePose_3.position.y)), 20, (0,0,255), -1)

            size = (self.map_width/5, self.map_height/5)
            resize_img = cv2.resize(cv_img, size)

            cv2.imshow("tracker result", resize_img)
            cv2.waitKey(1)

        except CvBridgeError, e:
            rospy.logerror("Failed to Subscribe Image Topic")

    def sub_Result_data_CB(self, msg):
        self.RibbonBridges = msg.RibbonBridges

        if len(msg.RibbonBridges) != 0:
            #浮体の長さを取得する
            self.Boat_diagonal = math.sqrt(pow((msg.RibbonBridges[0].corners[0].x-msg.RibbonBridges[0].corners[2].x),2) + pow((msg.RibbonBridges[0].corners[0].y-msg.RibbonBridges[0].corners[2].y),2))

    def pub_RibbonBridges(self):
        #追跡している浮体の位置を常にpublishする
        self.pub_RibbonBridgePose_1.publish(self.RibbonBridgePose_1)
        self.pub_RibbonBridgePose_2.publish(self.RibbonBridgePose_2)
        #self.pub_RibbonBridgePose_3.publish(self.RibbonBridgePose_3)

    def track(self):
        ### 前のタイミングの浮体の位置から一番近い浮体を現在の位置として採用する
        try:
            if len(self.RibbonBridges) == 0:
                rospy.logwarn("Any RibbonBridges DO NOT exist")

            else:
                ##### RibbonBridgePose_1の探索
                dist_list = [] #pastRibbonBridgePose_1からの距離を格納するための配列
                for i in range(len(self.RibbonBridges)):
                    dist = math.sqrt(pow(self.pastRibbonBridgePose_1.position.x-self.RibbonBridges[i].center.x, 2)+pow(self.pastRibbonBridgePose_1.position.y-self.RibbonBridges[i].center.y, 2))
                    #if dist < self.Boat_diagonal:#見つからない状態がずっと続くとここの条件をゆるくする必要があるかも
                    dist_list.append(dist)

                if len(dist_list) == 0:#条件を満たす浮体が見つからなかった
                    rospy.logwarn("RibbonBridge_1 -> LOST")
                else:
                    target_index = dist_list.index(min(dist_list))
                    self.RibbonBridgePose_1.position.x = self.RibbonBridges[target_index].center.x
                    self.RibbonBridgePose_1.position.y = self.RibbonBridges[target_index].center.y
                    self.pastRibbonBridgePose_1 = self.RibbonBridgePose_1

                ##### RibbonBridgePose_2の探索
                dist_list = [] #pastRibbonBridgePose_2からの距離を格納するための配列
                for i in range(len(self.RibbonBridges)):
                    dist = math.sqrt(pow(self.pastRibbonBridgePose_2.position.x-self.RibbonBridges[i].center.x, 2)+pow(self.pastRibbonBridgePose_2.position.y-self.RibbonBridges[i].center.y, 2))
                    #if dist < self.Boat_diagonal:#見つからない状態がずっと続くとここの条件をゆるくする必要があるかも
                    dist_list.append(dist)

                if len(dist_list) == 0:#条件を満たす浮体が見つからなかった
                    rospy.logwarn("RibbonBridge_2 -> LOST")
                else:
                    target_index = dist_list.index(min(dist_list))
                    self.RibbonBridgePose_2.position.x = self.RibbonBridges[target_index].center.x
                    self.RibbonBridgePose_2.position.y = self.RibbonBridges[target_index].center.y
                    self.pastRibbonBridgePose_2 = self.RibbonBridgePose_2

                """##### RibbonBridgePose_3の探索
                dist_list = [] #pastRibbonBridgePose_3からの距離を格納するための配列
                for i in range(len(self.RibbonBridges)):
                    dist = math.sqrt(pow(self.pastRibbonBridgePose_3.position.x-self.RibbonBridges[i].center.x, 2)+pow(self.pastRibbonBridgePose_3.position.y-self.RibbonBridges[i].center.y, 2))
                    #if dist < self.Boat_diagonal:#見つからない状態がずっと続くとここの条件をゆるくする必要があるかも
                    dist_list.append(dist)

                if len(dist_list) == 0:#条件を満たす浮体が見つからなかった
                    rospy.logwarn("RibbonBridge_3 -> LOST")
                else:
                    target_index = dist_list.index(min(dist_list))
                    self.RibbonBridgePose_3.position.x = self.RibbonBridges[target_index].center.x
                    self.RibbonBridgePose_3.position.y = self.RibbonBridges[target_index].center.y
                    self.pastRibbonBridgePose_3 = self.RibbonBridgePose_3"""
        except:
            pass

    def main(self):
        #浮体ごとに初期位置を設定する
        self.pastRibbonBridgePose_1.position.x = 0
        self.pastRibbonBridgePose_1.position.y = 0

        self.pastRibbonBridgePose_2.position.x = 0
        self.pastRibbonBridgePose_2.position.y = self.img_height

        while not rospy.is_shutdown():
            self.track()
            self.pub_RibbonBridges()


if __name__ == "__main__":
    rospy.init_node("TrackingNode")
    c = Tracking()
    c.main()

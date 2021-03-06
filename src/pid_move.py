#!/usr/bin/env python
#coding: utf-8
import rospy
import sys
import time
import math
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from ribbon_bridge_measurement.msg import *
from matplotlib import pyplot as plt
from nav_msgs.msg import *

class PathMoving():
    def __init__(self):
        self.Model_name = "tug_boat"
        self.Reference_frame = "world"

        self.Path_status = False #Pathの生成が上手くできていればTrue

        self.Goal_pose = Pose()
        self.Model_pose = Pose()

        self.True_Model_state = ModelState() #浮体の位置の真値

        self.Path = Path()
        self.GetPathFlag = False

        self.Duration_time = 0.001 #一回の操作で浮体に力を与える時間
        self.Sampling_time = 1.0 #PID制御を行う時間
        self.Arrival_distance = 10 #この値より小さくなれば到着したと判定する(小さすぎると止まるタイミングを見失う)
        self.NotArrival_distance = self.Arrival_distance + 1.0

        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

        #浮体のPose
        self.RibbonBridgePose_1 = Pose() #制御対象
        self.RibbonBridgePose_2 = Pose() #障害物
        #self.RibbonBridgePose_3 = Pose() #障害物

        #それぞれの浮体の位置をsubscribeする
        self.sub_RibbonBridgePose_1 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_1", Pose, self.sub_RibbonBridgePose_1_CB)
        self.sub_RibbonBridgePose_2 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_2", Pose, self.sub_RibbonBridgePose_2_CB)
        #self.sub_RibbonBridgePose_3 = rospy.Subscriber("/ribbon_bridge_path_generate/RibbonBridgePose_3", Pose, self.sub_RibbonBridgePose_3_CB)

        self.sub_True_Model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.sub_True_Model_pose_CB)

        self.sub_Path = rospy.Subscriber("ribbon_bridge_path_generate/path", Path, self.sub_Path_CB)

        self.sub_Boat_num = rospy.Subscriber("/darknet_ros/found_object", Int8, self.sub_Boat_num_CB) #YOLOのノードとの接続を確認するために作成

        self.sub_Path_status = rospy.Subscriber("/ribbon_bridge_path_generate/status", Bool, self.sub_Path_status_CB)

        self.pub_Model_pose = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        #self.pub_Model_pose = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=1)

        #PID制御用のパラメータ
        self.DistX_1 = 0.0
        self.DistY_1 = 0.0
        self.DistZ_1 = 0.0
        self.DistX_2 = 0.0
        self.DistY_2 = 0.0
        self.DistZ_2 = 0.0
        self.Fx = 0.0
        self.Fy = 0.0
        self.Fz = 0.0
        self.Kp = 0.5
        self.Ki = 0.00
        self.Kd = 0.001
        self.count = 1

    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg
        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

    def sub_Boat_num_CB(self, msg):
        self.Boat_num = msg.data

    def sub_Path_status_CB(self, msg):
        self.Path_status = msg.data

    def sub_Path_CB(self, msg):
        if self.Path == msg:
            self.GetPathFlag = False
        else:
            self.Path = msg
            self.GetPathFlag = True

    def sub_RibbonBridgePose_1_CB(self, msg):
        self.RibbonBridgePose_1 = msg
        self.Model_pose = msg

        #到着判定
        if self.ArrivedFlag_X == False:
            #if round(self.Model_pose.position.x,2) == self.Goal_pose.position.x:
            if abs(self.Model_pose.position.x - self.Goal_pose.position.x) < self.Arrival_distance:
                #self.stop_ribbon_bridge("x")
                self.ArrivedFlag_X = True
                #rospy.loginfo("Arrived X")

        else: #self.ArrivedFlag_X == True:
            #if round(self.Model_pose.position.x,2) != self.Goal_pose.position.x:
            if abs(self.Model_pose.position.x - self.Goal_pose.position.x) >= self.NotArrival_distance:
                self.ArrivedFlag_X = False

        if self.ArrivedFlag_Y == False:
            #if round(self.Model_pose.position.y,2) == self.Goal_pose.position.y:
            if abs(self.Model_pose.position.y - self.Goal_pose.position.y) < self.Arrival_distance:
                #self.stop_ribbon_bridge("y")
                self.ArrivedFlag_Y = True
                #rospy.loginfo("Arrived Y")

        else: #self.ArrivedFlag_Y == True:
            #if round(self.Model_pose.position.y,2) != self.Goal_pose.position.y:
            if abs(self.Model_pose.position.y - self.Goal_pose.position.y) >= self.NotArrival_distance:
                self.ArrivedFlag_Y = False



    def sub_RibbonBridgePose_2_CB(self, msg):
        self.RibbonBridgePose_2 = msg

    def sub_RibbonBridgePose_3_CB(self, msg):
        self.RibbonBridgePose_3 = msg

    def stop_ribbon_bridge(self, way):
        """ wayの向きにかかるforceを0にする→ブレーキをかける """
        ### Srv版
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        model_state_srv = ModelState()

        model_state_srv.model_name = self.Model_name
        model_state_srv.reference_frame = self.Reference_frame
        model_state_srv.pose = self.True_Model_state.pose
        model_state_srv.twist = self.True_Model_state.twist

        if way == "x":
            model_state_srv.twist.linear.x = 0.000

        elif way == "y":
            model_state_srv.twist.linear.y = 0.000

        elif way == "z":
            pass

        else:
            rospy.logerr("Invailed argument [%s]"%str(way))

        try:
            set_model_state(model_state=model_state_srv)

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/set_model_state] Exception")
            rospy.sleep(5)

    def sub_True_Model_pose_CB(self, msg):
        """ gazebo空間における浮体の位置の真値を取得する """
        i = msg.name.index(self.Model_name)
        self.True_Model_state.model_name = msg.name[i]
        self.True_Model_state.pose = msg.pose[i]
        self.True_Model_state.twist = msg.twist[i]

    def euler_to_quaternion(self, euler):
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def quaternion_to_euler(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])

    def Add_force_with_param(self, fx, fy, fz, tx, ty, tz):
        apply_body_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        clear_body_wrenches = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)
        get_link_state = rospy.ServiceProxy("/gazebo/get_link_state",GetLinkState)

        target_link = self.Model_name + "::body"
        wrench = Wrench()
        duration = rospy.Duration(self.Duration_time) #単位はsecond

        wrench.force.x = fx
        wrench.force.y = fy
        wrench.force.z = fz
        wrench.torque.x = tx
        wrench.torque.y = ty
        wrench.torque.z = tz

        try:
            apply_body_wrench(body_name=target_link,
            reference_frame=self.Reference_frame,
            wrench=wrench,
            duration=duration)

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/apply_body_wrench] Exception")
            rospy.sleep(5)

    def move(self):
        #if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
            #pass

        #else:
        dist_x = self.Goal_pose.position.x - self.Model_pose.position.x
        dist_y = self.Model_pose.position.y - self.Goal_pose.position.y

        print "Goal_position:x[%s] y[%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)))
        print "Model_position:x[%s] y[%s]"%(str(round(self.Model_pose.position.x)),str(round(self.Model_pose.position.y)))
        print "position_distance:x[%s], y[%s]"%(str(round(dist_x, 2)), str(round(dist_y, 2)))
        print "Fx:[%s], Fy:[%s]"%(str(round(self.Fx,2)), str(round(self.Fy,2)))
        print "---"

        #MVn = MVn-1 + Kp(en-en-1) + Ki en + Kd((en-en-1) - (en-1-en-2))

        fx = self.Fx + self.Kp * (dist_x-self.DistX_1) + self.Ki * dist_x + self.Kd * ((dist_x-self.DistX_1)-(self.DistX_1-self.DistX_2))
        fy = self.Fy + self.Kp * (dist_y-self.DistY_1) + self.Ki * dist_y + self.Kd * ((dist_y-self.DistY_1)-(self.DistY_1-self.DistY_2))

        self.Add_force_with_param(fx,fy,0.0,0.0,0.0,0.0)

        self.Fx = fx
        self.Fy = fy
        #self.Fz = tz

        self.DistX_2 = self.DistX_1
        self.DistY_2 = self.DistY_1
        #self.DistZ_2 = self.DistZ_1

        self.DistX_1 = dist_x
        self.DistY_1 = dist_y
        #self.DistZ_1 = dist_z

        #rospy.sleep(self.Duration_time)
        time.sleep(self.Sampling_time)


    def main(self):
        rospy.loginfo("waiting path")
        while not rospy.is_shutdown():
            if len(self.Path.poses) != 0:
                rospy.loginfo("***** Start *****")
                break

        next_goal_flag = False

        while not rospy.is_shutdown():
            if len(self.Path.poses) <= 2:
                break

            else:
                pass

            if next_goal_flag == True:
                self.Goal_pose.position.x = self.Path.poses[2].pose.position.y
                self.Goal_pose.position.y = self.Path.poses[2].pose.position.x

            else:
                self.Goal_pose.position.x = self.Path.poses[1].pose.position.y
                self.Goal_pose.position.y = self.Path.poses[1].pose.position.x

            if self.sub_Boat_num.get_num_connections() == 0 or self.Path_status == False:
                self.stop_ribbon_bridge("x")
                self.stop_ribbon_bridge("y")
                self.stop_ribbon_bridge("z")

                if self.Path_status == False:
                    #経路生成に問題発生
                    rospy.logwarn("Some error has occured")

                if self.sub_Boat_num.get_num_connections() == 0:
                    #YOLOのノードとの接続が切れた場合
                    rospy.logwarn("YOLO has Dead")
                rospy.sleep(1)

            else:
                if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
                    self.ArrivedFlag_X == False
                    self.ArrivedFlag_Y == False
                    #rospy.loginfo("next sub-Goal Position")
                    #if next_goal_flag == False:
                    next_goal_flag = True
                    #else:
                        #next_goal_flag = False

                self.move()

        self.Goal_pose.position.x = self.Path.poses[len(self.Path.poses)-1].pose.position.y
        self.Goal_pose.position.y = self.Path.poses[len(self.Path.poses)-1].pose.position.x

        #goalでstay
        while not rospy.is_shutdown():
            self.move()

            if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
                rospy.loginfo("***** Arrived *****")





if __name__ == "__main__":
    rospy.init_node("PathMoving", anonymous=True)
    c = PathMoving()
    c.main()
    rospy.spin()

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

class MoveAlongPath():
    def __init__(self):
        self.Model_name = "tug_boat"
        self.Reference_frame = "world"

        self.Model_pose = Pose()
        self.pastModel_pose = Pose()
        self.Model_corners = [(0,0),(0,0),(0,0),(0,0)]
        self.Boat_diagonal = 1000

        self.Boat_num = 0 #YOLOで認識した浮体の数

        self.Goal_pose = Pose()
        self.True_Model_state = ModelState() #浮体の位置の真値

        self.Path = Path()
        self.GetPathFlag = False

        self.Duration_time = 0.01 #一回の操作で浮体に力を与える時間
        self.Arrival_distance = 1.0 #この値より小さくなれば到着したと判定する(小さすぎると止まるタイミングを見失う)
        self.NotArrival_distance = self.Arrival_distance + 1.0

        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

        self.sub_Goal_pose = rospy.Subscriber("/ribbon_bridge_path_generate/goal_pose", Pose, self.sub_Goal_pose_CB)

        self.sub_Model_pose = rospy.Subscriber("/ribbon_bridge_measurement/result_data", RibbonBridges, self.sub_Model_pose_CB)

        self.sub_True_Model_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.sub_True_Model_pose_CB)

        self.sub_Path = rospy.Subscriber("ribbon_bridge_path_generate/path", Path, self.sub_Path_CB)

        self.sub_Boat_num = rospy.Subscriber("/darknet_ros/found_object", Int8, self.sub_Boat_num_CB) #YOLOのノードとの接続を確認するために作成

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
        self.Kp = 0.25
        self.Ki = 0.0
        self.Kd = 0.0
        self.count = 1

    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg
        self.ArrivedFlag_X = False
        self.ArrivedFlag_Y = False
        self.ArrivedFlag_Z = False

    def sub_Boat_num_CB(self, msg):
        self.Boat_num = msg

    def sub_Path_CB(self, msg):
        if self.Path == msg:
            self.GetPathFlag = False
        else:
            self.Path = msg
            self.GetPathFlag = True

        """path = Path()
        previous_x = 0
        previous_y = 0
        for i in range(len(msg.poses)):
            if i == 0:
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = msg.poses[i].pose.position.x
                poseStamped.pose.position.y = msg.poses[i].pose.position.y
                path.poses.append(poseStamped)

            elif previous_x != msg.poses[i].pose.position.x and previous_y != msg.poses[i].pose.position.y:
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = msg.poses[i-1].pose.position.x
                poseStamped.pose.position.y = msg.poses[i-1].pose.position.y
                path.poses.append(poseStamped)

            elif i == len(msg.poses)-1:
                poseStamped = PoseStamped()
                poseStamped.pose.position.x = msg.poses[i].pose.position.x
                poseStamped.pose.position.y = msg.poses[i].pose.position.y
                path.poses.append(poseStamped)

            else:
                pass

            previous_x = msg.poses[i].pose.position.x
            previous_y = msg.poses[i].pose.position.y

        self.Path = path"""


    def stop_ribbon_bridge(self, way):
        """ wayの向きにかかるforceを0にする→ブレーキをかける """
        pub_msg = ModelState()
        pub_msg.model_name = self.Model_name

        #pub_msg = LinkState()
        #pub_msg.link_name = self.Model_name + "::body"
        pub_msg.pose = self.True_Model_state.pose
        pub_msg.twist = self.True_Model_state.twist

        if way == "x":
            pub_msg.twist.linear.x = 0.0
            #if self.ArrivedFlag_Y == True:
                #pub_msg.twist.linear.y = 0.0
            self.pub_Model_pose.publish(pub_msg)
            #rospy.sleep(0.1)

        elif way == "y":
            pub_msg.twist.linear.y = 0.0
            #if self.ArrivedFlag_X == True:
                #pub_msg.twist.linear.x = 0.0
            self.pub_Model_pose.publish(pub_msg)
            #rospy.sleep(0.1)


        elif way == "z":
            pass

        else:
            rospy.logerr("Invailed argument [%s]"%str(way))

        """set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        model_state_srv = ModelState()

        model_state_srv.model_name = self.Model_name
        model_state_srv.reference_frame = self.Reference_frame
        model_state_srv.pose = self.True_Model_state.pose
        model_state_srv.twist = self.True_Model_state.twist

        if way == "x":
            model_state_srv.twist.linear.y = 0.000

        elif way == "y":
            model_state_srv.twist.linear.x = 0.000

        elif way == "z":
            pass

        else:
            rospy.logerr("Invailed argument [%s]"%str(way))

        try:
            set_model_state(model_state=model_state_srv)
            #print "##########"
            #print res

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/set_model_state] Exception")
            rospy.sleep(5)"""

    def sub_True_Model_pose_CB(self, msg):
        """ gazebo空間における浮体の位置の真値を取得する """
        i = msg.name.index(self.Model_name)
        self.True_Model_state.model_name = msg.name[i]
        self.True_Model_state.pose = msg.pose[i]
        self.True_Model_state.twist = msg.twist[i]

    def sub_Model_pose_CB(self, msg):
        """ 指定した浮体の位置をsubscribeする、Goalに到着したらブレーキをかける """
        i = 0

        if len(msg.RibbonBridges) == 0:
            print "There are no RibbonBridges"

        else:
            dist_list = []
            for i in range(len(msg.RibbonBridges)):
                dist = math.sqrt(pow(self.pastModel_pose.position.x-msg.RibbonBridges[i].center.x,2)+pow(self.pastModel_pose.position.y-msg.RibbonBridges[i].center.y,2))

                if dist < self.Boat_diagonal:
                    dist_list.append(dist)

            #print "Boat_diagonal:[%s]"%str(self.Boat_diagonal)
            #print "dist_list:[%s]"%str(dist_list)

            if len(dist_list) == 0:#条件を満たす浮体がいない場合
                self.stop_ribbon_bridge("x")
                self.stop_ribbon_bridge("y")
                self.stop_ribbon_bridge("z")
                rospy.logwarn("RibbonBridge LOST")

            else:
                target_index = dist_list.index(min(dist_list))
                self.Model_pose.position.x = msg.RibbonBridges[target_index].center.x
                self.Model_pose.position.y = msg.RibbonBridges[target_index].center.y
                self.pastModel_pose = self.Model_pose

                diagonal = pow((msg.RibbonBridges[target_index].corners[0].x-msg.RibbonBridges[target_index].corners[2].x),2) + pow((msg.RibbonBridges[target_index].corners[0].y-msg.RibbonBridges[target_index].corners[2].y),2)
                self.Boat_diagonal = math.sqrt(diagonal)

                if self.ArrivedFlag_X == False:
                    #if round(self.Model_pose.position.x,2) == self.Goal_pose.position.x:
                    if abs(self.Model_pose.position.x - self.Goal_pose.position.x) < self.Arrival_distance:
                        self.stop_ribbon_bridge("x")
                        self.ArrivedFlag_X = True
                        #rospy.loginfo("Arrived X")

                else: #self.ArrivedFlag_X == True:
                    #if round(self.Model_pose.position.x,2) != self.Goal_pose.position.x:
                    if abs(self.Model_pose.position.x - self.Goal_pose.position.x) >= self.NotArrival_distance:
                        self.ArrivedFlag_X = False

                if self.ArrivedFlag_Y == False:
                    #if round(self.Model_pose.position.y,2) == self.Goal_pose.position.y:
                    if abs(self.Model_pose.position.y - self.Goal_pose.position.y) < self.Arrival_distance:
                        self.stop_ribbon_bridge("y")
                        self.ArrivedFlag_Y = True
                        #rospy.loginfo("Arrived Y")

                else: #self.ArrivedFlag_Y == True:
                    #if round(self.Model_pose.position.y,2) != self.Goal_pose.position.y:
                    if abs(self.Model_pose.position.y - self.Goal_pose.position.y) >= self.NotArrival_distance:
                        self.ArrivedFlag_Y = False



        """#到着の確認
        if self.ArrivedFlag_X == False:
            if round(self.Model_pose.position.x,2) == self.Goal_pose.position.x:
                pub_msg = ModelState()
                pub_msg.model_name = self.Model_name
                pub_msg.pose = msg.pose[i]
                pub_msg.twist = msg.twist[i]
                pub_msg.twist.linear.x = 0.0
                self.pub_Model_pose.publish(pub_msg)
                self.ArrivedFlag_X = True

        else:
            if round(self.Model_pose.position.x,2) != self.Goal_pose.position.x:
                self.ArrivedFlag_X = False

        if self.ArrivedFlag_Y == False:
            if round(self.Model_pose.position.y,2) == self.Goal_pose.position.y:
                pub_msg = ModelState()
                pub_msg.model_name = self.Model_name
                pub_msg.pose = msg.pose[i]
                pub_msg.twist = msg.twist[i]
                pub_msg.twist.linear.y = 0.0
                self.pub_Model_pose.publish(pub_msg)
                self.ArrivedFlag_Y = True

        else:
            if round(self.Model_pose.position.y,2) != self.Goal_pose.position.y:
                self.ArrivedFlag_Y = False


        if self.ArrivedFlag_Z == False:
            if round(self.quaternion_to_euler(Quaternion(x=self.Model_pose.orientation.x,y=self.Model_pose.orientation.y,z=self.Model_pose.orientation.z,w=self.Model_pose.orientation.w)).z, 2) == round(self.quaternion_to_euler(Quaternion(x=self.Goal_pose.orientation.x,y=self.Goal_pose.orientation.y,z=self.Goal_pose.orientation.z,w=self.Goal_pose.orientation.w)).z, 2):
            #if round(self.Model_pose.orientation.z,2) == self.Goal_pose.orientation.z:
                #if round(self.Model_pose.orientation.w,2) == self.Goal_pose.orientation.w:
                pub_msg = ModelState()
                pub_msg.model_name = self.Model_name
                pub_msg.pose = msg.pose[i]
                pub_msg.twist = msg.twist[i]
                pub_msg.twist.angular.z = 0.0
                self.pub_Model_pose.publish(pub_msg)
                self.ArrivedFlag_Z = True

        else:
            if round(self.quaternion_to_euler(Quaternion(x=self.Model_pose.orientation.x,y=self.Model_pose.orientation.y,z=self.Model_pose.orientation.z,w=self.Model_pose.orientation.w)).z, 2) != round(self.quaternion_to_euler(Quaternion(x=self.Goal_pose.orientation.x,y=self.Goal_pose.orientation.y,z=self.Goal_pose.orientation.z,w=self.Goal_pose.orientation.w)).z, 2):
                self.ArrivedFlag_Z = False"""

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
        if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
            pass

        else:
            dist_x = self.Goal_pose.position.x - self.Model_pose.position.x
            dist_y = self.Model_pose.position.y - self.Goal_pose.position.y

            print "Goal_position:x[%s] y[%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)))
            print "Model_position:x[%s] y[%s]"%(str(round(self.Model_pose.position.x)),str(round(self.Model_pose.position.y)))
            print "position_distance:x[%s], y[%s]"%(str(round(dist_x, 2)), str(round(dist_y, 2)))
            print "---"

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


    def main(self):
        rospy.loginfo("waiting path")
        while not rospy.is_shutdown():
            if len(self.Path.poses) != 0:
                rospy.loginfo("***** Start *****")
                break

        for i in range(len(self.Path.poses)):
        #while not rospy.is_shutdown():
            rospy.sleep(1)
            if i == len(self.Path.poses):
                break

            self.Goal_pose.position.x = self.Path.poses[i].pose.position.y
            self.Goal_pose.position.y = self.Path.poses[i].pose.position.x

            self.Goal_pose.orientation.z = 0.707
            self.Goal_pose.orientation.w = 0.707

            self.ArrivedFlag_X == False
            self.ArrivedFlag_Y == False

            while not rospy.is_shutdown():
                if self.sub_Boat_num.get_num_connections == 0:
                    #YOLOのノードとの接続が切れた場合
                    self.stop_ribbon_bridge("x")
                    self.stop_ribbon_bridge("y")
                    self.stop_ribbon_bridge("z")
                    rospy.logwarn("YOLO has Dead")

                else:
                    self.move()
                    print "%s/%s"%(str(i), str(len(self.Path.poses)))

                    #print "---"
                    #for i in range(len(self.Path.poses)):
                        #print "x:[%s] y:[%s]"%(str(self.Path.poses[i].pose.position.x), str(self.Path.poses[i].pose.position.y))


                    if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
                        self.ArrivedFlag_X == False
                        self.ArrivedFlag_Y == False
                        rospy.loginfo("next sub-Goal Position")
                        break

        self.Goal_pose.position.x = self.Path.poses[len(self.Path.poses)-1].pose.position.y
        self.Goal_pose.position.y = self.Path.poses[len(self.Path.poses)-1].pose.position.x
        #goalでstay
        while not rospy.is_shutdown():
            self.move()

            if self.ArrivedFlag_X == True and self.ArrivedFlag_Y == True:
                rospy.loginfo("***** Arrived *****")





if __name__ == "__main__":
    rospy.init_node("MoveAlongPath", anonymous=True)
    c = MoveAlongPath()
    c.main()
    rospy.spin()

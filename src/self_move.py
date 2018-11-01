#!/usr/bin/env python
#coding: utf-8
import rospy
import sys

from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from matplotlib import pyplot as plt

class Self_Move():
    def __init__(self):
        self.Goal_pose = Pose()
        self.Target_pose = Pose()
        self.Force_param = 5.0
        self.Torque_param = 10.0
        self.Target_model = "tug_boat_4"
        self.Reference_frame = "world"

        # DefaultでGoal_poseは決めておく
        self.Goal_pose.position.x = 10.0
        self.Goal_pose.position.y = 0.0

        self.sub_Goal_pose = rospy.Subscriber("/ribbon_bridge_path_generate/goal_pose", Pose, self.sub_Goal_pose_CB)
        self.sub_Target_pose = rospy.Subscriber("/gazebo/model_states", ModelStates, self.sub_Target_pose_CB)

        #PIDのパラメータ
        self.L = 0.5 #ムダ時間
        self.T = 1.0 #時定数
        self.K = 10.0 #160.0 #定常値(最初の計測値と目標値との差)
        self.DistX_1 = 0.0
        self.DistY_1 = 0.0
        self.DistX_2 = 0.0
        self.DistY_2 = 0.0
        self.Fx = 0.0
        self.Fy = 0.0
        self.Max = 100.00
        self.Min = -100.00

        #グラフ描画用
        self.count = 0
        self.list_fx = []
        self.list_now = []
        self.list_count = []


    def sub_Goal_pose_CB(self, msg):
        self.Goal_pose = msg

    def sub_Target_pose_CB(self, msg):
        i = msg.name.index(self.Target_model)
        self.Target_pose = msg.pose[i]

    def Add_force(self, way):
        apply_body_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

        clear_body_wrenches = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)

        get_link_state = rospy.ServiceProxy("/gazebo/get_link_state",GetLinkState)

        target_link = self.Target_model + "::body"
        wrench = Wrench()
        duration = rospy.Duration(0.1) #単位はsecond

        if way == "front":
            wrench.force.x = self.Force_param
            wrench.force.y = 0
            wrench.force.z = 0
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = 0

        elif way == "back":
            wrench.force.x = -self.Force_param
            wrench.force.y = 0
            wrench.force.z = 0
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = 0

        elif way == "right":
            wrench.force.x = 0
            wrench.force.y = self.Force_param
            wrench.force.z = 0
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = 0

        elif way == "left":
            wrench.force.x = 0
            wrench.force.y = -self.Force_param
            wrench.force.z = 0
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = 0

        elif way == "turn_right":
            wrench.force.x = 0
            wrench.force.y = 0
            wrench.force.z = 0
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = -self.Torque_param

        elif way == "turn_left":
            wrench.force.x = 0
            wrench.force.y = 0
            wrench.force.z = 0
            wrench.torque.x = 0
            wrench.torque.y = 0
            wrench.torque.z = self.Torque_param

        elif way == "brake":
            res = get_link_state(link_name=target_link)
            if res.link_state.twist.linear.x < -0.001:
                wrench.force.x = self.Force_param * 100

            elif res.link_state.twist.linear.x > 0.001:
                wrench.force.x = -self.Force_param * 100

            if res.link_state.twist.linear.y < -0.001:
                wrench.force.y = self.Force_param * 100

            elif res.link_state.twist.linear.y > 0.001:
                wrench.force.y = -self.Force_param * 100

            if res.link_state.twist.angular.z < -0.001:
                wrench.torque.z = self.Torque_param

            elif res.link_state.twist.angular.z > 0.001:
                wrench.torque.z = -self.Torque_param

            else:
                pass

        else:
            pass

        try:
            apply_body_wrench(body_name=target_link,
            reference_frame=self.Reference_frame,
            wrench=wrench,
            duration=duration)

        except rospy.ServiceException as e:
            rospy.logerr("Service[/gazebo/apply_body_wrench] Exception")

    def Add_force_with_param(self, fx, fy, fz, tx, ty, tz):
        apply_body_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
        clear_body_wrenches = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)
        get_link_state = rospy.ServiceProxy("/gazebo/get_link_state",GetLinkState)

        target_link = self.Target_model + "::body"
        wrench = Wrench()
        duration = rospy.Duration(0.1) #単位はsecond

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

    def pid(self):
        #Kp = (0.60 * self.T) / (self.K * self.L) #0.60~0.95
        #Ki = 0.60 / (self.K * self.L) #0.60~0.70
        #Kd = (0.30 * self.T) / self.K #0.30~0.45

        Kp = 1.5
        Ki = 0.0
        Kd = 0.0

        dist_x = self.Goal_pose.position.x - self.Target_pose.position.x
        dist_y = self.Goal_pose.position.y - self.Target_pose.position.y

        fx = self.Fx + Kp * (dist_x-self.DistX_1) + Ki * dist_x + Kd * ((dist_x-self.DistX_1)-(self.DistX_1-self.DistX_2))
        fy = self.Fy + Kp * (dist_y-self.DistY_1) + Ki * dist_y + Kd * ((dist_y-self.DistY_1)-(self.DistY_1-self.DistY_2))

        """if fx > self.Max:
            fx = self.Max
        if fy > self.Max:
            fy = self.Max

        if fx < self.Min:
            fx = self.Min
        if fy < self.Min:
            fy = self.Min"""

        self.Add_force_with_param(fx,0.0,0.0,0.0,0.0,0.0)

        print "Goal:[%s,%s]"%(str(round(self.Goal_pose.position.x)),str(round(self.Goal_pose.position.y)))
        print "Now:[%s,%s]"%(str(round(self.Target_pose.position.x)),str(round(self.Target_pose.position.y)))
        print "Distance:[%s,%s]"%(str(round(dist_x)), str(round(dist_y)))
        print "fx:[%s], fy:[%s]"%(str(round(fx,2)),str(round(fy,2)))
        print "Counter:[%s]"%str(self.count)
        print "-----"

        self.Fx = fx
        self.Fy = fy
        self.DistX_1 = dist_x
        self.DistY_1 = dist_y
        self.DistX_2 = self.DistX_1
        self.DistY_2 = self.DistY_1

        self.count += 1
        self.list_fx.append(fx)
        self.list_now.append(self.Target_pose.position.x)
        self.list_count.append(self.count)

        if self.count == 10000:
            plt.plot(self.list_count, self.list_fx, color="blue")
            #plt.plot(self.list_count, self.list_now, color="green")
            #plt.axhline(self.Goal_pose.position.x, color="red")
            plt.axhline(0.0, color="black")
            plt.xlim(0,self.count)
            #plt.ylim(-10,30)
            #plt.pause(0.00001)  # 引数はsleep時間
            #plt.cla()  # 現在描写されているグラフを消去
            plt.title("Kp:[%s]"%str(Kp))
            plt.show()

    def processing(self):
        Goal_area_x = 2.0
        Goal_area_y = 0.0
        x_flag = False
        y_flag = False

        dist_x = self.Goal_pose.position.x - self.Target_pose.position.x
        dist_y = self.Goal_pose.position.y - self.Target_pose.position.y

        if abs(dist_x) > Goal_area_x:
            x_flag = False

        if abs(dist_y) > Goal_area_y:
            y_flag = False

        if x_flag == False or y_flag == False:
            if dist_x < Goal_area_x and dist_x > -1.0 * Goal_area_x:
                #cmd = "brake"
                x_flag = True

            if dist_y < Goal_area_y and dist_y > -1.0 * Goal_area_y:
                #cmd = "brake"
                y_flag = True

            if dist_y > Goal_area_y:
                cmd = "right"
                y_flag = False

            if dist_y < -1.0 * Goal_area_y:
                cmd = "left"
                y_flag = False

            if dist_x > Goal_area_x: #front
                cmd = "front"
                x_flag = False


            if dist_x < -1.0 * Goal_area_x: #back
                cmd = "back"
                x_flag = False

            else:
                #rospy.logwarn("warning")
                #cmd = "brake"
                pass

        if x_flag == True and y_flag == True:
            #Force("brake")
            cmd = "brake"
            rospy.loginfo("Arrived Goal Point")

        else:
            pass

        self.Add_force(cmd)

        print "-------------------------------------"
        print ("Goal Location:[%s][%s]"%(str(round(self.Goal_pose.position.x)), str(round(self.Goal_pose.position.y))))
        print ("Present Location:[%s][%s]"%(str(round(self.Target_pose.position.x)), str(round(self.Target_pose.position.y))))
        print ("Distance:[%s][%s]"%(str(round(dist_x)), str(round(dist_y))))
        print ("Arrived:x[%s] y[%s]"%(str(x_flag), str(y_flag)))
        print ("Command:[%s]"%(cmd))


    def main(self):
        while not rospy.is_shutdown():
            #self.processing()
            self.pid()
            rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node("RibbonBridgeSelfMove4")
    s = Self_Move()
    s.main()
    rospy.spin()

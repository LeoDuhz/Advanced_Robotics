#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import LaserScan
import math
from dwa import DWAPlanner

from threading import Lock,Thread
import time

def limitVal(minV,maxV,v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v

class LocalPlanner:
    def __init__(self):
        self.arrive = 0.1
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        # init plan_config for once
        self.dwa = DWAPlanner()
        self.threshold = self.dwa.max_v_speed*self.dwa.predict_time
        self.thres = 0.3

        self.laser_lock = Lock()
        self.lock = Lock()
        self.path = Path()

        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/webService/cmd_vel',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan_emma_nav_front',LaserScan,self.laserCallback)
        self.planner_thread = None

        self.count = 0
        pass
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/base_footprint',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw

    def laserCallback(self,msg):
        self.laser_lock.acquire()
        # preprocess
        self.ob = [[100,100]]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)):
            a = angle_min + angle_increment*i
            r = msg.ranges[i]
            if r < 1.6 and r > 0.1:
                self.ob.append([math.cos(a)*r,math.sin(a)*r])
        self.laser_lock.release()
        pass

    def u2t(self, u):
        w = u[2]
        x = u[0]
        y = u[1]
        return np.array([[math.cos(w),-math.sin(w),x],[math.sin(w),math.cos(w),y],[0,0,1]])

    def updateObstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        self.plan_ob = np.array(self.ob)

        #do transformation from robot coordinate to global coordinate
        trans = self.u2t([self.x, self.y, self.yaw])
        temp = np.ones([len(self.plan_ob),1])
        self.plan_ob = np.concatenate((self.plan_ob,temp),axis=1)
        self.plan_ob = trans.dot(self.plan_ob.T)
        self.plan_ob = self.plan_ob.T
        self.plan_ob = self.plan_ob[:,0:2]

        self.laser_lock.release()
        pass
    def pathCallback(self,msg):
        self.path = msg
        self.lock.acquire()
        self.initPlanning()
        self.lock.release()
        # if self.planner_thread == None:
        #     self.planner_thread = Thread(target=self.planThreadFunc)
        #     self.planner_thread.start()
        # pass
        self.planThreadFunc()
    def initPlanning(self):
        self.goal_index = 0
        self.vx = 0.0
        self.vw = 0.0
        self.dis = 99999
        self.updateGlobalPose()
        cx = []
        cy = []
        for pose in self.path.poses:
            cx.append(pose.pose.position.x)
            cy.append(pose.pose.position.y)
        self.goal = np.array([cx[0],cy[0]])
        self.plan_cx,self.plan_cy = np.array(cx),np.array(cy)
        self.destination = np.array([cx[-1],cy[-1]])
        self.plan_goal = np.array([cx[-1],cy[-1]])
        self.plan_x = np.array([0.0,0.0,0.0,self.vx,self.vw])
        pass

    def planThreadFunc(self):
        print("running planning thread!!")
        i = 0
        for pose in self.path.poses:
            if i == 0:
                i = 1
                continue
            else:
                self.goal_node = np.array([pose.pose.position.x, pose.pose.position.y])

                goal = pose
                self.midpose_pub.publish(goal)
                self.goal_node = np.array([goal.pose.position.x,goal.pose.position.y])
                while True:
                    self.lock.acquire()
                    # start = time.time()
                    # self.planOnce()
                    end = time.time()
                    # print('this plan time cost:', end-start)
                    self.lock.release()
                    self.updateGlobalPose()
                    self.goal_dis = math.hypot(self.x-pose.pose.position.x, self.y-pose.pose.position.y)
                    print('goal_dis:', self.goal_dis)
                    if i == len(self.path.poses) - 1:
                        threshold = 0.3
                    else:
                        threshold = 0.4
                    if self.goal_dis < 0.4:
                        print("arrive one goal node!")
                    time.sleep(0.001)
                self.lock.acquire()
                self.publishVel(True)
                self.lock.release()
                self.planning_thread = None
            i += 1

        self.publishVel(True)
        print('at goal！')






    def planOnce(self):
        self.updateGlobalPose()
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.plan_x = [self.x,self.y,self.yaw,self.vx,self.vw]
        # Update obstacle
        self.updateObstacle()
        u = self.dwa.plan(self.plan_x, self.goal_node, self.plan_ob)
        alpha = 0.5
        self.vx = u[0]*alpha + self.vx*(1-alpha)
        self.vw = u[1]*alpha + self.vw*(1-alpha)
        print("v, w: ", self.vx, self.vw)
        # print("mdbg; ",u)
        self.publishVel()
        pass

    def publishVel(self,zero = False):
        if zero:
            self.vx = 0
            self.vw = 0
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('path_Planning')
    lp = LocalPlanner()
    rospy.spin()

if __name__ == '__main__':
    main()

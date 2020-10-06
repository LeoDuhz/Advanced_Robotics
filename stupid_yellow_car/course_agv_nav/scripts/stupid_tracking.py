#!/usr/bin/env python
import rospy
#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import LaserScan
from threading import Lock,Thread
import math
import time
class Tracking:
    def __init__(self):
        self.arrive = 0.1
        self.arrive_threshold = 0.3
        self.vx = 0.0
        self.vw = 0.0

        self.lock = Lock()
        self.path = Path()
        self.tf = tf.TransformListener()
        # self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        # self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        # self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        # self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/webService/cmd_vel',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laserCallback)
        self.tracking_thread = None
        pass
    def laserCallback(self,msg):
        self.received = True
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

        p = self.path.poses[self.goal_index].pose.position
        dis = math.hypot(p.x-self.x,p.y-self.y)
        if dis < self.arrive_threshold and self.goal_index < len(self.path.poses)-1:
            self.goal_index = self.goal_index + 1
        self.midpose_pub.publish(self.path.poses[self.goal_index])
        self.goal_dis = math.hypot(self.x-self.path.poses[-1].pose.position.x,self.y-self.path.poses[-1].pose.position.y)

    def pathCallback(self,msg):
        # print("get path msg!!!!!",msg)
        self.path = msg
        self.lock.acquire()
        self.initTracking()
        self.lock.release()
        if self.tracking_thread == None:
            self.tracking_thread = Thread(target=self.trackThreadFunc)
            self.tracking_thread.start()
        pass
    def initTracking(self):
        self.goal_index = 0
        self.updateGlobalPose()
        pass
    def trackThreadFunc(self):
        print("running track thread!!")
        # while self.plan_lastIndex > self.plan_target_ind:
        while True:
            time.sleep(0.1)
            if self.received == False:
                continue
            self.received = False
            self.planOnce()
            if self.goal_dis < self.arrive:
                print("arrive goal!")
                break
        print("exit track thread!!")
        self.lock.acquire()
        self.publishVel(True)
        # time.sleep(0.001)
        self.lock.release()
        self.tracking_thread = None
        pass
    def planOnce(self):
        self.lock.acquire()

        self.updateGlobalPose()

        target = self.path.poses[self.goal_index].pose.position

        dx = target.x - self.x
        dy = target.y - self.y

        target_angle = math.atan2(dy, dx)

        per_step = 0.001
        self.vw = self.pi2pi(target_angle-self.yaw)/2.0
        self.vw = self.clip(self.vw,-0.3,0.3)

        if abs(self.vw) > 0.1:
            self.vx = self.clip(0,self.vx-per_step,self.vx+per_step)
        else:
            self.vx = self.clip(math.hypot(dx,dy)/2.0,self.vx-per_step,self.vx+per_step)
        self.vx = self.clip(self.vx,0,0.5)
        self.publishVel()

        self.lock.release()
        pass
    def clip(self,v,minv,maxv):
        if v < minv:
            return minv
        if v > maxv:
            return maxv
        return v

    def pi2pi(self,angle):
        a = angle%(2*math.pi)
        if a > math.pi:
            a = a - 2*math.pi
        return a

    def publishVel(self,zero = False):
        cmd = Twist()
        print("v, w", self.vx, self.vw)
        # self.vx, self.vw = 0, 0
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        if zero:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('stupid_tracking')
    t = Tracking()
    rospy.spin()

def test(t):
    rx = [0,1,2,3,4,5,6,7]
    ry = [1,0,1,0,1,0,1,0]
    path = Path()
    path.header.seq = 0
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'map'
    for i in range(len(rx)):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'map'
        pose.pose.position.x = rx[i]
        pose.pose.position.y = ry[i]
        pose.pose.position.z = 0.01
        pose.pose.orientation.x = 0#self.rot[0]
        pose.pose.orientation.y = 0#self.rot[1]
        pose.pose.orientation.z = 0#self.rot[2]
        pose.pose.orientation.w = 1#self.rot[3]
        path.poses.append(pose)
    pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 10)
    time.sleep(0.5)
    pub.publish(path)
    print("publish path!!!!!")
    # t.pathCallback(path) 

if __name__ == '__main__':
    main()
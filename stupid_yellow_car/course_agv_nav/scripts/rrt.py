from scipy.spatial import KDTree
import numpy as np
import random
import math
import time

"""
RRT path planning implementation with python
"""
class RRTPlanner():
    def __init__(self, ox, oy, robot_radius, avoid_buffer,minx,miny,maxx,maxy):
        self.obstacle_x = ox
        self.obstacle_y = oy
        
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        self.robot_size = robot_radius
        self.avoid_dist = avoid_buffer
        self.rang=1
        self.Nr=1000
        self.delta=1.5
        self.MAX_EDGE_LEN = maxx-minx
        pass

    def plan(self, start_x, start_y, goal_x, goal_y):
        # Obstacles
        
        # Obstacle KD Tree
        # print(np.vstack((obstacle_x, obstacle_y)).T)
        obstree = KDTree(np.vstack((self.obstacle_x, self.obstacle_y)).T)
        # Building tree
        print('c')
        pt_x, pt_y,pt_f,endnum = self.building(start_x, start_y, goal_x, goal_y, obstree)
        print('d')

        # Search Path
        path_x, path_y = self.backpath(pt_x,pt_y,pt_f,endnum)
        path_x,path_y=self.shrink(path_x,path_y,obstree)
        path_x.reverse()
        path_y.reverse()

        return path_x, path_y
        
    def building(self, start_x, start_y, goal_x, goal_y, obstree):
        pt_x, pt_y,pt_f = [], [],[]
        pt_x.append(start_x)
        pt_y.append(start_y)
        pt_f.append(-1)
        for count in range(self.Nr):
            tx = (random.random() * (self.maxx - self.minx)) + self.minx
            ty = (random.random() * (self.maxy - self.miny)) + self.miny

            #distance, index = obstree.query(np.array([tx, ty]))

            #if distance >= self.robot_size + self.avoid_dist:
             #   continue
            ptstree=KDTree(np.vstack((pt_x, pt_y)).T)
            distance, index = ptstree.query(np.array([tx, ty]))
            qnearst_x=pt_x[index]
            qnearst_y=pt_y[index]
            if math.hypot(qnearst_x-goal_x,qnearst_y-goal_y)<self.rang:
                return pt_x,pt_y,pt_f,index
            dx = tx-qnearst_x
            dy = ty-qnearst_y
            angle = math.atan2(dy,dx)
            
            
            qnew_x=self.delta*math.cos(angle)+qnearst_x
            qnew_y=self.delta*math.sin(angle)+qnearst_y
            
            if not self.check_obs(qnearst_x,qnearst_y,qnew_x,qnew_y,obstree):
                pt_x.append(qnew_x)
                pt_y.append(qnew_y)
                pt_f.append(index)

            if count == self.Nr-1:
                print('ERROR!')
        endnum=-1
        return pt_x,pt_y,pt_f,endnum


        
    def backpath(self,pt_x,pt_y,pt_f,endnum):
        path_x,path_y=[],[]
        while endnum>=0:
            path_x.append(pt_x[endnum])
            path_y.append(pt_y[endnum])
            endnum=pt_f[endnum]
        
        return path_x,path_y

    def shrink(self,path_x,path_y,obstree):
        length=len(path_x)
        i=length-1
        while i > 1:
            if not self.check_obs(path_x[i],path_y[i],path_x[i-2],path_y[i-2],obstree):
                path_x.remove(path_x[i-1])
                path_y.remove(path_y[i-1])
            
            i=i-1

        return path_x,path_y

    def check_obs(self, ix, iy, nx, ny, obstree):
        x = ix
        y = iy
        dx = nx - ix
        dy = ny - iy
        angle = math.atan2(dy, dx)
        dis = math.hypot(dx, dy)

        if dis > self.MAX_EDGE_LEN:
            return True

        step_size = self.robot_size + self.avoid_dist
        steps = int(round(dis/step_size))
        # print(dis)
        # print(step_size)
        # print(steps)
        for i in range(steps):
            distance, index = obstree.query(np.array([x, y]))
            if distance <= self.robot_size + self.avoid_dist:
                return True
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)

        # check for goal point
        distance, index = obstree.query(np.array([nx, ny]))
        if distance <= self.robot_size + self.avoid_dist:
            return True

        return False

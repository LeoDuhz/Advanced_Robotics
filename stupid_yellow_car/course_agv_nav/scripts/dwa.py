import math
import numpy as np
import rospy
import tf

"""
Dynamic window approach implementation with python
Reference: The Dynamic Window Approach to Collision Avoidance
https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf
"""
class DWAPlanner():
    def __init__(self):
        # robot parameter
        self.min_v_speed = 0
        self.max_v_speed = 0.8  # [m/s]
        self.min_w_speed = -0.5
        self.max_w_speed = 0.5
        self.predict_time = 2  # [s]
        self.va = 0.2
        self.wa = 0.1
        self.v_step = 0.2
        self.w_step = 0.1
        self.radius = 0.15
        self.delta_t = 0.4
        self.alpha = 4         #goal
        self.beta = 1           #velocity
        self.gamma = 2    #ob
        self.avoid_size = 0.05

    def plan(self, x, goal, ob):
        u = x[3:5]
        u = self.dwa_search(x,u,goal,ob)
        return u

    def dwa_search(self, X, u, goal, obstacles):
        vw=self.calculate_vw_range(X)
        min_score = 10000000000
        i = 0
        print('vw[0],vw[1]:', vw[0], vw[1])
        print('vw[2],vw[3]:', vw[2], vw[3])

        for v in np.arange(vw[0], vw[1], self.v_step):    
            for w in np.arange(vw[2], vw[3], self.w_step): 
                traj = self.predict_trajectory(X,[v,w])
                # if self.check_collision(traj,obstacles) == True:
                #     print('check_collision == True')
                #     continue 
                goal_score = self.heading_obj_func(traj, goal)
                vel_score = self.velocity_obj_func(traj)
                obs_score = self.clearance_obj_func(traj,obstacles)
                score = self.alpha * goal_score + self.beta * vel_score + self.gamma * obs_score
                if score <= min_score:                   
                    min_score = score
                    u = np.array([v,w])
                i += 1
        return u

    def calculate_vw_range(self, X):
        v_min = X[3] - self.va * self.predict_time          
        v_max = X[3] + self.va * self.predict_time         
        w_min = X[4] - self.wa * self.predict_time          
        w_max = X[4] + self.wa * self.predict_time         

        VW = [max(self.min_v_speed,v_min),min(self.max_v_speed,v_max),max(self.min_w_speed,w_min),min(self.max_w_speed,w_max)]
        return VW

    def Motion(self,X,u):
        X[0]+=u[0] * self.delta_t * math.cos(X[2])           
        X[1]+=u[0] * self.delta_t * math.sin(X[2])           
        X[2]+=u[1] * self.delta_t                     
        X[3]=u[0]                         
        X[4]=u[1] 
        return X

    def predict_trajectory(self, x_init, u):
        """
        Predict trajectory: return trajectory in predict time
        """
        Traj = np.array(x_init)
        Xnew = np.array(x_init)
        
        time=0
        while time <= self.predict_time:
            Xnew=self.Motion(Xnew,u)

            time = time + self.delta_t
            Traj = np.vstack((Traj,Xnew))
        return Traj

        pass

    def check_collision(self, trajectory, ob):
        """
        Check Collision: return true if collision happens
        """
        
        for i in range(len(trajectory)):
            for j in range(len(ob)):
                dist = math.sqrt((trajectory[i,0] - ob[j,0]) ** 2 + (trajectory[i,1] - ob[j,1]) ** 2)

                if dist <= self.radius + self.avoid_size:
                    print('dist:', dist)
                    return True

        return False

        pass

    def heading_obj_func(self, trajectory, goal):
        """
        Target heading: heading is min_v_speed measure of progress towards the goal location.
        It is maximal if the robot moves directly towards the target.
        """
        return math.sqrt((trajectory[-1,0] - goal[0]) ** 2 + (trajectory[-1,1] - goal[1]) ** 2)
        pass

        # angle1 = math.atan2(goal[1]-trajectory[-1,1], goal[0]-trajectory[-1,0])
        # angle2 = trajectory[-1,2]
        # delta = abs(angle1-angle2)
        # if abs(angle1-angle2)>math.pi:
        #     delta = 2*math.pi - delta

        # return delta
        
    def clearance_obj_func(self, trajectory, ob):
        """
        Clearance: dist is the distance to the closest obstacle on the trajectory.
        The smaller the distance to an obstacle the higher is the robot's desire to move around it.
        """
        min_dist = 1000000000000
        # dist_array = np.array([])
        for i in range(len(trajectory)):
            for j in range(len(ob)):
                dist = math.sqrt((trajectory[i,0] - ob[j,0]) ** 2 + (trajectory[i,1] - ob[j,1]) ** 2)
                if dist < self.radius:
                    return float('Inf')

                # dist_array = np.append(dist_array, dist)

                if dist < min_dist:
                    min_dist = dist
                    flag = j

        return 1/min_dist
        pass

    def velocity_obj_func(self, trajectory):
        """
        Velocity: vel is the forward velocity of the robot and supports fast movements.
        """
        return self.max_v_speed - trajectory[-1,3]
        pass

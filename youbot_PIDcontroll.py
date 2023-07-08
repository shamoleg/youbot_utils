#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Twist
import rospy, math
from PID import PIDcontroller


class Youbot:
    def __init__(self, robotID: str):
        rospy.init_node('youbot_controll')
        self.robotID = robotID
        self.cur_pose = PointStamped()
        self.pub_cmd_vel = rospy.Publisher('/base/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/marker_pose', PointStamped, self.callback_pose)
        self.pid = PIDcontroller()

        #velocity params
        self.velMax = 0.3
        self.velMin = 0.02

    def callback_pose(self, data: PointStamped):
        if data.header.frame_id == self.robotID:
            self.cur_pose = data

    def set_goal(self, goal_x, goal_y, threshold=0.005):
        vel = Twist()
        coef_P = 0.1

        error_x = goal_x - self.cur_pose.point.x
        while abs(error_x) > abs(threshold):
            vel.linear.x = coef_P * error_x
            error_x = goal_x - self.cur_pose.point.x
            self.pub_cmd_vel.publish(vel)

        # error_y = goal_y - self.cur_pose.point.y
        # while abs(error_y) > abs(threshold):
        #     vel.linear.y = coef_P * error_y
        #     error_y = goal_y - self.cur_pose.point.y
        #     self.pub_cmd_vel.publish(vel)

        self.pub_cmd_vel.publish(Twist())

    def set_goal_with_pid(self, goal_x: float, goal_y: float):
        '''Calculate speeds, time for robot using PID-controller \n
        goal_x, goal_y: in meters'''
        vel = Twist()

        vectors = self.vector_to_goal(goal_x, goal_y)
        abs_v, _ = self.pid.update(self.velMax, 0) #absolute speed
        dec_way = self.decseleration_way() #distance to stop moving

        while vectors[2] >= 0.005:
            try:
                tau = vectors[2] / abs_v #time for calculation speeds
            except ZeroDivisionError:
                tau = 0.000001
            v_x = vectors[0] / tau #speed in x-axis
            v_y = vectors[1] / tau #speed in y-axis
            vel.linear.x = v_x
            vel.linear.y = v_y

            self.pub_cmd_vel.publish(vel)

            if vectors[2] <= dec_way:
                abs_v, _ = self.pid.update(self.velMin, abs_v)
            else:
                abs_v, _ = self.pid.update(self.velMax, abs_v)
            vectors = self.vector_to_goal(goal_x, goal_y)
            
        acselerations = self.calculate_acselerations(vector, 0.3, 0.01)
        vel = Twist()
        self.pub_cmd_vel.publish(vel)

    def decseleration_way(self):
        abstract_pid =  PIDcontroller(self.pid.Kp, self.pid.Ki, self.pid.Kd)
        v, t = abstract_pid.update(0.0, self.velMax)
        while v > self.velMin+0.001:
            v, t = abstract_pid.update(self.velMin, v)
        s = self.velMax * t * 0.5
        print(s)
        return s
            
    def vector_to_goal(self, goal_x, goal_y):
        '''Calculate vector to goal from current position
        \nReturns a tuple of vector_x, vector_y, vector_module'''
        vector_x = goal_x - self.cur_pose.point.x
        vector_y = goal_y - self.cur_pose.point.y
        vector_module = math.sqrt(vector_x**2 + vector_y**2)
        return (vector_x, vector_y, vector_module)
    
    def max_simulation_time(self, vector: tuple):
        '''Calculate max time needed for motion to goal place \n
          vector: (x, y, module) \n
          return max sample time according to min velocity'''
        return vector[2] / self.velMin
    
    def calculate_acselerations(self, vector: tuple, max_dist: float, dt: float):
        '''Calculate speeds for current vector \n
        vector: (x, y, module) \n
        max_dist: distance for acseleration and decseleration \n
        dt: discretisation of time \n'''
        
        '''From S = (v2**2-v1**2)/2a get a'''
        self.a = (self.velMax**2 - self.velMin**2) / (2 * max_dist)

        '''From S = Vt + 0.5at^2 get t for acseleration and decseleration
        0.5 * a * t^2 + V * t - S = 0
        D = V**2 - 0.5*a*S
        t = [-V +- sqrt(D)] / a'''
        if (max_dist >= vector[2] / 2):
            D = self.velMin**2 - 0.5*a*vector[2]/2
        else:
            D = self.velMin**2 - 0.5*a*max_dist
        t1 = [-self.velMin + math.sqrt(D)] / a
        t2 = [-self.velMin - math.sqrt(D)] / a
        t_a = max(t1,t2) #Time for acseleration and decseleration
        
        '''From S = Vmax*t get t for uniform motion'''
        if (max_dist < vector[2] / 2):
            t_m = (vector[2] - (2 * max_dist)) / self.velMax
        else:
            t_m = 0
        
        if (2*t_a + t_m) >= self.max_simulation_time(vector):
            raise TimeoutError('Time for motion out of sample time')
        
        t_a_error = t_a % dt #error of discretisation for ac/decseleration
        n_a = t_a // dt #nums of iteration for ac/decseleration
        t_m_error = t_a % dt #error of discretisation for uniform motion
        n_m = t_m // dt #nums of iteration for uniform motion

        return (n_a, n_m)
        

        
if __name__ == "__main__":
    u = Youbot('\x01')
    rospy.sleep(0.5)
    u.set_goal_with_pid(0.3, 0.3)
    print(u.cur_pose)
    u.pub_cmd_vel.publish(Twist())
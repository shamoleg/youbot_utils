#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Twist
import rospy
import numpy as np
from scipy import integrate



class PID:
    def __init__(self, input_sig, back_sig = None, Ki=1, Kp = 1, Kd =1):
        self.input_sig = input_sig
        self.Ki = Ki
        self.Kp = Kp
        self.Kd = Kd
        self.back_sig = back_sig
        self.output_sig = None

    def func(self, put):
        return self.Kp*put
    
    def getBack_sig(self):
        #self.back_sig = чему-то
        pass

    def calculate(self):
        output_sig = self.func(self.input_sig)
        if self.back_sig is not None:
            output_sig = self.input_sig - self.back_sig
        else:
            self.output_sig = output_sig
            return 
        self.output_sig = output_sig
        return output_sig


class Youbot:
    def __init__(self, robotID: str):
        self.robotID = robotID
        self.cur_pose = PointStamped()
        self.pub_cmd_vel = rospy.Publisher('/base/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/marker_pose', PointStamped, self.callback_pose)

    def callback_pose(self, data: PointStamped):
        self.cur_pose = data

    def setGoal(self, goal_x, goal_y, threshold=0.005):
        vel = Twist()
        coef_P = 0.1
        error_x = goal_x - self.cur_pose.point.x
        while abs(error_x) > abs(threshold):
            vel.linear.x = coef_P * error_x
            error_x = goal_x - self.cur_pose.point.x
            self.pub_cmd_vel.publish(vel)

        self.pub_cmd_vel.publish(Twist())

if __name__ == "__main__":
    # u = Youbot('\x01')
    # u.setGoal(0.1, 0)
    # u.setGoal(0.1, 0)
    # u.setGoal(-0.1, 0)
    # u.setGoal(0.1, 0)
    # u.setGoal(-0.1, 0)

    p = PID(2, Kp=0.8) 
    p.calculate()
    print(p.output_sig)
    p.back_sig = p.output_sig*1.25
    p.calculate()
    print(p.output_sig)
    p.back_sig = p.output_sig*1.25
    p.calculate()
    print(p.output_sig)


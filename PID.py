#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Twist
import rospy

class PID:
    def __init__(self, Ki=1, Kp = 1, Kd =1):
        self.Ki = Ki
        self.Kp = Kp
        self.Kd = Kd
        self.errorSum = 0
        self.cur_error = None
        self.prev_error = None

    def PIDtransferFunc(self): #функция для вычисления ПИД-регулирования
        P = self.Kp * self.cur_error
        I = self.Ki * self.errorSum
        D = self.Kd * self.derrdt()
        print(f'P: {P}')
        print(f'I: {I}')
        print(f'D: {D}')
        return P+I+D
    
    def derrdt(self, dt=1): # dt - время дискретизации
        if (self.prev_error is not None) and (self.cur_error is not None):
            return (self.cur_error - self.prev_error)/dt
        else:
            return 0
    
    def calculate(self, input_sig, back_sig): #вычисление выходного сигнала после ПИД-регулирования
        error_sig = input_sig - back_sig
        self.errorSum += error_sig
        if self.cur_error is None:
            self.cur_error = error_sig
        else:
            self.prev_error = self.cur_error
            self.cur_error = error_sig
        return self.PIDtransferFunc()


class Youbot:
    def __init__(self, robotID: str):
        self.robotID = robotID
        self.cur_pose = PointStamped()
        self.pub_cmd_vel = rospy.Publisher('/base/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/marker_pose', PointStamped, self.callback_pose)
        self.pid = PID()

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

    def setGoalwithPID(self, goal_X, goal_y):
        vel = Twist()

if __name__ == "__main__":
    # u = Youbot('\x01')
    # u.setGoal(0.1, 0)
    # u.setGoal(0.1, 0)
    # u.setGoal(-0.1, 0)
    # u.setGoal(0.1, 0)
    # u.setGoal(-0.1, 0)

    p = PID(Ki = 0.0001, Kp=200, Kd = 0)
    output = p.calculate(2, 1.99)
    print(output)
    output = output*0.995
    output = p.calculate(2, output)
    print(output)
    output = output*0.995
    output = p.calculate(2, output)
    print(output)


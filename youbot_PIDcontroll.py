#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Twist
import rospy
from PID import PIDcontroller, test_func


class Youbot:
    def __init__(self, robotID: str):
        self.robotID = robotID
        self.cur_pose = PointStamped()
        self.pub_cmd_vel = rospy.Publisher('/base/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/marker_pose', PointStamped, self.callback_pose)
        self.pid = PIDcontroller(0, 1, 0)

        #velocity params
        self.velMax = 0.3
        self.velMin = 0.01

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

        # error_y = goal_y - self.cur_pose.point.y
        # while abs(error_y) > abs(threshold):
        #     vel.linear.y = coef_P * error_y
        #     error_y = goal_y - self.cur_pose.point.y
        #     self.pub_cmd_vel.publish(vel)

        self.pub_cmd_vel.publish(Twist())

    def setGoalwithPID(self, goal_x, goal_y):
        vel = Twist()
        coef_P = 0.1

        while abs(round(self.pid.output, 4)) != abs(round(goal_x, 4)):
            vel.linear.x = self.pid.updatePID(goal_x, self.cur_pose.point.x)*coef_P
            if abs(vel.linear.x) > self.velMax:
                vel.linear.x = self.velMax * (goal_x/abs(goal_x))
            elif abs(vel.linear.x) < self.velMin:
                vel.linear.x = self.velMin * (goal_x/abs(goal_x))
            self.pub_cmd_vel.publish(vel)

        # while abs(round(self.pid.output, 4)) != abs(round(goal_y, 4)):
        #     vel.linear.y = self.pid.updatePID(goal_y, self.cur_pose.point.y)*coef_P
        #     if abs(vel.linear.y) > self.velMax:
        #         vel.linear.y = self.velMax * (goal_y/abs(goal_y))
        #     elif abs(vel.linear.y) < self.velMin:
        #         vel.linear.y = self.velMin * (goal_y/abs(goal_y))
        #     self.pub_cmd_vel.publish(vel)

        self.pub_cmd_vel.publish(Twist())



if __name__ == "__main__":
    u = Youbot('\x01')
    u.setGoalwithPID(0.1, 0)
    u.setGoalwithPID(0.1, 0)
    u.setGoalwithPID(-0.1, 0)
    u.setGoalwithPID(0.1, 0)
    u.setGoalwithPID(-0.1, 0)
    pass

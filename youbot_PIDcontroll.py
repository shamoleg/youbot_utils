#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Twist
import rospy


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
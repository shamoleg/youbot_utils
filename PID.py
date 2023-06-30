#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Twist
import rospy


class Youbot:
    def __init__(self, robotID: str):
        self.robotID = robotID
        self.cur_x = None
        self.cur_y = None
        self.cur_z = None
        self.updatepose()
        self.velPub = rospy.Publisher('/base/cmd_vel', Twist, queue_size=1)

    def _callbackpose(self, data: PointStamped):
        self.cur_x = round(data.point.x, 2)
        self.cur_y = round(data.point.y, 2)
        self.cur_z = round(data.point.z, 2)
               
    def updatepose(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/marker_pose', PointStamped, self._callbackpose)
        rospy.sleep(0.05)

    def goTo(self, x, y):
        self.updatepose()
        vel = Twist()
        while round(x, 2) != self.cur_x:
            if x > self.cur_x:
                coef = 1
            elif x < self.cur_x:
                coef = -1
            else:
                coef = 0
            vel.linear.x = 0.1*coef
            self.velPub.publish(vel)
            self.updatepose()
        vel = Twist()
        self.velPub.publish(vel)

    def setGoal(self, x, y, treshold=0.005):
        try:
            vel = Twist()
            self.updatepose()
            start_x = self.cur_x
            start_y = self.cur_y
            t1 = rospy.Time.now()
            while True:
                if x > 0:
                    if (abs(self.cur_x) - abs(start_x) < x+treshold) and (abs(self.cur_x) - abs(start_x) > x-treshold):
                        break
                    coef = 1
                elif x < 0:
                    if (abs(start_x) - abs(self.cur_x) < abs(x)+treshold) and (abs(start_x) - abs(self.cur_x)  > abs(x)-treshold):
                        break
                    coef = -1
                else:
                    coef = 0
                    break
                vel.linear.x = 0.05*coef
                self.velPub.publish(vel)
                self.updatepose()
                print('cur_x: ' + str(self.cur_x))
        finally:
            vel = Twist()
            self.velPub.publish(vel)
            print(self.cur_x)




u =Youbot('\x01')
u.setGoal(0.1, 0)
u.setGoal(0.1, 0)
u.setGoal(-0.1, 0)
u.setGoal(0.1, 0)
u.setGoal(-0.1, 0)
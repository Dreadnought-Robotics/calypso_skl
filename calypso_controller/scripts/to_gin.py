#!/usr/bin/python3

import rospy

from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
from calypso_msgs.msg import gin

class gins:
    def __init__(self):
        rospy.init_node('gin', anonymous=False)
        self.rate = rospy.Rate(50)
        rospy.Subscriber("/rosetta/gypseas", gypseas, self.get_gyp)
        rospy.Subscriber("/rosetta/dolphins", dolphins, self.get_dol)
        self.gin_publisher = rospy.Publisher("rosetta/gin", gin, queue_size=10)

        self.t1 = self.t2 = self.d1 = self.d2 = 0

    def get_gyp(self, data):
        self.t1 = data.t1 
        self.t2 = data.t2

    def get_dol(self, data):
        self.d1 = data.d1
        self.d2 = data.d2

if __name__=='__main__':
  try:
    x = gins()

    while not rospy.is_shutdown():
       
        g = gin()
        g.t1 = x.t1
        g.t2 = x.t2
        g.d1 = x.d1
        g.d2 = x.d2
        x.gin_publisher.publish(g)
        x.rate.sleep()
  except rospy.ROSInterruptException:
      pass
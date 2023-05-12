#! /usr/bin/python3
import rospy
from calypso_msgs.msg import gypseas
from sensor_msgs.msg import Imu
from pid_calypso import pid as PID
from geometry_msgs.msg import Quaternion
import time 
# from scipy.integrate import trapezoid

class pid_gypseas:
  
  def __init__(self):

    rospy.init_node('gypseas_pid', anonymous=False)


    self.roll=PID()
    self.yaw=PID()
    # self.heave=PID()

    self.throttle1 = 0
    self.throttle2 = 0
    self.throttle3 = 0
    self.throttle4 = 0

    self.yaw.k=[0,0,0]
    self.roll.k=[0,0,0]
    # self.heave.k=[11,0.165,7.95]

    # self.heave_temp = 0 
    # self.heave.final = 3

    # self.m = interp1d([0, 90],[1582,2000])
    # self.n = interp1d([-90, 0], [1500,1582])

    self.rate = rospy.Rate(50)
    self.gypseas_publisher = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=10)
    self.base_subscriber = rospy.Subscriber('/controller/gypseas', gypseas, self.getgyp)
    self.pid_constants = rospy.Subscriber('/calypso_sim/constants', Quaternion, self.get_constants)
    # self.heave_subscriber = rospy.Subscriber('/calypso_sim/pose', Quaternion, self.get_current_pose)

    self.start_time = time.time()
  
  def gypseas_pid(self):

    PID_yaw = PID.getPID(self.yaw,True)
    PID_roll = PID.getPID(self.roll,True)
    # PID_heave = PID.getPID(self.heave, False)

    # # if we wanna map linear motiion: 
    # PID_heave_ = PID.getPID(self.heave, False)
    # if PID_heave_>=0:
    #   PID_heave = self.m(PID_heave_)
    # else:
    #   PID_heave = self.n(PID_heave_)
    # # for testing gyro PID, comment the PID_heave part.

    #uncomment for gyro testing
    # PID_heave = 1580
    print("base_throttle: ", self.throttle1)
    self.g=gypseas()
    self.g.t1 = round(self.throttle1 + PID_roll)
    self.g.t2 = round(self.throttle2 - PID_roll)
    self.g.t3 = round(self.throttle3 - PID_yaw)
    self.g.t4 = round(self.throttle4 + PID_yaw)

    self.gypseas_publisher.publish(self.g)
    self.rate.sleep()
      
  def Imu_subscriber(self, Imu):

    time_elapsed =  time.time()-self.start_time
    self.yaw.time.append(time_elapsed)
    self.roll.time.append(time_elapsed)
    # self.heave.time.append(time_elapsed)
    # self.heave.acc.append(Imu.linear_acceleration.z)

    self.roll.current_vel= Imu.angular_velocity.x
    self.yaw.current_vel = Imu.angular_velocity.z
    self.roll.current_position,pitch,self.yaw.current_position= PID.convert(Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w)
    # self.heave.current_position = self.heave_temp
    #print("current_position : ", self.heave.current_position)
    print("roll: ", self.roll.current_position)
    print("yaw: ", self.yaw.current_position)
    self.gypseas_pid()
    
  
  # def Heading_subscriber(self,pose):
  #   self.heave.final=pose.z

  def getgyp(self, gypseas):
    self.throttle1 = gypseas.t1
    self.throttle2 = gypseas.t2
    self.throttle3 = gypseas.t3
    self.throttle4 = gypseas.t4

  def get_constants(self, const):
    self.roll.k[0] = const.x
    self.roll.k[1] = const.y
    self.roll.k[2] = const.z

  # def get_current_pose(self, pose):
  #   self.heave_temp = pose.z

  
  def start(self):
    
    try:
      IMU = rospy.Subscriber("imu/data", Imu, self.Imu_subscriber)
      rospy.spin()
    except:
      print("ros shut down")

if __name__=='__main__':
  try:
      x = pid_gypseas()
      x.start()
  except rospy.ROSInterruptException:
      pass
# !usr/bin/env python3
import rospy
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
from scipy.interpolate import interp1d
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class control:
	
	def __init__(self):

		self.node=rospy.init_node("controller")

		self.publisher_gypseas = rospy.Publisher("/rosetta/gypseas", gypseas, queue_size=100)
		self.publisher_dolphins = rospy.Publisher("/rosetta/dolphins", dolphins, queue_size=100)
		# self.publisher_pid_values = rospy.Publisher("/calypso_sim/constants",Quaternion, queue_size=100)
		self.camera_switcher = rospy.Publisher("/calypso/camera_switch", Int8, queue_size=10) 
		self.rate = rospy.Rate(100000)
		self.camera_switch = 0
		self.stable_gypseas = 1500
		self.stable_dolphins=1500
		self.gypseas_min=1520
		self.dolphins_min=1530

		self.thrust_up_gypseas = interp1d([0, 1],[1500, 1600]) 
		self.thrust_down_gypseas=interp1d([-1, 0],[1300, 1500]) 
		self.yaw_left = interp1d([-1,1],[-20,20])
		# 1536 to 1800, 1200 to 1464


		self.thrust_up_dolphins = interp1d([0, 1],[1500, 1650])
		self.thrust_down_dolphins = interp1d([-1, 0],[1350, 1500])
		self.is_locked = False
		self.kp = 0
		self.ki = 0
		self.kd = 0
		self.last_gypseas=0
		self.q1=Quaternion()
		self.q1.x=self.q1.y=self.q1.z=self.q1.w =0
		# self.publisher_pid_values.publish(self.q1)
		self.g = gypseas()
		self.d = dolphins()
		self.q = Quaternion()
		self.front_cam = False
		self.bottom_cam = False
	#def load_mappings(self, ns):
        	#axes_remap = rospy.get_param(ns + "/axes", [])
        	#rospy.loginfo("loaded remapping: %d buttons, %d axes" % (len(btn_remap), len(axes_remap)))
        	#return {"buttons": btn_remap, "axes": axes_remap}
	def f(self,x):

		if x<0:
			x=self.stable_dolphins
		
		if x>260:
			x=260
		
		return x
	
	
	def JoyCallback(self, msg):
		

		self._deadzone = 0.05
		
		#self.d.d1 = self.d.d2=self.d.d3 =self.d.d4=0
		self.publisher_dolphins.publish(self.d)
		if rospy.has_param('~deadzone'):
			self._deadzone = float(rospy.get_param('~deadzone'))
		
		if msg is not None:
			# if (msg.buttons[9] == 1):
			# 	self.autonomy = not (self.autonomy)

			# if (self.autonomy == True):
			# 	self.autonomy_publisher.publish(self.autonomy)

			# else:
				if (msg.buttons[0] == 1):
					self.bottom_cam = True
					self.front_cam = False
				if (msg.buttons[1] == 1):
					self.front_cam = True
					self.bottom_cam = False
				
				if self.bottom_cam:
					self.camera_switch = 0
				if self.front_cam:
					self.camera_switch = 1
				self.camera_switcher.publish(self.camera_switch)
				###################################################################################################
				#################### toggle button / gypseas ######################################################

				if(msg.buttons[2]== 1):
					self.is_locked= not (self.is_locked)
					#print(self.is_locked)
				
				if self.is_locked:

					self.g.t1=self.g.t2=self.last_gypseas
				
				else :

					if (msg.axes[1] > 0 and abs(msg.axes[1]) > self._deadzone ):
							self.g.t1=self.g.t2 = self.last_gypseas = int(self.thrust_up_gypseas(msg.axes[1]))

					# if (msg.axes[1] == 0 and abs(msg.axes[1]) > self._deadzone ):
					# 		self.g.t1=self.g.t2 = self.stable_gypseas	
					
					elif (msg.axes[1] < 0 and abs(msg.axes[1]) > self._deadzone):
							self.g.t1 = self.g.t2 = self.last_gypseas = int(self.thrust_down_gypseas(msg.axes[1]))

					else:
						self.g.t1=self.g.t2 = self.last_gypseas = self.stable_gypseas

					if(msg.buttons[7]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
						self.g.t1 = self.g.t2=0



				self.publisher_gypseas.publish(self.g)
					
				########################################################################################################
				#################### dolphins ##########################################################################

				#### yaw hard left - LT ####


				#### stop dolphins ####
				if(msg.buttons[10]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
						self.d.d2=1500 #592 rpm
						self.d.d1=1500 #597 rpm
				#### forward ####
				if msg.axes[4] > 0 and abs(msg.axes[4]) > self._deadzone:
						self.d.d1 = self.d.d2 = int(self.thrust_up_dolphins(msg.axes[4]))
				
				#### backward ####
				elif msg.axes[4] < 0 and abs(msg.axes[4]) > self._deadzone:
						self.d.d1 = self.d.d2 = int(self.thrust_down_dolphins(msg.axes[4]))

				else:
					self.d.d1 = self.d.d2 = 1500

				#### turn ####
				# if msg.axes[3] and abs(msg.axes[3]) > self._deadzone:
				# 	self.d.d1 = self.d.d1 - int(self.yaw_left(msg.axes[3]))
				# 	self.d.d2 = self.d.d1 + int(self.yaw_left(msg.axes[3]))	
				
				if(msg.buttons[4]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
						while self.d.d1>1440 and self.d.d2<1560:
							self.d.d1=self.d.d1 - 1 #597 rpm
							self.d.d2=self.d.d2 + 1 #592 rpm
							self.publisher_dolphins.publish(self.d)

				#### yaw hard right- RT ####
				if(msg.buttons[5]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
						while self.d.d2>1440 and self.d.d1<1560:
							self.d.d2=self.d.d2 - 1 #597 rpm
							self.d.d1=self.d.d1 + 1 #592 rpm
							self.publisher_dolphins.publish(self.d)
						
				self.publisher_dolphins.publish(self.d)
				#########################################################################################################
				############### Kp Ki Kd    #############################################################################
				#Kp - Button A
				# if(msg.buttons[0]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
				# 	self.kp+=0.1
				# #Ki - Button - B
				# if(msg.buttons[1]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
				# 	self.ki+=0.1
				# #Kd - Button Y
				# if(msg.buttons[3]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
				# 	self.kd+=0.1
				
				# self.q.x = self.kp
				# self.q.y = self.ki
				# self.q.z = self.kd
				# self.q.w = 0.0
				# self.publisher_pid_values.publish(self.q)
				# self.rate.sleep()
		else:
			print("no connection") 
		
	
	def start(self):

		rospy.Subscriber('/joy', Joy, self.JoyCallback)
		rospy.spin()
               		 
if __name__=='__main__':
    try:
        x = control()
        x.start()
    except rospy.ROSInterruptException:
        pass

		
		

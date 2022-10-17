#!/usr/bin/env python3

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		rospy.init_node('drone_control')
		
		self.drone_position = [0.0,0.0,0.0]
		self.setpoint = [2,2,20]
		
		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		
		self.Kp = [30,30,40]
		self.Ki = [0,0,0.00018]	
		self.Kd = [150000,150000,150000]
		
		#by me
		self.sample_time = 60
		self.prev_values = [0,0,0]
		self.max_values = 2000
		self.min_values = 1000
		self.error = [0,0,0]
		self.now=0.0000
		self.timechange=0.000
		self.errsum=[0,0,0]
		self.derr=[0,0,0]
		self.last_time=0.0000
		
		self.out_roll=0.000
		self.out_pitch=0.000
		self.out_throttle=0.000
		
		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		self.altError = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitchError = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.rollError = rospy.Publisher('/roll_error', Float64, queue_size=1)
		
		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		
		self.arm() # ARMING THE DRONE
		
		# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)	
		
		
	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x	
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		
	# Callback function for /pid_tuning_altitude	
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.09 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.4
			
	def pitch_set_pid(self,pitch):
		self.Kp[0] = pitch.Kp * 0.09 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = pitch.Ki * 0.008
		self.Kd[0] = pitch.Kd * 0.4


	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp * 0.09 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = roll.Ki * 0.008
		self.Kd[1] = roll.Kd * 0.4	
		
	def pid(self):	
		#time functions
		self.now = int(round(time.time() * 1000))
		self.timechange=self.now-self.last_time
		
		#delta time must be more than step time of Gazebo, otherwise same values will be repeated 
		if (self.timechange>self.sample_time):
			if (self.last_time!=0):			

				#Getting error of all coordinates		
				self.error[0]=self.drone_position[0] - self.setpoint[0]
				self.error[1]=self.drone_position[1] - self.setpoint[1]
				self.error[2]=self.drone_position[2] - self.setpoint[2]
				

				#Integration for Ki
				#self.errsum[0]=self.errsum[0]+(self.error[0]*self.timechange)
				#self.errsum[1]=self.errsum[1]+(self.error[1]*self.timechange)
				self.errsum[2]=self.errsum[2]+(self.error[2]*self.timechange)


				#Derivation for Kd
				self.derr[0]=(self.error[0]-self.prev_values[0])/self.timechange
				self.derr[1]=(self.error[1]-self.prev_values[1])/self.timechange
				self.derr[2]=(self.error[2]-self.prev_values[2])/self.timechange


				#Calculating output in 1500
				self.cmd.rcRoll=int (1500-(self.Kp[0]*self.error[0])-(self.Kd[0]*self.derr[0]))
				self.cmd.rcPitch=int (1500+(self.Kp[1]*self.error[1])+(self.Kd[1]*self.derr[1]))
				self.cmd.rcThrottle=int (1500+(self.Kp[2]*self.error[2])+(self.Kd[2]*self.derr[2])-(self.errsum[2]*self.Ki[2]))

				
				#Checking min and max threshold and updating on true
				#Throttle Conditions
				if self.cmd.rcThrottle>2000:
					self.cmd.rcThrottle=self.max_values
				if self.cmd.rcThrottle<1000:
					self.cmd.rcThrottle=self.min_values		

				#Pitch Conditions
				if self.cmd.rcPitch>2000:
					self.cmd.rcPitch=self.max_values	
				if self.cmd.rcPitch<1000:
					self.cmd.rcPitch=self.min_values

				#Roll Conditions
				if self.cmd.rcRoll>2000:
					self.cmd.rcRoll=self.max_values
				if self.cmd.rcRoll<1000:
					self.cmd.rcRoll=self.min_values


				#Publishing values on topic 'drone command'
				self.command_pub.publish(self.cmd)

				
				#Updating prev values for all axis
				self.prev_values[0]=self.error[0]
				self.prev_values[1]=self.error[1]
				self.prev_values[2]=self.error[2]
		

		 		
		 	#Updating last time value	
			self.last_time=self.now
	

			#Getting values for Plotjuggler
			self.rollError.publish(self.error[0])
			self.pitchError.publish(self.error[1])
			self.altError.publish(self.error[2])
		
		
if __name__ == '__main__':
	e_drone = Edrone()
	r = rospy.Rate(33) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()

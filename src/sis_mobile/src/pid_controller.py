#!/usr/bin/env python
import serial
import rospy
import tf

from math import sin, cos, isnan
from Adafruit_MotorHAT import Adafruit_MotorHAT
from simple_pid import PID
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped

WIDTH = 0.179
RADIUS = 0.032

class Car_controller(object):
	def __init__(self):
		self.motorhat = Adafruit_MotorHAT(0x60)
		self.left_motor = self.motorhat.getMotor(1)
		self.right_motor = self.motorhat.getMotor(2)
		self.pid_r = PID(1000.0, 500.0, 100.0, sample_time = 0.05) # P/I/D for right wheel
		self.pid_l = PID(1000.0, 500.0, 100.0, sample_time = 0.05) # P/I/D for left wheel
		self.pid_r.output_limits = (-255, 255)
		self.pid_l.output_limits = (-255, 255)
		self.port = rospy.get_param("~port", '/dev/ttyACM0')
		self.ard = serial.Serial(self.port, 57600)
		self.visual_path = True
		# flush serial data
		for i in range(0, 20):
			_ = self.ard.readline()
		print "Data flushed"
		self.pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size = 10)
		if self.visual_path:
			self.pub_path = rospy.Publisher('wheel_odom_path', Path, queue_size = 10)
			self.path = Path()
			self.path.header.frame_id = 'odom'
		self.sub_cmd  = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb,  queue_size = 1)
		self.tf_br = tf.TransformBroadcaster()
		rospy.Timer(rospy.Duration(0.01), self.read_data) # 100Hz
		self.velocity_right = None
		self.velocity_left  = None
		self.heading = 0
		self.x = 0
		self.y = 0
		self.time = rospy.Time.now()
	def read_data(self, event):
		data_str = self.ard.readline()
		data_list = data_str.split()
		try:
			data_list = [float(i) for i in data_list]
		except ValueError:
			return
		if len(data_list) != 3:
			return 
		if data_list[0] >= 0.12 or data_list[1] >= 0.12:
			return # weird data
		if len(data_list) == 3:
			self.velocity_right, self.velocity_left, self.heading = data_list
			# dead reckoning
			dt = rospy.Time.now().to_sec() - self.time.to_sec()
			s_r = self.velocity_right * dt
			s_l = self.velocity_left  * dt
			self.x = self.x + (s_l+s_r)/2 * cos(self.heading)
			self.y = self.y + (s_l+s_r)/2 * sin(self.heading)
			self.tf_br.sendTransform((self.x, self.y, 0),
				      (0, 0, sin(self.heading/2), cos(self.heading/2)),
				      rospy.Time.now(),
				      'car_base',
				      'odom') 
			odom = Odometry()
			odom.header.frame_id = 'odom'
			odom.header.stamp = rospy.Time.now()
			odom.child_frame_id = 'car_base'
			odom.pose.pose.orientation.z = sin(self.heading/2)
			odom.pose.pose.orientation.w = cos(self.heading/2)
			odom.twist.twist.linear.x = (self.velocity_right+self.velocity_left)/2
			self.pub_odom.publish(odom)
			if self.visual_path:
				pose = PoseStamped()
				pose.header.frame_id = "odom"
				pose.header.stamp = rospy.Time.now()
				pose.pose.position.x = self.x
				pose.pose.position.y = self.y
				pose.pose.orientation.z = sin(self.heading/2)
				pose.pose.orientation.w = cos(self.heading/2)
				self.path.poses.append(pose)
				self.pub_path.publish(self.path)
			self.time = rospy.Time.now() # update time
	def cmd_cb(self, msg):
		if msg.linear.x == 0 and msg.angular.z == 0:
			print "reach"
			self.pid_r.auto_mode = False
			self.pid_l.auto_mode = False
			self.motor_motion(0, 0)
		if not isnan(self.velocity_right): 
			self.pid_r.auto_mode = True
			self.pid_l.auto_mode = True
			v_d = msg.linear.x 
			w_d = msg.angular.z # desired one
			v_d_r = v_d + WIDTH/2*w_d
			v_d_l = v_d - WIDTH/2*w_d
			self.pid_r.setpoint = v_d_r
			self.pid_l.setpoint = v_d_l
			pwm_r = self.pid_r(self.velocity_right)
		  	pwm_l = self.pid_l(self.velocity_left)
			self.motor_motion(pwm_r, pwm_l)
	def motor_motion(self, pwm_r, pwm_l):
		print int(pwm_r), int(pwm_l), self.velocity_right, self.velocity_left, self.heading
		if pwm_r < 0:
			right_state = Adafruit_MotorHAT.BACKWARD
			pwm_r = -pwm_r
		elif pwm_r > 0:
			right_state = Adafruit_MotorHAT.FORWARD
                else:
                        right_state = Adafruit_MotorHAT.RELEASE
		if pwm_l < 0:
			left_state  = Adafruit_MotorHAT.BACKWARD
			pwm_l = -pwm_l
		elif pwm_l > 0:
			left_state = Adafruit_MotorHAT.FORWARD
                else:
                        left_state = Adafruit_MotorHAT.RELEASE
		self.right_motor.setSpeed(int(pwm_r))
		self.left_motor.setSpeed(int(pwm_l))
		self.right_motor.run(right_state)
		self.left_motor.run(left_state)
		if pwm_r == 0 and pwm_l == 0:
			rospy.sleep(2.0)
	def shutdown(self):
		print "shutdown"
		self.sub_cmd.unregister()
		rospy.sleep(1.0)
		self.right_motor.setSpeed(0)
		self.left_motor.setSpeed(0)
		self.right_motor.run(Adafruit_MotorHAT.RELEASE)
		self.left_motor.run(Adafruit_MotorHAT.RELEASE)
		del self.motorhat	
		print "complete"

if __name__ == '__main__':
	rospy.init_node('pid_controller_node')
	controller = Car_controller()
	rospy.on_shutdown(controller.shutdown)
	rospy.spin()

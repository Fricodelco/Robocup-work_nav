#!/usr/bin/env python3  
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep, time
class ParkController:
	def __init__(self):
		self.ranges = []
		self.sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_cb)
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	def scan_cb(self, data):
		if data.header.frame_id == "laser_hokuyo":
			self.ranges = data.ranges
	def parallel_parking_left(self):
		error_z = self.ranges[228] - self.ranges[305]
		up_z = 3.5*error_z
		error_y = self.ranges[228] + self.ranges[305] - 0.45
		up_y = -error_y
		if abs(up_y) > 0.08:
			up_y = self.sign(up_y)*0.08
		if abs(up_z) > 0.1:
			up_y = self.sign(up_z)*0.1
		# print("up_x: "+str(up_x))
		# print("up_y: "+str(up_y))
		# print("up_z: "+str(up_z))
		# self.speed_publisher(up_x,up_y,up_z)
	def sign(self, data):
		if data > 0:
			return 1
		else:
			return -1
	def speed_publisher(self,x,y,z):
		msg = Twist()
		msg.linear.x = x
		msg.linear.y = y
		msg.angular.z = z 
		self.cmd_pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('park_controller')
	controller = ParkController()
	sleep(1)
	while not rospy.is_shutdown():
		try:
			sleep(0.05)
		except KeyboardInterrupt:
			print("something goes wrong")
		# finally:
			# controller.speed_publisher(0,0,0)
	

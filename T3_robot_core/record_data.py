#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry #nav_msgs/Odometry
import os

file_path = os.path.dirname(os.path.abspath(__file__))

record_file = open(file_path + '/data.txt', 'w')

def record_imu_callback(data):
	x = data.twist.twist.linear.x
	#my = data.orientation.y
	#mz = data.orientation.z

	to_write = "mx: %3f\n"%(x)
	record_file.write(to_write)


if __name__=='__main__':
	rospy.init_node("record_data")
	rospy.Subscriber("odom", Odometry, record_imu_callback)

	try:
		while not rospy.is_shutdown():
			pass
	except rospy.ROSInterruptException:
		record_file.close()

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def controller():
	velpub = rospy.Publisher(/cmd_vel', Twist, queue_size=1)
	sub = rospy.Subscriber("mmm/sensor_data", SensorData, self.display_sensor_data)



if __name__ == '__main__':

    try:
        rospy.init_node('teleop', anonymous=False)
        while not rospy.is_shutdown():
        	
    except rospy.ROSInterruptException:
        exit(0)

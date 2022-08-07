#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
import std_msgs.msg

class camera_1:

  def __init__(self):
    self.cloud_pub=rospy.Publisher("point_cloud", PointCloud, queue_size=10 )
    self.image_sub = rospy.Subscriber("virat/camera/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    gray_image= cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    _, thresh_image=cv2.threshold(gray_image,220,225,cv2.THRESH_BINARY)
    contours, _= cv2.findContours (thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
      #getting coords from contours
      pts= cnt.squeeze()
      for coord in pts:
        new=np.append(coord,[1])
        #----------PlS TRANSFORM HERE--------------------
        
        
        
        
        
        
        
        
        
        #__________________________________________________
        #creating point cloud and publishing
        pt_32=Point32(new[0],new[1],new[2])
        points=[]
        points.append(pt_32)
        rospy.loginfo(pt_32)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        cloud=PointCloud()
        cloud.header=header
        cloud.points=points
        self.cloud_pub.publish(cloud)

      cv2.drawContours(cv_image, [cnt],0,(255, 0, 0),3)
    cv2.imshow("Camera output", cv_image)
    cv2.imshow("Processed", thresh_image)
    cv2.waitKey(3)

def main():
	camera_1()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()

#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class camera_1:

  def __init__(self):
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

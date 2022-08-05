#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch


def inverse_projection_transform(twod_coords):
    pass


<<<<<<< HEAD
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
=======
def img_callback(data):
    bridge = CvBridge()  # Setting up a Cvbridge object to convert ROSImg data into cv2 readable data
    og_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")  # Converting
    cv2.imshow("Camera output", og_image)  # Showing the camera feed in a new window
>>>>>>> Detected contours
    cv2.waitKey(3)

    # Greyscaling and then thresholding the image to have only potholes in the image
    gray_img = cv2.cvtColor(og_image, cv2.COLOR_BGR2GRAY)
    ret, gray_img = cv2.threshold(gray_img, 200, 250, cv2.THRESH_TOZERO)
    # Detecting the contours
    contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # print(inverse_projection_transform(contours))
    # ct_image = cv2.drawContours(og_image, contours, -1, (0, 0, 0), 3)
    # cv2.imshow("Detections", og_image)


if __name__ == '__main__':

    rospy.init_node('potholedetector', anonymous=False)
    image_sub = rospy.Subscriber("virat/camera/image_raw", Image, img_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

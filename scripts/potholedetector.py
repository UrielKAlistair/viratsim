#!/usr/bin/env python3

import rospy

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Point32
import std_msgs.msg


def transform_and_publish(contours):
    global cloud_pub
    global P

    if P is None:
        return

    pointsin3d = []

    for contour in contours:

        # getting coords from contours
        cntrpts = contour.squeeze()

        for point in cntrpts:
            new = np.append(point, [1])
            new = np.linalg.solve(P[:, [0, 1, 2]], new - P[:, 3])
            pt_32 = Point32(new[0], new[1], new[2])
            pointsin3d.append(pt_32)

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'
    cloud = PointCloud()
    cloud.header = header
    cloud.points = pointsin3d
    cloud_pub.publish(cloud)


def camera_details_callback(data):
    global P

    Pt = data.P
    P = []

    for i in range(3):
        P.append([])
        for j in range(4):
            P[i].append(Pt[i * 4 + j])

    P = np.array(P)


def img_callback(data):
    bridge = CvBridge()  # Setting up a Cvbridge object to convert ROSImg data into cv2 readable data
    og_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")  # Converting

    # Greyscaling and then thresholding the image to have only potholes in the image
    gray_img = cv2.cvtColor(og_image, cv2.COLOR_BGR2GRAY)
    _, gray_img = cv2.threshold(gray_img, 200, 250, cv2.THRESH_TOZERO)

    # Detecting the contours
    contours, _ = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    ct_image = cv2.drawContours(og_image, contours, -1, (0, 0, 0), 3)

    # Publishing to point cloud
    transform_and_publish(contours)

if __name__ == '__main__':

    rospy.init_node('potholedetector', anonymous=False)
    image_sub = rospy.Subscriber("virat/camera/image_raw", Image, img_callback)
    cloud_pub = rospy.Publisher("point_cloud", PointCloud, queue_size=10)

    P = None
    camera_info_sub = rospy.Subscriber("virat/camera/camera_info", CameraInfo, camera_details_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

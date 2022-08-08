#!/usr/bin/env python3

import rospy

import torch
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
import std_msgs.msg


def transform_and_publish(twod_coords):
    global cloud_pub
    global K
    global P

    if K is None or P is None:
        return

    for point in twod_coords:

        # getting coords from contours
        pts = point.squeeze()
        for coord in pts:
            new = np.append(coord, [0])
            # ----------PlS TRANSFORM HERE--------------------
            new = np.linalg.solve(np.matmul(K, P[:, [0, 1, 3]]), new)
            new[2] = 0
            # __________________________________________________
            # creating point cloud and publishing

            pt_32 = Point32(new[0], new[1], new[2])
            points = [pt_32]
            rospy.loginfo(pt_32)
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base_link'
            cloud = PointCloud()
            cloud.header = header
            cloud.points = points
            cloud_pub.publish(cloud)


def camera_details_callback(data):
    global K
    global P

    Kt = data.K
    Pt = data.P
    K = []
    P = []

    for i in range(3):
        K.append([])
        for j in range(3):
            K[i].append(Kt[i * 3 + j])

    for i in range(3):
        P.append([])
        for j in range(4):
            P[i].append(Pt[i * 3 + j])

    K = np.array(K)
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

    # Displaying to user
    cv2.imshow("Camera output", og_image)  # Showing the camera feed in a new window
    cv2.imshow("Detections", ct_image)
    cv2.waitKey(3)


if __name__ == '__main__':

    rospy.init_node('potholedetector', anonymous=False)
    image_sub = rospy.Subscriber("virat/camera/image_raw", Image, img_callback)
    cloud_pub = rospy.Publisher("point_cloud", PointCloud, queue_size=10)

    K = None
    P = None
    camera_info_sub = rospy.Subscriber("virat/camera/camera_info", CameraInfo, camera_details_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

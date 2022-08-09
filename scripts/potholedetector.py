#!/usr/bin/env python3

import rospy

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo

from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg


def point_to3d(u, v):
    global K

    kinv = np.linalg.inv(K)
    R = np.array([[1, 0, 0], [0, np.cos(0.45), -np.sin(0.45)], [0, np.sin(0.45), np.cos(0.45)]])
    # R = R.transpose()
    nc = R.dot(np.array([0, 1, 0]))

    kinv_dot_uv = kinv.dot(np.array([u, v, 1]))
    ans = 1.18 * kinv_dot_uv / nc.dot(kinv_dot_uv)
    return ans


def transform_and_publish(contours):
    global cloud_pub

    if K is None:
        return

    pointsin3d = []

    for contour in contours:
        # getting coords from contours
        contour = contour.squeeze()
        for point in contour:
            ans = point_to3d(point[0], point[1])
            pointsin3d.append((ans[2], -ans[0], -ans[1], 1))

    header = std_msgs.msg.Header()
    header.frame_id = 'camera'
    header.stamp = rospy.Time.now()

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 16, PointField.UINT32, 1),
              ]

    cloud = point_cloud2.create_cloud(header, fields, pointsin3d)
    cloud_pub.publish(cloud)


def camera_details_callback(data):
    global K

    temp = data.K
    K = []

    for i in range(3):
        K.append([])
        for j in range(3):
            K[i].append(temp[i * 3 + j])

    K = np.array(K)


def img_callback(data):
    bridge = CvBridge()  # Setting up a Cvbridge object to convert ROSImg data into cv2 readable data
    og_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")  # Converting

    # Greyscaling and then thresholding the image to have only potholes in the image
    gray_img = cv2.cvtColor(og_image, cv2.COLOR_BGR2GRAY)
    _, gray_img = cv2.threshold(gray_img, 200, 250, cv2.THRESH_TOZERO)

    # Detecting the contours
    contours, _ = cv2.findContours(gray_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Publishing to point cloud
    transform_and_publish(contours)


if __name__ == '__main__':

    rospy.init_node('potholedetector', anonymous=False)
    image_sub = rospy.Subscriber("virat/camera/image_raw", Image, img_callback)
    cloud_pub = rospy.Publisher("point_cloud", PointCloud2, queue_size=10)

    K = None
    camera_info_sub = rospy.Subscriber("virat/camera/camera_info", CameraInfo, camera_details_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

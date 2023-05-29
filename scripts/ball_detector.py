#!/usr/bin/env python3
# import the necessary packages
import numpy as np
import cv2
import imutils
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Point


vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

greenLower = (20, 100, 100)
greenUpper = (40, 255, 255)

bridge = CvBridge()
twist = Twist()

prev_x = 0


def callback(image):
    global prev_x
    try:
        frame = bridge.imgmsg_to_cv2(image, 'bgr8')
    except CvBridgeError as e:
        print("Cv bridge error")

    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except ZeroDivisionError:
            center = (0, 0)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # defining the height and width of the frame
        height = 336
        width = 600
    else:
        center = (0, 0)
        radius = 500  # not found

    x_center_image = center[0]-300
    radius_limit = 120
    if center[0] == 0 and center[1] == 0:
        # This means no ball is detected
        if prev_x < 0:
            # rotate anticlockwise
            twist.linear.x = 0
            twist.angular.z = 1
            prev_x = x_center_image
            print("Cant find the ball rotating anticlockwise")
        else:
            print("Cant find the ball rotating clockwise")
            prev_x = x_center_image
            twist.linear.x = 0
            twist.angular.z = -1
    else:
        # The ball is found in this case
        if x_center_image < -20 or x_center_image > 20:  # This means ball is to the left of the bot
            if radius > 45:
                # just make the bot align with the ball
                print("Aligning with the ball")
                prev_x = x_center_image
                twist.angular.z = -(0.01*x_center_image)*0.01
                twist.linear.x = 0
            else:
                print("moving forward with linear as well a angular speed")
                twist.angular.z = -(0.01*x_center_image)
                twist.linear.x = 0.1
                prev_x = x_center_image
        else:
            if radius < 45:
                print("moving forward in a straight line")
                twist.linear.x = 0.1
                twist.angular.z = 0
                prev_x = x_center_image
            else:
                print("stopping")
                twist.linear.x = 0
                twist.linear.z = 0

                # Display the frame
    cv2.imshow("Video Stream", frame)
    cv2.waitKey(1)
    vel_pub.publish(twist)


def twist_publisher():
    global vel_pub
    global twist
    vel_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('ball_detector')
    rospy.Subscriber('/camera_image', Image, callback)
    rospy.Timer(rospy.Duration(1.0/10.0), twist_publisher)
    rospy.spin()

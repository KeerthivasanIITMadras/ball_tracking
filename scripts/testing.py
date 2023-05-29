#!/usr/bin/env python3
import cv2
import socket
import struct
import io
import numpy as np
from collections import deque
import numpy as np
import imutils

# Set the Raspberry Pi's IP address and port
raspberry_pi_ip = "10.9.72.244"
raspberry_pi_port = 8000

# Create a socket and connect to the Raspberry Pi
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((raspberry_pi_ip, raspberry_pi_port))
client_socket = client_socket.makefile("rb")

greenLower = (20, 100, 100)
greenUpper = (40, 255, 255)
pts = deque(maxlen=64)

try:
    while True:
        # Retrieve the image length
        image_len = struct.unpack(
            "<L", client_socket.read(struct.calcsize("<L")))[0]

        if not image_len:
            break

        # Retrieve the image data
        image_data = io.BytesIO()
        image_data.write(client_socket.read(image_len))
        image_data.seek(0)

        # Convert the image data to a numpy array
        image = np.asarray(bytearray(image_data.read()), dtype=np.uint8)

        # Decode the numpy array as an image
        frame = cv2.imdecode(image, cv2.IMREAD_COLOR)

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
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # update the points queue
        pts.appendleft(center)
        # Process the frame here (e.g., detect objects, apply filters, etc.)

        # Display the frame
        cv2.imshow("Video Stream", frame)
finally:
    client_socket.close()
    cv2.destroyAllWindows

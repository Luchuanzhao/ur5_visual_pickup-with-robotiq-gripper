#!/usr/bin/env python


from cv2 import threshold, waitKey
from numpy import imag
import rospy, sys, numpy as np
from copy import deepcopy
from ur5_pickup.msg import Tracker
import cv2, cv_bridge
from sensor_msgs.msg import Image
tracker = Tracker()
class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)
        self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)


    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        image=image[200:600,100:800]
        mask=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret,mask=cv2.threshold(mask,250,255,cv2.THRESH_BINARY_INV)
        # print(image.shape)
        # END BRIDGE
        # BEGIN HSV
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # END HSV
        
    ########### Find the blocks by filtering color(red) #################################
        
        # BEGIN FILTER
        # lower_red = np.array([ 0,  100, 100])
        # upper_red = np.array([10, 255, 255])
        # mask = cv2.inRange(hsv, lower_red, upper_red)
        # cv2.imshow("win",mask)
        # cv2.waitKey(0)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel)
        (cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #area = cv2.contourArea(cnts)
        h, w, d = image.shape
        area=0
        idx=0
        for i, c in enumerate(cnts):
            if area<cv2.contourArea(c):
                area=cv2.contourArea(c)
                idx=i
        M = cv2.moments(cnts[idx])
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

        # cx range (55,750) cy range( 55, ~ )
        # END FINDER
        # Isolate largest contour
        #  contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
        #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            for i, c in enumerate(cnts):
                area = cv2.contourArea(c)
                if area > 500:
                    self.track_flag = True  # If area>7500, set track_flag True, which means we have track the block
                    self.cx = cx
                    self.cy = cy
                    self.error_x = 600 - self.cx
                    self.error_y = self.cy - (h/2+195)
                    tracker.x = cx
                    tracker.y = cy
                    tracker.error_x=self.error_x*0.04/177
                    #(_,_,w_b,h_b)=cv2.boundingRect(c)
                    #print w_b,h_b
                    # BEGIN circle
                    cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
                    cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.drawContours(image, cnts, -1, (255, 0, 0),2)
                    #BGIN CONTROL
                    break

        self.cxy_pub.publish(tracker)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)

follower=ur5_vision()
rospy.spin()

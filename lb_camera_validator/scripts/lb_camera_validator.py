#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2 

class Cam_capture(object):
    def __init__(self, rgb_image_topic):

        self.row                 = 13
        self.col                 = 9
        self.checkerboard        = (self.row, self.col)
        
        self.set_pixel_distance  = 32
        self.allowable_pixel_err = 5
        self.criteria            = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  
        self.bridge              = CvBridge()

        self._rgb_image_topic    = rgb_image_topic

        self._check_camera_working()

        self.sub_rectified_img   =rospy.Subscriber(self._rgb_image_topic, Image, self.cameraCallback,queue_size=1)

    def _check_camera_working(self):

        self.frame = None
        rospy.loginfo("Waiting for "+self._rgb_image_topic+" to be READY...")

        while self.frame is None and not rospy.is_shutdown():
            try:
                image_msg = rospy.wait_for_message(self._rgb_image_topic, Image, timeout=5.0)
                self.frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
                rospy.loginfo("Current "+self._rgb_image_topic+" READY=>")

            except Exception as e: 
                print(e)

                rospy.logerr("Current "+self._rgb_image_topic+" not ready yet, retrying...")
    
      
    def cameraCallback(self, data):

        self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        

    def validation(self):

        while True:

            gray_frame  = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) 
            cv2.imshow('Camera feed' , gray_frame)

            pressedKey = cv2.waitKey(1) & 0xFF
        
            # Check whether 'a' key is pressed, and if it is pressed, check pixel dst values for each square in checkerboard
            if pressedKey == ord('a'):
                    
                print("Start pixel validation")
                    
                # Local variables for resetting after every camera pixel check
                valid_pixel_x_count = 0
                valid_pixel_y_count = 0
                x_pixel_list = []
                y_pixel_list = []

                ret, corners = cv2.findChessboardCorners(
                                                gray_frame, self.checkerboard,
                                                cv2.CALIB_CB_ADAPTIVE_THRESH
                                                + cv2.CALIB_CB_FAST_CHECK +
                                                cv2.CALIB_CB_NORMALIZE_IMAGE)

                if(ret == True):
                    corners2 = cv2.cornerSubPix(gray_frame, corners, (11,11), (-1,-1), self.criteria)

                    # Check pixel dst values for every sqaure in checkerboard
                    for n in range(self.row * self.col):

                        # for out of range error check
                        if(n < ((self.row * self.col)- self.row)):
                            x_pixel = corners2[n][0][0] - corners2[n+self.row][0][0]
                            x_pixel_list.append(abs(x_pixel))
                                
                            # Accept validation if the x pixel dst error is within allowable value
                            if(self.set_pixel_distance-self.allowable_pixel_err <= abs(x_pixel) <= self.set_pixel_distance+self.allowable_pixel_err):
                                valid_pixel_x_count += 1
                            
                        # for out of range error check      
                        if(n < (self.row * self.col)-1 and (n+1)% self.row != 0):
                            y_pixel = corners2[n+1][0][1] - corners2[n][0][1]
                            y_pixel_list.append(abs(y_pixel))
                                
                            # Accept validation if the y pixel dst error is within allowable value
                            if(self.set_pixel_distance-self.allowable_pixel_err <= abs(y_pixel) <= self.set_pixel_distance+self.allowable_pixel_err):
                                valid_pixel_y_count += 1
        
                    print(max(x_pixel_list))
                    print(max(y_pixel_list))                
                        
                    # Accept overall validation if x and y pixel valid count has specific value
                    if(valid_pixel_x_count == (self.row*(self.col-1)) and valid_pixel_y_count == (self.col*(self.row-1))):
                        print("Success validation")

            # Check whether space button is pressed and if it is pressed, break the while loop
            if pressedKey == ord(' '):
                print("Stop streaming")
                break

        # Close the window
        cv2.destroyAllWindows()

        return 0

    def run(self):
        self.validation()

if __name__ == '__main__': 

    try:
        rospy.init_node("camera_validator", anonymous=True)
        rgb_topic  = rospy.get_param("~rgb_camera_topic", "/usb_cam/image_rect_color")
        cam_obj    = Cam_capture(rgb_topic)
        cam_obj.run()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
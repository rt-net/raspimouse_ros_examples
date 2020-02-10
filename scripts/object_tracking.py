#!/usr/bin/env python
#coding: utf8

import rospy, cv2, math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# Lower limit of the ratio of the detected area to the screen.
# Object tracking is not performed below this ratio.
LOWER_LIMIT = 0.01


class ObjectTracker():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_org = None # Acquired image
        self.area_max = 0 # Maximum area detected in the current image[pixel]
        self.area_default = 0 # Maximum area detected from the first image[pixel]
        self.disp_default_now = 0 # Difference between area_max and area_default[%]
        self.area_whole = None # Total number of pixels[pixel]
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.pub = rospy.Publisher("object", Image, queue_size=1)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.wait_for_service("/motor_on")
        rospy.wait_for_service("/motor_off")
        rospy.on_shutdown(rospy.ServiceProxy("/motor_off", Trigger).call)
        rospy.ServiceProxy("/motor_on", Trigger).call()

    def get_image(self, img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")
            # Calculate total number of pixels
            self.area_whole = self.image_org.shape[0] * self.image_org.shape[1]

        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_ball(self):
        if self.image_org is None:
            return None
        org = self.image_org
        # Extract orange(use HSV color model) 
        hsv = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)
        min_hsv_orange = np.array([15, 150, 40])
        max_hsv_orange = np.array([20, 255, 255])
        binary = cv2.inRange(hsv, min_hsv_orange, max_hsv_orange)
        # Morphology
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations = 2)
        # Calculate center of gravity
        cog_img, point_cog = self.detect_cog(binary)
        self.monitor(cog_img)
        return point_cog

    def detect_cog(self, binary):
        self.area_max = 0
        area_max_num = 0
        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        point_cog = (self.image_org.shape[1], self.image_org.shape[0])
        # Find index of maximum area
        for i, cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if(self.area_max < area):
                    self.area_max = area
                    area_max_num = i
        # Determine initial area
        if(self.area_default == 0 and self.area_max != 0):
            self.area_default = self.area_max
        self.disp_default_now = (self.area_default - self.area_max) / self.area_whole
        # Draw countours
        cog_img = cv2.drawContours(self.image_org, contours, area_max_num, (0, 255, 0), 5)

        if(self.area_max/self.area_whole > LOWER_LIMIT):
            # Calsulate center of gravity
            M = cv2.moments(contours[area_max_num])
            cog_x = int(M['m10'] / M['m00'])
            cog_y = int(M['m01'] / M['m00'])
            point_cog = (cog_x, cog_y)
            cog_img = cv2.circle(cog_img, point_cog, 15, (255, 0, 0), thickness=-1) 
        return cog_img, point_cog

    def monitor(self, org):
        self.pub.publish(self.bridge.cv2_to_imgmsg(org, "bgr8"))
        return "detected"

    # Determine rotation angle from center of gravity position
    def rot_vel(self):
        point_cog = self.detect_ball()
        if (self.area_max/self.area_whole < LOWER_LIMIT):
            return 0.0
        wid = self.image_org.shape[1]/2
        pos_x_rate = (point_cog[0] - wid)*1.0/wid
        rot = -0.25*pos_x_rate*math.pi
        rospy.loginfo("detect %f", rot)
        return rot

    def control(self):

        m = Twist()
        # m.linear.x: speed parameter
        # m.angular.z: angle parameter
        if(self.area_max/self.area_whole > LOWER_LIMIT):
            # Move backward and forward by difference from default area
            if(self.disp_default_now > 0.01):
                m.linear.x = 0.1
                print("forward")
            elif(self.disp_default_now < -0.01):
                m.linear.x = -0.1
                print("backward")
            else:
                m.linear.x = 0
                print("stay")
        # Center of gravity is centered on the image
        m.angular.z = self.rot_vel()
        self.cmd_vel.publish(m)


if __name__ == '__main__':
    rospy.init_node('object_tracking')
    rospy.sleep(3.)
    ot = ObjectTracker()

    rate = rospy.Rate(10)
    rate.sleep()
    while not rospy.is_shutdown():
        ot.control()
        rate.sleep()


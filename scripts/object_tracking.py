#!/usr/bin/env python
# coding: utf-8

import rospy, cv2, math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class ObjectTracker():
    # Lower limit of the ratio of the detected area to the screen.
    # Object tracking is not performed below this ratio.
    LOWER_LIMIT = 0.01

    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_image = None
        self._object_pixels = 0 # Maximum area detected in the current image[pixel]
        self._object_pixels_default = 0 # Maximum area detected from the first image[pixel]

        self._pub_binary_image = rospy.Publisher("binary", Image, queue_size=1)
        self._pub_pbject_image = rospy.Publisher("object", Image, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self._sub_image = rospy.Subscriber("/cv_camera/image_raw", Image, self._image_callback)

        rospy.wait_for_service("/motor_on")
        rospy.wait_for_service("/motor_off")
        rospy.on_shutdown(rospy.ServiceProxy("/motor_off", Trigger).call)
        rospy.ServiceProxy("/motor_on", Trigger).call()

    def _image_callback(self, img):
        try:
            self._captured_image = self._cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def _pixels(self, cv_image):
        return cv_image.shape[0] * cv_image.shape[1]

    def _detected_target(self):
        if not self._captured_image is None:
            object_per_image = self._object_pixels / self._pixels(self._captured_image)
            return object_per_image > ObjectTracker.LOWER_LIMIT
        else:
            return False

    def _object_pixels_ratio(self):
        if not self._captured_image is None:
            diff_pixels = self._object_pixels - self._object_pixels_default
            return diff_pixels / self._pixels(self._captured_image)
        else:
            return 0

    def _object_is_bigger_than_default(self):
        return self._object_pixels_ratio() > 0.01

    def _object_is_smaller_than_default(self):
        return self._object_pixels_ratio() < -0.01

    def _set_color_orange(self):
        # [H(0~180), S(0~255), V(0~255)]
        min_hsv_orange = np.array([15, 200, 80])
        max_hsv_orange = np.array([20, 255, 255])
        return min_hsv_orange, max_hsv_orange
    
    def _set_color_green(self):
        min_hsv_green = np.array([60, 60, 40])
        max_hsv_green = np.array([70, 255, 255])
        return min_hsv_green, max_hsv_green

    def _set_color_blue(self):
        min_hsv_blue= np.array([105, 90, 40])
        max_hsv_blue= np.array([120, 255, 255])
        return min_hsv_blue, max_hsv_blue

    # Extract object(use HSV color model)
    def _detect_ball(self):
        if self._captured_image is None:
            return None
        org = self._captured_image
        hsv = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)

        min_hsv, max_hsv = self._set_color_orange()
        # min_hsv, max_hsv = self._set_color_green()
        # min_hsv, max_hsv = self._set_color_blue()

        binary = cv2.inRange(hsv, min_hsv, max_hsv)
        # Morphology
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations = 2)
        return binary

    def _calibrate_object_pixels_default(self):
        if self._object_pixels_default == 0 and self._object_pixels != 0:
            self._object_pixels_default = self._object_pixels

    def _detect_centroid(self, binary):
        self._object_pixels = 0
        area_max_num = 0
        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        point_centroid = (self._captured_image.shape[1], self._captured_image.shape[0])
        # Find index of maximum area
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if self._object_pixels < area:
                self._object_pixels = area
                area_max_num = i
        # Define object_pixels_default
        self._calibrate_object_pixels_default()
        # Draw countours
        centroid_img = cv2.drawContours(self._captured_image, contours, area_max_num, (0, 255, 0), 5)
        if self._detected_target():
            M = cv2.moments(contours[area_max_num])
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            point_centroid = (centroid_x, centroid_y)
            centroid_img = cv2.circle(centroid_img, point_centroid, 15, (255, 0, 0), thickness=-1) 
        return centroid_img, point_centroid

    def _monitor(self, img, pub):
        if img.ndim == 2:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "mono8"))
        elif img.ndim == 3:
            pub.publish(self._cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            pass

    # Determine rotation angle from center of gravity position
    def _rot_vel(self):
        if not self._detected_target():
            return 0.0
        wid = self._captured_image.shape[1]/2
        pos_x_rate = (self.point_centroid[0] - wid)*1.0/wid
        rot = -0.25*pos_x_rate*math.pi
        rospy.loginfo("detect %f", rot)
        return rot

    def image_processing(self):
        object_binary_img = self._detect_ball()
        self._monitor(object_binary_img, self._pub_binary_image)
        centroid_img, self.point_centroid = self._detect_centroid(object_binary_img)
        self._monitor(centroid_img, self._pub_pbject_image)

    def control(self):
        cmd_vel = Twist()
        if self._detected_target():
            # Move backward and forward by difference from default area
            if self._object_is_smaller_than_default():
                cmd_vel.linear.x = 0.1
                print("forward")
            elif self._object_is_bigger_than_default():
                cmd_vel.linear.x = -0.1
                print("backward")
            else:
                cmd_vel.linear.x = 0
                print("stay")
            cmd_vel.angular.z = self._rot_vel()
        self._pub_cmdvel.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('object_tracking')
    rospy.sleep(3.)
    ot = ObjectTracker()

    rate = rospy.Rate(60)
    rate.sleep()
    while not rospy.is_shutdown():
        ot.image_processing()
        ot.control()
        rate.sleep()


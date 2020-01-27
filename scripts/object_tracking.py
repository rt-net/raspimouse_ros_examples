#!/usr/bin/env python
#encoding: utf8

# 色相からボールを抽出して重心と面積を求める
# 重心は画像の中心に，面積は初期面積と同じになるように移動する

import rospy, cv2, math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

# 検出された面積が画像中に占める割合の下限
# この割合以下では物体追跡は行わない
LOWER_PERCENT = 0.01


class ObjectTracker():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_org = None # 取得した画像
        self.area_max = 0 # 現在の画像中で検出された最大面積[pixel^2]
        self.area_default = 0 # 最初(起動時)に取得した面積[pixel^2]
        self.disp_default_now = 0 # 初期面積と現在の面積の差 [%]
        self.area_whole = None # 画像全体の面積[pixel^2]
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.pub = rospy.Publisher("object", Image, queue_size=1)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.wait_for_service("/motor_on")
        rospy.wait_for_service("/motor_off")
        rospy.on_shutdown(rospy.ServiceProxy("/motor_off", Trigger).call)
        rospy.ServiceProxy("/motor_on", Trigger).call()

    # cv_camera_nodeから取得したメッセージをnparray型に
    def get_image(self, img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")
            # 画面全体の面積計算
            self.area_whole = self.image_org.shape[0] * self.image_org.shape[1]

        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_ball(self):
        if self.image_org is None:
            return None
        org = self.image_org
        # HSV色空間を用いてオレンジ色を抽出
        hsv = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)
        min_hsv_orange = np.array([15, 150, 40])
        max_hsv_orange = np.array([20, 255, 255])
        binary = cv2.inRange(hsv, min_hsv_orange, max_hsv_orange)
        # モルフォロジー
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations = 2)
        # 重心位置を求める
        center_img, point_cog = self.detect_center(binary)
        self.monitor(center_img)
        return point_cog

    # 2値画像の輪郭からモーメントを求める
    def detect_center(self, binary):
        self.area_max = 0
        area_max_num = 0
        _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        point_cog = (self.image_org.shape[1], self.image_org.shape[0])
        # 最大面積のインデックスを求める
        for i, cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if(self.area_max < area):
                    self.area_max = area
                    area_max_num = i
        # 初期面積を設定
        if(self.area_default == 0 and self.area_max != 0):
            self.area_default = self.area_max
        self.disp_default_now = (self.area_default - self.area_max) / self.area_whole
        # 輪郭を描画
        center_img = cv2.drawContours(self.image_org, contours, area_max_num, (0, 255, 0), 5)

        if(self.area_max/self.area_whole > LOWER_PERCENT):
            # 重心位置を計算
            M = cv2.moments(contours[area_max_num])
            cog_x = int(M['m10'] / M['m00'])
            cog_y = int(M['m01'] / M['m00'])
            point_cog = (cog_x, cog_y)
            center_img = cv2.circle(center_img, point_cog, 15, (255, 0, 0), thickness=-1) 
        return center_img, point_cog

    def monitor(self, org):
        self.pub.publish(self.bridge.cv2_to_imgmsg(org, "bgr8"))
        return "detected"

    # 物体の重心位置から回転角を決定
    def rot_vel(self):
        point_cog = self.detect_ball()
        if (self.area_max/self.area_whole < LOWER_PERCENT):
            return 0.0
        wid = self.image_org.shape[1]/2
        pos_x_rate = (point_cog[0] - wid)*1.0/wid
        rot = -0.25*pos_x_rate*math.pi
        rospy.loginfo("detect %f", rot)
        return rot

    def control(self):

        m = Twist()
        # m.linear.x: 直進のパラメータ
        # m.angular.z: 回転のパラメータ
        if(self.area_max/self.area_whole > LOWER_PERCENT):
            # 初期面積との差が一定以上or以下なら前後に移動する
            if(self.disp_default_now > 0.01):
                m.linear.x = 0.1
                print("adv")
            elif(self.disp_default_now < -0.01):
                m.linear.x = -0.1
                print("back")
            else:
                m.linear.x = 0
                print("stay")
        # 画像中心は常に重心位置と合わせるように移動する
        m.angular.z = self.rot_vel()
        self.cmd_vel.publish(m)


if __name__ == '__main__':
    rospy.init_node('object_tracking')
    ot = ObjectTracker()

    rate = rospy.Rate(10)
    rate.sleep()
    while not rospy.is_shutdown():
        ot.control()
        rate.sleep()


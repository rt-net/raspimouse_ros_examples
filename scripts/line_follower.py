#!/usr/bin/env python
# coding: UTF-8

# Copyright 2020 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import math
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from raspimouse_ros_2.msg import LightSensorValues
from raspimouse_ros_2.msg import ButtonValues
from raspimouse_ros_2.msg import LedValues


class LineFollower(object):
    def __init__(self):
        self._SENSORS = {"left": 0, "mid_left": 0, "mid_right": 0, "right": 0}
        self._sensor_line_values = dict(self._SENSORS)
        self._sensor_field_values = dict(self._SENSORS)
        self._line_thresholds = dict(self._SENSORS)
        self._line_is_detected_by_sensor = dict(self._SENSORS)
        self._present_sensor_values = dict(self._SENSORS)

        self._line_values_are_sampled = False
        self._field_values_are_sampled = False
        self._can_publish_cmdvel = False

        self._mouse_buttons = ButtonValues()

        self._pub_cmdvel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._pub_buzzer = rospy.Publisher('buzzer', UInt16, queue_size=1)
        self._pub_leds = rospy.Publisher('leds', LedValues, queue_size=1)

        self._sub_lightsensor = rospy.Subscriber(
            'lightsensors', LightSensorValues, self._callback_lightsensor, queue_size=1)
        self._sub_buttons = rospy.Subscriber(
            'buttons', ButtonValues, self._callback_buttons, queue_size=1)

        try:
            rospy.wait_for_service("motor_on", timeout=5)
            rospy.wait_for_service("motor_off", timeout=5)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service not found")
            rospy.signal_shutdown(e.message)
        else:
            rospy.on_shutdown(self._on_shutdown)

    def _on_shutdown(self):
        self._pub_leds.publish(LedValues())
        self._motor_off()

    def _motor_on(self):
        rospy.ServiceProxy("motor_on", Trigger).call()
        rospy.loginfo("motor_on")

    def _motor_off(self):
        rospy.ServiceProxy("motor_off", Trigger).call()
        rospy.loginfo("motor_off")

    def _callback_buttons(self, msg):
        self._mouse_buttons = msg

    def _callback_lightsensor(self, msg):
        # The order of the front distance sensors and the line following sensors are not same
        self._present_sensor_values["left"] = msg.right_forward
        self._present_sensor_values["mid_left"] = msg.right_side
        self._present_sensor_values["mid_right"] = msg.left_side
        self._present_sensor_values["right"] = msg.left_forward

        if self._sampling_is_done():
            self._update_line_detection()

    def _update_line_detection(self):
        for key in self._SENSORS:
            is_positive = self._present_sensor_values[key] > self._line_thresholds[key]

            if self._line_is_bright() == is_positive:
                self._line_is_detected_by_sensor[key] = True
            else:
                self._line_is_detected_by_sensor[key] = False

    def _beep_buzzer(self, freq, beep_time=0):
        self._pub_buzzer.publish(freq)
        rospy.sleep(beep_time)
        self._pub_buzzer.publish(0)

    def _beep_start(self):
        self._beep_buzzer(1000, 0.5)

    def _beep_success(self):
        self._beep_buzzer(1000, 0.1)
        rospy.sleep(0.1)
        self._beep_buzzer(1000, 0.1)

    def _beep_failure(self):
        for i in range(4):
            self._beep_buzzer(500, 0.1)
            rospy.sleep(0.1)

    def _sampling_is_done(self):
        if self._line_values_are_sampled and self._field_values_are_sampled:
            return True
        else:
            return False

    def _median(self, sensor1, sensor2):
        diff = math.fabs(sensor1 - sensor2)
        if sensor1 < sensor2:
            return sensor1 + diff * 0.5
        else:
            return sensor2 + diff * 0.5

    def _line_is_bright(self):
        SAMPLE = "right"
        if self._sensor_line_values[SAMPLE] > self._sensor_field_values[SAMPLE]:
            return True
        else:
            return False

    def _set_line_thresholds(self):
        if not self._sampling_is_done():
            return

        for key in self._SENSORS:
            self._line_thresholds[key] = self._median(
                    self._sensor_line_values[key],
                    self._sensor_field_values[key])

        rospy.loginfo("thresholds:" + str(self._line_thresholds))

    def _get_multisampled_sensor_values(self):
        NUM_OF_SAMPLES = 10
        WAIT_TIME = 0.1  # sec

        # Multisampling
        sensor_values = dict(self._SENSORS)
        for i in range(NUM_OF_SAMPLES):
            for key in self._SENSORS:
                sensor_values[key] += self._present_sensor_values[key]
            rospy.sleep(WAIT_TIME)

        for key in self._SENSORS:
            sensor_values[key] /= NUM_OF_SAMPLES

        return sensor_values

    def _line_sampling(self):
        self._beep_start()
        self._sensor_line_values = self._get_multisampled_sensor_values()
        self._beep_success()

        rospy.loginfo(self._sensor_line_values)
        self._line_values_are_sampled = True
        self._set_line_thresholds()

    def _filed_sampling(self):
        self._beep_start()
        self._sensor_field_values = self._get_multisampled_sensor_values()
        self._beep_success()

        rospy.loginfo(self._sensor_field_values)
        self._field_values_are_sampled = True
        self._set_line_thresholds()

    def _indicate_line_detections(self):
        led_values = LedValues()

        led_values.right_side = self._line_is_detected_by_sensor["right"]
        led_values.right_forward = self._line_is_detected_by_sensor["mid_right"]
        led_values.left_forward = self._line_is_detected_by_sensor["mid_left"]
        led_values.left_side = self._line_is_detected_by_sensor["left"]
        self._pub_leds.publish(led_values)

    def _publish_cmdvel_for_line_following(self):
        VEL_LINER_X = 0.08  # m/s
        VEL_ANGULAR_Z = 0.8  # rad/s
        LOW_VEL_ANGULAR_Z = 0.5  # rad/s

        cmd_vel = Twist()
        if not all(self._line_is_detected_by_sensor.values()) and\
                any(self._line_is_detected_by_sensor.values()):
            cmd_vel.linear.x = VEL_LINER_X

            if self._line_is_detected_by_sensor["left"]:
                cmd_vel.angular.z += VEL_ANGULAR_Z

            if self._line_is_detected_by_sensor["right"]:
                cmd_vel.angular.z -= VEL_ANGULAR_Z

            if self._line_is_detected_by_sensor["mid_left"]:
                cmd_vel.angular.z += LOW_VEL_ANGULAR_Z

            if self._line_is_detected_by_sensor["mid_right"]:
                cmd_vel.angular.z -= LOW_VEL_ANGULAR_Z

        self._pub_cmdvel.publish(cmd_vel)

    def update(self):
        if self._mouse_buttons.front:  # SW0 of Raspberry Pi Mouse
            if self._sampling_is_done() and self._can_publish_cmdvel is False:
                rospy.loginfo("start following")
                self._motor_on()
                self._beep_success()
                self._can_publish_cmdvel = True
            else:
                rospy.loginfo("stop following")
                self._motor_off()
                self._beep_failure()
                self._can_publish_cmdvel = False

        elif self._mouse_buttons.mid:  # SW1
            rospy.loginfo("line sampling:")
            self._line_sampling()

        elif self._mouse_buttons.rear:  # SW2
            rospy.loginfo("field sampling:")
            self._filed_sampling()

        if self._can_publish_cmdvel:
            self._publish_cmdvel_for_line_following()

        self._indicate_line_detections()


def main():
    rospy.init_node('line_follower')

    line_follower = LineFollower()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        line_follower.update()

        r.sleep()

if __name__ == '__main__':
    main()

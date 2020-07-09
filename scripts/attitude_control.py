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
import numpy
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger
from raspimouse_ros_2.msg import ButtonValues


class PIDController(object):
    def __init__(self, p_gain, i_gain, d_gain):

        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain

        self._error_1 = 0.0
        self._error_2 = 0.0
        self._output = 0.0

    def update(self, current, target):
        error = target - current

        delta_output = self._p_gain * (error - self._error_1)
        delta_output += self._i_gain * (error)
        delta_output += self._d_gain * (error - 2*self._error_1 + self._error_2)

        self._output += delta_output

        self._error_2 = self._error_1
        self._error_1 = error

        return self._output


class AttitudeController(object):
    _MODE_NONE = 0
    _MODE_CALIBRATION = 1
    _MODE_KEEP_ZERO_RADIAN = 2
    _MODE_ROTATION = 3

    def __init__(self):
        self._mouse_buttons = ButtonValues()
        self._imu_data_raw = Imu()

        # for acceleration low pass filter
        self._filtered_acc = Vector3()
        self._prev_acc = Vector3()

        # for self.update()
        self._current_mode = self._MODE_NONE
        self._has_motor_enabled = False

        # for angle control
        self._omega_pid_controller = PIDController(10, 0, 20)
        self._target_angle = 0.0
        self._increase_target_angle = True

        # for heading_angle calculation
        self._heading_angle = 0.0
        self._omega_bias = 0.0
        self._prev_imu_timestamp = rospy.Time()

        self._sub_buttons = rospy.Subscriber(
            'buttons', ButtonValues, self._callback_buttons, queue_size=1)
        self._sub_imu = rospy.Subscriber(
            'imu/data_raw', Imu, self._callback_imu, queue_size=1)

        self._pub_cmdvel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._pub_buzzer = rospy.Publisher('buzzer', UInt16, queue_size=1)

        try:
            rospy.wait_for_service("motor_on", timeout=5)
            rospy.wait_for_service("motor_off", timeout=5)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service not found")
            rospy.signal_shutdown(e.message)
        else:
            rospy.on_shutdown(self._motor_off)


    def _callback_buttons(self, msg):
        self._mouse_buttons = msg

    def _callback_imu(self, msg):
        self._imu_data_raw = msg

        self._calculate_heading_angle(
            self._imu_data_raw.angular_velocity.z, 
            self._imu_data_raw.header.stamp
        )

        self._filter_acceleration(self._imu_data_raw.linear_acceleration)

    def _motor_on(self):
        rospy.ServiceProxy("motor_on", Trigger).call()
        rospy.loginfo("motor_on")

    def _motor_off(self):
        rospy.ServiceProxy("motor_off", Trigger).call()
        rospy.loginfo("motor_off")

    def _calculate_heading_angle(self, omega_ref, timestamp):
        ALPHA = 1.0

        if not self._prev_imu_timestamp:
            self._prev_imu_timestamp = timestamp

        omega = ALPHA * (omega_ref - self._omega_bias)
        diff_timestamp = timestamp - self._prev_imu_timestamp
        
        self._heading_angle += omega * diff_timestamp.to_sec()
        self._prev_imu_timestamp = timestamp

    def _filter_acceleration(self, acc):
        ALPHA = 0.1

        # Simple low pass filter
        self._filtered_acc.x = ALPHA * acc.x + (1.0 - ALPHA) * self._prev_acc.x
        self._filtered_acc.y = ALPHA * acc.y + (1.0 - ALPHA) * self._prev_acc.y
        self._filtered_acc.z = ALPHA * acc.z + (1.0 - ALPHA) * self._prev_acc.z
        self._prev_acc = self._filtered_acc

    def _gyro_calibration(self):
        SAMPLE_NUM = 100
        WAIT_TIME = 0.01
        rospy.loginfo("Gyro Calibration")

        # Multisampling
        samples = []
        prev_imu_seq = 0
        for i in range(SAMPLE_NUM):
            if prev_imu_seq != self._imu_data_raw.header.seq:
                samples.append(self._imu_data_raw.angular_velocity.z)
                prev_imu_seq = self._imu_data_raw.header.seq
            rospy.sleep(WAIT_TIME)
        self._omega_bias = numpy.mean(samples)

        # Reset variables for heading angle calculation
        self._heading_angle = 0.0
        self._prev_imu_timestamp = self._imu_data_raw.header.stamp

    def _angle_control(self, target_angle=0.0):
        SIGN = -1.0

        cmdvel = Twist()
        cmdvel.angular.z = SIGN * self._omega_pid_controller.update(target_angle, self._heading_angle)

        self._pub_cmdvel.publish(cmdvel)

    def _keep_zero_radian(self):
        self._angle_control(0.0)

    def _rotation(self, start_angle = -math.pi*0.5, end_angle = math.pi*0.5):
        ADD_ANGLE = math.radians(2)

        if start_angle > end_angle:
            rospy.logwarn("Set start_angle < end_angle.")
            return

        if self._increase_target_angle:
            self._target_angle += ADD_ANGLE
        else:
            self._target_angle -= ADD_ANGLE

        if self._target_angle >= end_angle:
            self._target_angle = end_angle
            self._increase_target_angle = False
        elif self._target_angle <= start_angle:
            self._target_angle = start_angle
            self._increase_target_angle = True

        self._angle_control(self._target_angle)

    def _beep_buzzer(self, freq, beep_time=0):
        self._pub_buzzer.publish(freq)
        rospy.sleep(beep_time)
        self._pub_buzzer.publish(0)

    def _beep_success(self):
        self._beep_buzzer(1000, 0.1)
        rospy.sleep(0.1)
        self._beep_buzzer(1000, 0.1)

    def _beep_failure(self):
        for i in range(4):
            self._beep_buzzer(500, 0.1)
            rospy.sleep(0.1)

    def _suggest_mode_from_buttons(self):
        suggest = self._MODE_NONE
        if self._mouse_buttons.front:
            suggest = self._MODE_CALIBRATION

            # wait for release the button
            while self._mouse_buttons.front:
                pass

        elif self._mouse_buttons.mid:
            suggest = self._MODE_KEEP_ZERO_RADIAN

            # wait for release the button
            while self._mouse_buttons.mid:
                pass

        elif self._mouse_buttons.rear:
            suggest = self._MODE_ROTATION

            # wait for release the button
            while self._mouse_buttons.rear:
                pass

        return suggest

    def _has_stop_signal(self):
        output = False

        # Any button has pressed.
        if self._mouse_buttons.front or self._mouse_buttons.mid or self._mouse_buttons.rear:
            output = True
            while self._mouse_buttons.front or self._mouse_buttons.mid or self._mouse_buttons.rear:
                pass

        # The mouse body has rotated.
        if self._filtered_acc.z > 0.0:
            output = True

        return output

    def _select_mode(self):
        suggest = self._MODE_NONE
        suggest = self._suggest_mode_from_buttons()

        if suggest == self._MODE_CALIBRATION:
            rospy.loginfo("Calibration")
        elif suggest == self._MODE_KEEP_ZERO_RADIAN:
            rospy.loginfo("Keep zero radian")
        elif suggest == self._MODE_ROTATION:
            rospy.loginfo("Rotation")

        if suggest != self._MODE_NONE:
            self._current_mode = suggest
            self._beep_success()
            rospy.sleep(1.0)

    def update(self):
        if self._current_mode == self._MODE_NONE:
            self._select_mode()

        elif self._current_mode == self._MODE_CALIBRATION:
            rospy.sleep(1)
            self._gyro_calibration()
            self._current_mode = self._MODE_NONE
            self._beep_success()

        else:
            if not self._has_motor_enabled:
                self._motor_on()
                self._has_motor_enabled = True

            if self._current_mode == self._MODE_KEEP_ZERO_RADIAN:
                self._keep_zero_radian()

            elif self._current_mode == self._MODE_ROTATION:
                self._rotation()

            if self._has_stop_signal():
                self._motor_off()
                self._has_motor_enabled = False
                self._target_angle = 0.0
                self._beep_failure()
                self._current_mode = self._MODE_NONE


def main():
    rospy.init_node('attitude_control')

    controller = AttitudeController()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        controller.update()
        r.sleep()

if __name__ == '__main__':
    main()

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
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from raspimouse_ros_2.msg import LightSensorValues
from raspimouse_ros_2.msg import ButtonValues
from raspimouse_ros_2.msg import LedValues

class JoyWrapper(object):
    def __init__(self):

        self._BUTTON_SHUTDOWN_1 = rospy.get_param('~button_shutdown_1')
        self._BUTTON_SHUTDOWN_2 = rospy.get_param('~button_shutdown_2')

        self._BUTTON_MOTOR_ON = rospy.get_param('~button_motor_on')
        self._BUTTON_MOTOR_OFF = rospy.get_param('~button_motor_off')

        self._BUTTON_CMD_ENABLE = rospy.get_param('~button_cmd_enable')
        self._AXIS_CMD_LINEAR_X = rospy.get_param('~axis_cmd_linear_x')
        self._AXIS_CMD_ANGULAR_Z = rospy.get_param('~axis_cmd_angular_z')

        self._ANALOG_D_PAD = rospy.get_param('~analog_d_pad')
        self._D_PAD_UP = rospy.get_param('~d_pad_up')
        self._D_PAD_DOWN = rospy.get_param('~d_pad_down')
        self._D_PAD_LEFT = rospy.get_param('~d_pad_left')
        self._D_PAD_RIGHT = rospy.get_param('~d_pad_right')
        self._D_UP_IS_POSITIVE = rospy.get_param('~d_pad_up_is_positive')
        self._D_RIGHT_IS_POSITIVE = rospy.get_param('~d_pad_right_is_positive')

        self._BUTTON_BUZZER_ENABLE = rospy.get_param('~button_buzzer_enable')
        self._DPAD_BUZZER0 = rospy.get_param('~dpad_buzzer0')
        self._DPAD_BUZZER1 = rospy.get_param('~dpad_buzzer1')
        self._DPAD_BUZZER2 = rospy.get_param('~dpad_buzzer2')
        self._DPAD_BUZZER3 = rospy.get_param('~dpad_buzzer3')
        self._BUTTON_BUZZER4 = rospy.get_param('~button_buzzer4')
        self._BUTTON_BUZZER5 = rospy.get_param('~button_buzzer5')
        self._BUTTON_BUZZER6 = rospy.get_param('~button_buzzer6')
        self._BUTTON_BUZZER7 = rospy.get_param('~button_buzzer7')

        self._BUTTON_SENSOR_SOUND_EN = rospy.get_param('~button_sensor_sound_en')
        self._BUTTON_CONFIG_ENABLE = rospy.get_param('~button_config_enable')

        # for _joy_velocity_config()
        self._MAX_VEL_LINEAR_X = 2.0 # m/s
        self._MAX_VEL_ANGULAR_Z = 2.0 * math.pi # rad/s
        self._DEFAULT_VEL_LINEAR_X = 0.5 # m/s
        self._DEFAULT_VEL_ANGULAR_Z = 1.0 * math.pi # rad/s

        self._joy_msg = None
        self._lightsensor = LightSensorValues()
        self._mouse_buttons = ButtonValues()
        self._cmdvel_has_value = False
        self._buzzer_has_value = False
        self._sensor_sound_has_value = False
        self._vel_linear_x = self._DEFAULT_VEL_LINEAR_X
        self._vel_angular_z = self._DEFAULT_VEL_ANGULAR_Z

        self._pub_cmdvel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._pub_buzzer = rospy.Publisher('buzzer', UInt16, queue_size=1)
        self._pub_leds = rospy.Publisher('leds', LedValues, queue_size=1)
        self._sub_joy = rospy.Subscriber('joy', Joy, self._callback_joy, queue_size=1)
        self._sub_lightsensor = rospy.Subscriber('lightsensors', LightSensorValues,
                self._callback_lightsensor, queue_size=1)
        self._sub_buttons = rospy.Subscriber('buttons', ButtonValues,
                self._callback_buttons, queue_size=1)

        try:
            rospy.wait_for_service("motor_on", timeout=5)
            rospy.wait_for_service("motor_off", timeout=5)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service not found")
            rospy.signal_shutdown(e.message)
        else:
            rospy.on_shutdown(self._motor_off)
            self._motor_on()


    def _callback_joy(self, msg):
        self._joy_msg = msg


    def _callback_lightsensor(self, msg):
        self._lightsensor = msg


    def _callback_buttons(self, msg):
        self._mouse_buttons = msg


    def _motor_on(self):
        rospy.ServiceProxy("motor_on", Trigger).call()
        rospy.loginfo("motor_on")

    def _motor_off(self):
        rospy.ServiceProxy("motor_off", Trigger).call()
        rospy.loginfo("motor_off")


    def _joy_dpad(self, joy_msg, target_pad, positive_on):
        # d pad inputs of f710 controller are analog
        # d pad inputs of dualshock3 controller are digital
        if self._ANALOG_D_PAD:
            if positive_on:
                return joy_msg.axes[target_pad] > 0
            else:
                return joy_msg.axes[target_pad] < 0
        else:
            return joy_msg.buttons[target_pad]

    def _dpad_up(self, joy_msg):
        positive_on = self._D_UP_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_UP, positive_on)

    def _dpad_down(self, joy_msg):
        positive_on = not self._D_UP_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_DOWN, positive_on)

    def _dpad_left(self, joy_msg):
        positive_on = not self._D_RIGHT_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_LEFT, positive_on)

    def _dpad_right(self, joy_msg):
        positive_on = self._D_RIGHT_IS_POSITIVE
        return self._joy_dpad(joy_msg, self._D_PAD_RIGHT, positive_on)

    def _dpad(self, joy_msg, target):
        if target == "up":
            return self._dpad_up(joy_msg)
        elif target == "down":
            return self._dpad_down(joy_msg)
        elif target == "left":
            return self._dpad_left(joy_msg)
        elif target == "right":
            return self._dpad_right(joy_msg)
        else:
            return False


    def _joy_shutdown(self, joy_msg):
        if joy_msg.buttons[self._BUTTON_SHUTDOWN_1] and\
                joy_msg.buttons[self._BUTTON_SHUTDOWN_2]:

            self._pub_cmdvel.publish(Twist())
            self._pub_buzzer.publish(UInt16())
            self._pub_leds.publish(LedValues())
            self._motor_off()
            rospy.signal_shutdown('finish')


    def _joy_motor_onoff(self, joy_msg):
        if joy_msg.buttons[self._BUTTON_MOTOR_ON]:
            self._motor_on()

        if joy_msg.buttons[self._BUTTON_MOTOR_OFF]:
            self._motor_off()


    def _joy_cmdvel(self, joy_msg):
        cmdvel = Twist()
        if joy_msg.buttons[self._BUTTON_CMD_ENABLE]:
            cmdvel.linear.x = self._vel_linear_x * joy_msg.axes[self._AXIS_CMD_LINEAR_X]
            cmdvel.angular.z = self._vel_angular_z * joy_msg.axes[self._AXIS_CMD_ANGULAR_Z]
            rospy.loginfo(cmdvel)
            self._pub_cmdvel.publish(cmdvel)

            self._cmdvel_has_value = True
        else:
            if self._cmdvel_has_value:
                self._pub_cmdvel.publish(cmdvel)
                self._cmdvel_has_value = False


    def _joy_buzzer_freq(self, joy_msg):
        freq = UInt16()
        buttons = [
                self._dpad(joy_msg,self._DPAD_BUZZER0),
                self._dpad(joy_msg,self._DPAD_BUZZER1),
                self._dpad(joy_msg,self._DPAD_BUZZER2),
                self._dpad(joy_msg,self._DPAD_BUZZER3),
                joy_msg.buttons[self._BUTTON_BUZZER4],
                joy_msg.buttons[self._BUTTON_BUZZER5],
                joy_msg.buttons[self._BUTTON_BUZZER6],
                joy_msg.buttons[self._BUTTON_BUZZER7],
                ]
        # buzzer frequency Hz
        SCALES = [
                523, 587, 659, 699, 
                784, 880, 987, 1046
                ]
        
        if joy_msg.buttons[self._BUTTON_BUZZER_ENABLE]:
            for i, button in enumerate(buttons):
                if button:
                    freq.data = SCALES[i]
                    break
            self._pub_buzzer.publish(freq)
            rospy.loginfo(freq)

            self._buzzer_has_value = True
        else:
            if self._buzzer_has_value:
                self._pub_buzzer.publish(freq)
                self._buzzer_has_value = False


    def _joy_lightsensor_sound(self, joy_msg):
        freq = UInt16()
        if joy_msg.buttons[self._BUTTON_SENSOR_SOUND_EN]:
            rospy.loginfo(self._lightsensor)
            freq.data += self._positive(self._lightsensor.left_side)
            freq.data += self._positive(self._lightsensor.left_forward)
            freq.data += self._positive(self._lightsensor.right_forward)
            freq.data += self._positive(self._lightsensor.right_side)

            self._pub_buzzer.publish(freq)
            self._sensor_sound_has_value = True
        else:
            if self._sensor_sound_has_value:
                self._pub_buzzer.publish(freq)
                self._sensor_sound_has_value = False

    def _positive(self, value):
        if value < 0:
            return 0
        else:
            return value

    
    def _joy_velocity_config(self, joy_msg):
        ADD_VEL_LINEAR_X = 0.1 # m/s
        ADD_VEL_ANGULAR_Z = 0.1 * math.pi # m/s
        BUZZER_FREQ_ADD = 880 # Hz
        BUZZER_FREQ_SUB = 440 # Hz
        BUZZER_FREQ_RESET = 660 # Hz
        BUZZER_BEEP_TIME = 0.2 # sec

        if joy_msg.buttons[self._BUTTON_CONFIG_ENABLE]:
            if self._mouse_buttons.front:
                self._vel_linear_x = self._config_velocity(
                        self._vel_linear_x, ADD_VEL_LINEAR_X, 
                        0, self._MAX_VEL_LINEAR_X)
                self._vel_angular_z = self._config_velocity(
                        self._vel_angular_z, ADD_VEL_ANGULAR_Z,
                        0, self._MAX_VEL_ANGULAR_Z)

                self._beep_buzzer(BUZZER_FREQ_ADD, BUZZER_BEEP_TIME)
                # wait for release the button
                while self._mouse_buttons.front:
                    pass
            elif self._mouse_buttons.rear:
                self._vel_linear_x = self._config_velocity(
                        self._vel_linear_x, -ADD_VEL_LINEAR_X, 
                        0, self._MAX_VEL_LINEAR_X)
                self._vel_angular_z = self._config_velocity(
                        self._vel_angular_z, -ADD_VEL_ANGULAR_Z,
                        0, self._MAX_VEL_ANGULAR_Z)

                self._beep_buzzer(BUZZER_FREQ_SUB, BUZZER_BEEP_TIME)
                # wait for release the button
                while self._mouse_buttons.rear:
                    pass
            elif self._mouse_buttons.mid:
                self._vel_linear_x = self._DEFAULT_VEL_LINEAR_X
                self._vel_angular_z = self._DEFAULT_VEL_ANGULAR_Z

                self._beep_buzzer(BUZZER_FREQ_RESET, BUZZER_BEEP_TIME)
                # wait for release the button
                while self._mouse_buttons.mid:
                    pass

            rospy.loginfo(
                    "linear_x:" + str(self._vel_linear_x) +\
                    ", angular_z:" + str(self._vel_angular_z)
                    )

    def _config_velocity(self, current, add, lowerlimit, upperlimit):
        output = current + add
        
        if output < lowerlimit:
            output = lowerlimit
        if output > upperlimit:
            output = upperlimit

        return output
    
    def _beep_buzzer(self, freq, beep_time=0):
        self._pub_buzzer.publish(freq)
        rospy.sleep(beep_time)
        self._pub_buzzer.publish(0)

    def _joy_leds(self, joy_msg):
        led_values = LedValues()

        if joy_msg.buttons[self._BUTTON_CMD_ENABLE]:
            led_values.right_side = True

        if joy_msg.buttons[self._BUTTON_BUZZER_ENABLE]:
            led_values.right_forward = True

        if joy_msg.buttons[self._BUTTON_SENSOR_SOUND_EN]:
            led_values.left_forward = True

        if joy_msg.buttons[self._BUTTON_CONFIG_ENABLE]:
            led_values.left_side = True

        self._pub_leds.publish(led_values)

    def update(self):
        if self._joy_msg is None:
            return 

        self._joy_motor_onoff(self._joy_msg)
        self._joy_cmdvel(self._joy_msg)
        self._joy_buzzer_freq(self._joy_msg)
        self._joy_lightsensor_sound(self._joy_msg)
        self._joy_velocity_config(self._joy_msg)
        self._joy_leds(self._joy_msg)

        self._joy_shutdown(self._joy_msg)


def main():
    rospy.init_node('joystick_control')

    joy_wrapper = JoyWrapper()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        joy_wrapper.update()

        r.sleep()

if __name__ == '__main__':
    main()

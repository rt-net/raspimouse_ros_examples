#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from raspimouse_ros_2.msg import LightSensorValues
from raspimouse_ros_2.msg import ButtonValues
from raspimouse_ros_2.msg import LedValues


class LineTracer(object):
    def __init__(self):
        self._SENSORS = {"left":0, "mid_left":0, "mid_right":0, "right":0}
        self._line_cal_value = dict(self._SENSORS)
        self._field_cal_value = dict(self._SENSORS)
        self._sensor_threshold = dict(self._SENSORS)
        self._sensor_on_line = dict(self._SENSORS)
        self._line_is_bright = False
        self._line_is_calibrated = False
        self._field_is_calibrated = False

        self._tracing = False

        self._lightsensor = dict(self._SENSORS)
        self._mouse_buttons = ButtonValues()

        self._pub_cmdvel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._pub_buzzer = rospy.Publisher('buzzer', UInt16, queue_size=1)
        self._pub_leds = rospy.Publisher('leds', LedValues, queue_size=1)

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
            rospy.on_shutdown(self._on_shutdown)


    def _on_shutdown(self):
        self._motor_off()
        led_values = LedValues()
        self._pub_leds.publish(led_values)


    def _callback_lightsensor(self, msg):
        self._lightsensor["left"] = msg.right_forward
        self._lightsensor["mid_left"] = msg.right_side
        self._lightsensor["mid_right"] = msg.left_side
        self._lightsensor["right"] = msg.left_forward


        if self._calibration_is_done():
            self._update_sensor_on_line_status()


    def _update_sensor_on_line_status(self):
        for key in self._SENSORS:
            is_positive = self._lightsensor[key] > self._sensor_threshold[key]

            if self._line_is_bright == is_positive:
                self._sensor_on_line[key] = True
            else:
                self._sensor_on_line[key] = False


    def _callback_buttons(self, msg):
        self._mouse_buttons = msg


    def _motor_on(self):
        rospy.ServiceProxy("motor_on", Trigger).call()
        rospy.loginfo("motor_on")

    def _motor_off(self):
        rospy.ServiceProxy("motor_off", Trigger).call()
        rospy.loginfo("motor_off")


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


    def _calibration_is_done(self):
        if self._line_is_calibrated and self._field_is_calibrated:
            return True
        else:
            return False


    def _median(self, sensor1, sensor2):
        diff = math.fabs(sensor1 - sensor2)
        if sensor1 < sensor2:
            return sensor1 + diff * 0.5
        else:
            return sensor2 + diff * 0.5


    def _set_threshold(self):
        if not self._calibration_is_done():
            return

        if self._line_cal_value["right"] > self._field_cal_value["right"]:
            self._line_is_bright = True
        else:
            self._line_is_bright = False

        for key in self._SENSORS:
            self._sensor_threshold[key] = self._median(
                    self._line_cal_value[key], 
                        self._field_cal_value[key])

        rospy.loginfo("threshold:" + str(self._sensor_threshold))


    def _calibration(self):
        NUM_OF_SAMPLES = 100
        WAIT_TIME = 0.01 # sec

        sum_sensor_value = dict(self._SENSORS)
        for i in range(NUM_OF_SAMPLES):
            for key in self._SENSORS:
                sum_sensor_value[key] += self._lightsensor[key]
            rospy.sleep(WAIT_TIME)

        sensor_value = dict(self._SENSORS)
        for key in self._SENSORS:
            sensor_value[key] = sum_sensor_value[key] / NUM_OF_SAMPLES

        return sensor_value


    def _line_calibration(self):
        self._beep_start()
        self._line_cal_value = self._calibration()
        self._beep_success()
        rospy.loginfo(self._line_cal_value)
        self._line_is_calibrated = True
        self._set_threshold()


    def _filed_calibration(self):
        self._beep_start()
        self._field_cal_value = self._calibration()
        self._beep_success()
        rospy.loginfo(self._field_cal_value)
        self._field_is_calibrated = True
        self._set_threshold()


    def _indicate_lines(self):
        led_values = LedValues()

        if self._sensor_on_line["right"]:
            led_values.right_side = True

        if self._sensor_on_line["mid_right"]:
            led_values.right_forward = True

        if self._sensor_on_line["mid_left"]:
            led_values.left_forward = True

        if self._sensor_on_line["left"]:
            led_values.left_side  = True

        self._pub_leds.publish(led_values)


    def _pub_trace_cmdvel(self):
        VEL_LINER_X = 0.08 # m/s
        VEL_ANGULAR_Z = 0.8 # rad/s
        SMALL_VEL_ANGULAR_Z = 0.5 # rad/s

        some_sensors_on_line = False
        all_sensors_on_line = True
        for key in self._SENSORS:
            if self._sensor_on_line[key]:
                if some_sensors_on_line is False:
                    some_sensors_on_line = True
            else:
                all_sensors_on_line = False

        cmd_vel = Twist()
        if some_sensors_on_line is True and all_sensors_on_line is False:
            cmd_vel.linear.x = VEL_LINER_X

            if self._sensor_on_line["left"]:
                cmd_vel.angular.z += VEL_ANGULAR_Z

            if self._sensor_on_line["right"]:
                cmd_vel.angular.z -= VEL_ANGULAR_Z

            if self._sensor_on_line["mid_left"]:
                cmd_vel.angular.z += SMALL_VEL_ANGULAR_Z

            if self._sensor_on_line["mid_right"]:
                cmd_vel.angular.z -= SMALL_VEL_ANGULAR_Z

        self._pub_cmdvel.publish(cmd_vel)


    def update(self):
        if self._mouse_buttons.front:
            if self._calibration_is_done() and self._tracing is False:
                rospy.loginfo("start trace")
                self._motor_on()
                self._tracing = True
                self._beep_success()
            else:
                rospy.loginfo("stop trace")
                self._motor_off()
                self._tracing = False
                self._beep_failure()

        elif self._mouse_buttons.mid:
            rospy.loginfo("line calibration:")
            self._line_calibration()
        elif self._mouse_buttons.rear:
            rospy.loginfo("field calibration:")
            self._filed_calibration()

        if self._tracing:
            self._pub_trace_cmdvel()

        self._indicate_lines()


def main():
    rospy.init_node('line_trace')

    line_tracer = LineTracer()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        line_tracer.update()

        r.sleep()

if __name__ == '__main__':
    main()

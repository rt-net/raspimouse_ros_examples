[English](README.en.md) | [日本語](README.md)

# raspimouse_ros_examples

[![Build Status](https://travis-ci.com/rt-net/raspimouse_ros_examples.svg?token=44UfTwwGaAupMGxC2ZWA&branch=master)](https://travis-ci.com/rt-net/raspimouse_ros_examples)

Sample applications for Raspberry Pi Mouse with ROS.

![raspberry_pi_mouse](https://github.com/rt-net/raspimouse_ros_examples/blob/images/raspberry_pi_mouse.JPG)

## Requirements

- Raspberry Pi Mouse
  - https://www.rt-net.jp/products/raspimouse2
  - Linux OS
    - Ubuntu server 16.04
    - Ubuntu server 18.04
    - https://wiki.ubuntu.com/ARM/RaspberryPi
  - ROS
    - [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
    - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
  - Raspberry Pi Mouse ROS package
    - https://github.com/ryuichiueda/raspimouse_ros_2
- Remote Computer (Optional)
  - ROS
    - [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
    - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
  - Raspberry Pi Mouse ROS package
    - https://github.com/ryuichiueda/raspimouse_ros_2

## Installation

```sh
cd ~/catkin_ws/src
# Clone ROS packages
git clone https://github.com/ryuichiueda/raspimouse_ros_2
git clone https://github.com/rt-net/raspimouse_ros_examples 

# Install dependencies
rosdep install -r -y --from-paths . --ignore-src      

# make & install
cd ~/catkin_ws && catkin_make
source devel.setup.bash
```

## License

This repository is licensed under the Apache 2.0, see [LICENSE](./LICENSE) for details.

## How To Use Examples

- [keyboard_control](#keyboard_control)
- [joystick_control](#joystick_control)
- [object_tracking](#object_tracking)
- [line_tracing](#line_tracing)
- [SLAM](#SLAM)

---

### keyboard_control

This is an example to use [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package to send velocity command for Raspberry Pi Mouse. 

#### Requirements 

- Keyboard

#### How to use

Launch nodes with the following command:

```sh
roslaunch raspimouse_ros_examples teleop.launch key:=true

# Control from remote computer
roslaunch raspimouse_ros_examples teleop.launch key:=true mouse:=false
```

Then, call `/motor_on` service to enable motor control with the following command:

```sh
rosservice call /motor_on
```
[back to example list](#how-to-use-examples)

---

### joystick_control

This is an example to use joystick controller to control a Raspberry Pi Mouse.

#### Requirements 

- Joystick Controller
  - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
  - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)

#### How to use

Launch nodes with the following command:

```sh
roslaunch raspimouse_ros_examples teleop.launch joy:=true

# Use DUALSHOCK 3
roslaunch raspimouse_ros_examples teleop.launch joy:=true joyconfig:="dualshock3" 

# Control from remote computer
roslaunch raspimouse_ros_examples teleop.launch joy:=true mouse:=false
```

This picture shows the default key configuration.

![joystick_control_keyconfig](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/joystick_control_keyconfig.png)

#### Configure

Key assignments can be edited with key numbers in [./config/joy_f710.yml](./config/joy_f710.yml) or [./config/joy_dualshock3.yml](./config/joy_dualshock3.yml).

```yaml
button_shutdown_1       : 8
button_shutdown_2       : 9

button_motor_off        : 8
button_motor_on         : 9

button_cmd_enable       : 4
```
[back to example list](#how-to-use-examples)

--- 

### object_tracking

![object_tracking](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/object_tracking.JPG)

This is an example to use RGB camera images and OpenCV library for object tracking.

#### Requirements 

- Web camera
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- Camera mount
  - [Raspberry Pi Mouse Option kit No.4 \[Webcam mount\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584&language=en)
- Orange ball（Optional）
  - [Soft Ball (Orange)](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701&language=en)
- Software
  - python
    - opencv
    - numpy
  - v4l-utils

#### Installation

Install a camera mount and a web camera to Raspberry Pi Mouse, then connect the camera to the Raspberry Pi．

Next, install the v4l-utils package with the following command:

```sh
sudo apt install v4l-utils
```
#### How to use

Turn off automatic adjustment parameters of a camera (auto focus, auto while balance, etc.) with the following command:

```sh
rosrun raspimouse_ros_examples camera.bash
```

Then, launch nodes with the following command:

```sh
roslaunch raspimouse_ros_examples pimouse_object_tracking.launch
```

Browse \[IP address of Raspbery Pi Mouse\]:8080 to show a camera image and a tracking result.

![web_video_server](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/web_video_server.png)

#### Configure

Edit [`./scripts/object_tracking.py`](./scripts/object_tracking.py) to change a color of tracking target.

```python
    def detect_ball(self):
        # ~~~ 省略 ~~~
        min_hsv, max_hsv = self.set_color_orange()
        # min_hsv, max_hsv = self.set_color_green()
        # min_hsv, max_hsv = self.set_color_blue()
```

If object tracking is unstable, please edit the following lines.

```python
    def set_color_orange(self):
        # [H(0~180), S(0~255), V(0~255)]
        # min_hsv_orange = np.array([15, 200, 80])
        min_hsv_orange = np.array([15, 150, 40])
        max_hsv_orange = np.array([20, 255, 255])
        return min_hsv_orange, max_hsv_orange
```
[back to example list](#how-to-use-examples)

--- 

### line_tracing

![mouse_with_line_trace_sensor](https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_line_trace_sensor.JPG)

This is an example for line tracing.

#### Requirements

- Line tracing sensor
  - [Raspberry Pi Mouse Option kit No.3 \[Line trace\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591&language=en)
- Field and lines for tracing (Optional)

#### Installation

Install a line tracing sensor unit to Raspberry Pi Mouse.

#### How to use

Launch nodes with the following command:

```sh
roslaunch raspimouse_ros_examples line_trace.launch

# Control from remote computer
roslaunch raspimouse_ros_examples line_trace.launch mouse:=false
```

Next, place Raspberry Pi Mouse on a field and press SW2 to sample sensor values on the field.

![field_calibration](https://github.com/rt-net/raspimouse_ros_examples/blob/images/field_calibration.JPG)

Then, place Raspberry Pi Mouse to detect a line and press SW1 to sample sensor values on the line.

![line_calibration](https://github.com/rt-net/raspimouse_ros_examples/blob/images/line_calibration.JPG)

Last, place Raspberry Pi Mouse on the line and press SW0 to start line tracing.

![start_trace](https://github.com/rt-net/raspimouse_ros_examples/blob/images/start_trace.JPG)

Press SW0 again to stop the tracing.

[back to example list](#how-to-use-examples)

--- 

### SLAM

![slam_gmapping](https://github.com/rt-net/raspimouse_ros_examples/blob/images/slam_gmapping.png)

This is an example to use LiDAR for SLAM (Simultaneous Localization And Mapping).

#### Requirements 

- LiDAR
  - URG
    - [URG-04LX-UG01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1296&products_id=2816&language=en)
    - [Raspberry Pi Mouse Option kit No.2 \[URG mount\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3517&language=en)
  - RPLIDAR
    - [Raspberry Pi Mouse Option kit No.6 \[LiDAR\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3770&language=en)
- Joystick Controller (Optional)
  
#### Installation

Install a LiDAR to the Raspberry Pi Mouse.

- URG
  - ![mouse_with_urg](https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_urg.JPG)
- RPLIDAR
  - ![mouse_with_rplidar](https://github.com/rt-net/raspimouse_ros_examples/blob/images/mouse_with_rpilidar.JPG)
  
#### How to use

Launch nodes on Raspberry Pi Mouse with the following command:

```sh
# URG
roslaunch raspimouse_ros_examples mouse_with_lidar.launch urg:=true port:=/dev/ttyACM0

# RPLIDAR
roslaunch raspimouse_ros_examples mouse_with_lidar.launch rplidar=true port:=/dev/ttyUSB0
```

Next, launch `teleop.launch` to control Raspberry Pi Mouse with the following command:

```sh
# joystick control
roslaunch raspimouse_ros_examples teleop.launch mouse:=false joy:=true joyconfig:=dualshock3
```

Then, launch SLAM packages (on a remote computer recommend) with the following command:

```sh
# URG
roslaunch raspimouse_ros_examples slam_gmapping.launch urg:=true

# RPLIDAR
roslaunch raspimouse_ros_examples slam_gmapping.launch rplidar:=true
```

After moving Raspberry Pi Mouse and makeing a map, run a node to save the map with the following command:

```sh
mkdir ~/maps
rosrun map_server map_saver -f ~/maps/mymap
```

#### Configure

Edit [./launch/slam_gmapping.launch](./launch/slam_gmapping.launch) to configure parameters of [gmapping](http://wiki.ros.org/gmapping) package.

```xml
  <node pkg="gmapping" type="slam_gmapping" name="raspimouse_slam_gmapping" output="screen">
    <!-- <remap from="scan" to="base_scan"/> -->
    <param name="base_frame" value="base_link" />
    <param name="odom_frame" value="odom" />
    <param name="map_frame"  value="map" />
    <param name="map_update_interval" value="1.0"/>
    <param name="maxUrange" value="5.6" if="$(arg urg)"/>
    <param name="maxUrange" value="12" if="$(arg rplidar)"/>
    <!-- <param name="sigma" value="0.05"/> -->
    <!-- <param name="kernelSize" value="1"/> -->
    <!-- <param name="lstep" value="0.05"/> -->
```
[back to example list](#how-to-use-examples)

[English](README.en.md) | [日本語](README.md)

# raspimouse_ros_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

ROS examples for Raspberry Pi Mouse.

ROS 2 examples is [here](https://github.com/rt-net/raspimouse_ros2_examples).

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/raspberry_pi_mouse.JPG width=500>

## Requirements

- Raspberry Pi Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - Linux OS
    - Ubuntu server 16.04
    - Ubuntu server 18.04
    - Ubuntu server 20.04
    - https://ubuntu.com/download/raspberry-pi
  - Device Driver
    - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
  - ROS
    - [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
    - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - [Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu)
  - Raspberry Pi Mouse ROS package
    - https://github.com/ryuichiueda/raspimouse_ros_2
- Remote Computer (Optional)
  - ROS
    - [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
    - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - [Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu)
  - Raspberry Pi Mouse ROS package
    - https://github.com/ryuichiueda/raspimouse_ros_2

## Installation

```sh
cd ~/catkin_ws/src
# Clone ROS packages
git clone https://github.com/ryuichiueda/raspimouse_ros_2
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_ros_examples
# For direction control example
git clone https://github.com/rt-net/rt_usb_9axisimu_driver

# Install dependencies
rosdep install -r -y --from-paths . --ignore-src      

# make & install
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

## License

This repository is licensed under the Apache 2.0, see [LICENSE](./LICENSE) for details.

## How To Use Examples

- [keyboard_control](#keyboard_control)
- [joystick_control](#joystick_control)
- [object_tracking](#object_tracking)
- [line_follower](#line_follower)
- [SLAM](#SLAM)
- [direction_control](#direction_control)

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

![joystick_control_keyconfig](https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png)

#### Configure

Key assignments can be edited with key numbers in [./config/joy_f710.yml](./config/joy_f710.yml) or [./config/joy_dualshock3.yml](./config/joy_dualshock3.yml).

```yaml
button_shutdown_1       : 8
button_shutdown_2       : 9

button_motor_off        : 8
button_motor_on         : 9

button_cmd_enable       : 4
```

#### Videos

[![joystick_control](http://img.youtube.com/vi/GswxdB8Ia0Y/sddefault.jpg)](https://youtu.be/GswxdB8Ia0Y)

[back to example list](#how-to-use-examples)

--- 

### object_tracking

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking.JPG width=500 />

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
roslaunch raspimouse_ros_examples object_tracking.launch
```
This sample publishes `binary` and `object` topics for the object detection image.
These images can be viewed with [RViz](http://wiki.ros.org/ja/rviz)
or [rqt_image_view](http://wiki.ros.org/rqt_image_view).

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/object_tracking_ros.png width=500 />

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

#### Videos

[![object_tracking](http://img.youtube.com/vi/U6_BuvrjyFc/sddefault.jpg)](https://youtu.be/U6_BuvrjyFc)

[back to example list](#how-to-use-examples)

--- 

### line_follower

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_line_trace_sensor.JPG width=500 />

This is an example for line following.

#### Requirements

- Line following sensor
  - [Raspberry Pi Mouse Option kit No.3 \[Line follower\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591&language=en)
- Field and lines for following (Optional)

#### Installation

Install a line following sensor unit to Raspberry Pi Mouse.

#### How to use

Launch nodes with the following command:

```sh
roslaunch raspimouse_ros_examples line_follower.launch

# Control from remote computer
roslaunch raspimouse_ros_examples line_follower.launch mouse:=false
```

Next, place Raspberry Pi Mouse on a field and press SW2 to sample sensor values on the field.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/field_calibration.JPG width=500 />

Then, place Raspberry Pi Mouse to detect a line and press SW1 to sample sensor values on the line.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/line_calibration.JPG width=500 />

Last, place Raspberry Pi Mouse on the line and press SW0 to start line following.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/start_trace.JPG width=500 />

Press SW0 again to stop the following.

#### Configure

Edit [`./scripts/line_follower.py`](./scripts/line_follower.py) to change a velocity command.

```python
    def _publish_cmdvel_for_line_following(self):
        VEL_LINER_X = 0.08 # m/s
        VEL_ANGULAR_Z = 0.8 # rad/s
        LOW_VEL_ANGULAR_Z = 0.5 # rad/s

        cmd_vel = Twist()
```

#### Videos

[![line_follower](http://img.youtube.com/vi/oPm0sW2V_tY/sddefault.jpg)](https://youtu.be/oPm0sW2V_tY)

[back to example list](#how-to-use-examples)

--- 

### SLAM

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/slam_gmapping.png width=500 />

This is an example to use LiDAR for SLAM (Simultaneous Localization And Mapping).

#### Requirements 

- LiDAR
  - [URG-04LX-UG01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1296&products_id=2816&language=en)
  <!-- - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1) -->
  - [LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676&language=en)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867&language=en)
- Joystick Controller (Optional)
  
#### Installation

Install a LiDAR to the Raspberry Pi Mouse.

- URG-04LX-UG01
  - <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_urg.JPG width=500 />
<!-- - RPLIDAR A1
  - <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_rplidar.png width=500 /> -->
- LDS-01
  - <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_lds01.JPG width=500 />
  
#### How to use

Launch nodes on Raspberry Pi Mouse with the following command:

```sh
# URG
roslaunch raspimouse_ros_examples mouse_with_lidar.launch urg:=true port:=/dev/ttyACM0

# LDS
roslaunch raspimouse_ros_examples mouse_with_lidar.launch lds:=true port:=/dev/ttyUSB0
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

# LDS
roslaunch raspimouse_ros_examples slam_gmapping.launch lds:=true
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
    <param name="maxUrange" value="3.5" if="$(arg lds)"/>
    <!-- <param name="sigma" value="0.05"/> -->
    <!-- <param name="kernelSize" value="1"/> -->
    <!-- <param name="lstep" value="0.05"/> -->
```

#### Videos

[![slam_urg](http://img.youtube.com/vi/gWozU47UqVE/sddefault.jpg)](https://youtu.be/gWozU47UqVE)

[![slam_urg](http://img.youtube.com/vi/hV68UqAntfo/sddefault.jpg)](https://youtu.be/hV68UqAntfo)

[back to example list](#how-to-use-examples)

---

### direction_control

<img src=https://www.rt-net.jp/wp-content/uploads/2018/02/img-usb9s_01.png width=500 />

This is an example to use an IMU sensor for direction control.

#### Requirements

- [USB output 9 degrees IMU sensor module](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3416&language=en)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)
- RT-USB-9axisIMU ROS Package.
  - https://github.com/rt-net/rt_usb_9axisimu_driver

#### Installation

Install the IMU sensor module to the LiDAR mount.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_2.JPG width=500 />

Install the LiDAR mount to the Raspberry Pi Mouse.

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_1.JPG width=500 />

#### How to use

Launch nodes on Raspberry Pi Mouse with the following command:

```sh
roslaunch raspimouse_ros_examples direction_control.launch
```

Then, press SW0 ~ SW2 to change the control mode as following,

- SW0: Calibrate the gyroscope bias and reset a heading angle of Raspberry Pi Mouse to 0 rad.
- SW1: Start a direction control to keep the heading angle to 0 rad.
  - Press SW0 ~ SW2 or tilt the body to sideways to finish the control.
- SW2: Start a direction control to change the heading angle to `-π ~ π rad`.
  - Press SW0 ~ SW2 or tilt the body to sideways to finish the control.

#### Configure

Edit [`./scripts/direction_control.py`](./scripts/direction_control.py)
to configure gains of a PID controller for the direction control.

```python
class DirectionController(object):
    # ---
    def __init__(self):
        # ---
        # for angle control
        self._omega_pid_controller = PIDController(10, 0, 20)
```

#### Videos

[![slam_urg](http://img.youtube.com/vi/LDpC2wqIoU4/hqdefault.jpg)](https://youtu.be/LDpC2wqIoU4)

[back to example list](#how-to-use-examples)

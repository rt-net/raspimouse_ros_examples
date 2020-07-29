[English](README.en.md) | [日本語](README.md)

# raspimouse_ros_examples

[![industrial_ci](https://github.com/rt-net/raspimouse_ros_examples/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse_ros_examples/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

Raspberry Pi MouseのROSサンプルコード集です。

ROS 2のサンプルコード集は[こちら](https://github.com/rt-net/raspimouse_ros2_examples)。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/raspberry_pi_mouse.JPG width=500>

## Requirements

- Raspberry Pi Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - Linux OS
    - Ubuntu server 16.04
    - Ubuntu server 18.04
    - https://wiki.ubuntu.com/ARM/RaspberryPi
  - Device Driver
    - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
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
# For direction control example
git clone https://github.com/rt-net/rt_usb_9axisimu_driver

# Install dependencies
rosdep install -r -y --from-paths . --ignore-src      

# make & install
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

## License

このリポジトリはApache 2.0ライセンスの元、公開されています。 
ライセンスについては[LICENSE](./LICENSE)を参照ください。

## How To Use Examples

- [keyboard_control](#keyboard_control)
- [joystick_control](#joystick_control)
- [object_tracking](#object_tracking)
- [line_follower](#line_follower)
- [SLAM](#SLAM)
- [direction_control](#direction_control)

---

### keyboard_control

[teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)を使ってRaspberryPiMouseを動かします。

#### Requirements 

- Keyboard

#### How to use

次のコマンドでノードを起動します。

```sh
roslaunch raspimouse_ros_examples teleop.launch key:=true

# Control from remote computer
roslaunch raspimouse_ros_examples teleop.launch key:=true mouse:=false
```

ノードが起動したら`/motor_on`サービスをコールします。

```sh
rosservice call /motor_on
```
[back to example list](#how-to-use-examples)

---

### joystick_control

ジョイスティックコントローラでRaspberryPiMouseを動かすコード例です。

#### Requirements 

- Joystick Controller
  - [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
  - [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)

#### How to use

次のコマンドでノードを起動します。

```sh
roslaunch raspimouse_ros_examples teleop.launch joy:=true

# Use DUALSHOCK 3
roslaunch raspimouse_ros_examples teleop.launch joy:=true joyconfig:="dualshock3" 

# Control from remote computer
roslaunch raspimouse_ros_examples teleop.launch joy:=true mouse:=false
```

デフォルトのキー割り当てはこちらです。

![joystick_control_keyconfig](https://rt-net.github.io/images/raspberry-pi-mouse/joystick_control_keyconfig.png)

#### Configure

[./config/joy_f710.yml](./config/joy_f710.yml)、[./config/joy_dualshock3.yml](./config/joy_dualshock3.yml)
のキー番号を編集することで、キー割り当てを変更できます。

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

色情報をもとにオレンジ色のボールの追跡を行うコード例です。
USB接続のWebカメラとOpenCVを使ってボール追跡をします。

#### Requirements 

- Webカメラ
  - [Logicool HD WEBCAM C310N](https://www.logicool.co.jp/ja-jp/product/hd-webcam-c310n)
- カメラマウント
  - [Raspberry Pi Mouse オプションキット No.4 \[Webカメラマウント\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584)
- ボール（Optional）
  - [ソフトボール（オレンジ）](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)
- Software
  - python
    - opencv
    - numpy
  - v4l-utils

#### Installation

Raspberry Pi Mouseにカメラマウントを取り付け，WebカメラをRaspberry Piに接続します．

次のコマンドで、カメラ制御用のパッケージ（v4l-utils）をインストールします。

```sh
sudo apt install v4l-utils
```
#### How to use

次のスクリプトを実行して、カメラの自動調節機能（自動露光，オートホワイトバランス等）を切ります。

```sh
rosrun raspimouse_ros_examples camera.bash
```

次のコマンドでノードを起動します。

```sh
roslaunch raspimouse_ros_examples object_tracking.launch
```

\[Raspberry PiのローカルIPアドレス\]:8080でカメラから取得した画像や処理結果の確認が出来ます．

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/web_video_server.png width=500 />

#### Configure

追跡対象の色を変更するには
[`./scripts/object_tracking.py`](./scripts/object_tracking.py)を編集します。

```python
    def detect_ball(self):
        # ~~~ 省略 ~~~
        min_hsv, max_hsv = self.set_color_orange()
        # min_hsv, max_hsv = self.set_color_green()
        # min_hsv, max_hsv = self.set_color_blue()
```

反応が悪い時にはカメラの露光や関数内のパラメータを調整して下さい．

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

ライントレースのコード例です。

#### Requirements

- ライントレースセンサ
  - [Raspberry Pi Mouse オプションキット No.3 \[ライントレース\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3591)
- フィールドとライン (Optional)

#### Installation

Raspberry Pi Mouseにライントレースセンサを取り付けます。


#### How to use

次のコマンドでノードを起動します。

```sh
roslaunch raspimouse_ros_examples line_follower.launch

# Control from remote computer
roslaunch raspimouse_ros_examples line_follower.launch mouse:=false
```

Raspberry Pi Mouseをフィールドに置き、SW2を押してフィールド上のセンサ値をサンプリングします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/field_calibration.JPG width=500 />

次に、センサとラインが重なるようにRaspberry Pi Mouseを置き、SW1を押してライン上のセンサ値をサンプリングします。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/line_calibration.JPG width=500 />

最後に、ライン上にRaspberry Pi Mouseを置き、SW0を押してライントレースを開始します。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/start_trace.JPG width=500 />

もう一度SW0を押すとライントレースを停止します。

#### Configure

走行速度を変更するには[`./scripts/line_follower.py`](./scripts/line_follower.py)を編集します。

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

LiDARを使ってSLAM（自己位置推定と地図作成）を行うサンプルです。

#### Requirements 

- LiDAR
  - [URG-04LX-UG01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1296&products_id=2816)
  - [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)
  - [LDS-01](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_5&products_id=3676)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)
- Joystick Controller (Optional)
  
#### Installation

Raspberry Pi MouseにLiDARを取り付けます。

- URG-04LX-UG01
  - <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_urg.JPG width=500 />
- RPLIDAR A1
  - <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_rplidar.png width=500 />
- LDS-01
  - <img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_lds01.JPG width=500 />
  
#### How to use

Raspberry Pi Mouse上で次のコマンドでノードを起動します。

```sh
# URG
roslaunch raspimouse_ros_examples mouse_with_lidar.launch urg:=true port:=/dev/ttyACM0

# RPLIDAR
roslaunch raspimouse_ros_examples mouse_with_lidar.launch rplidar:=true port:=/dev/ttyUSB0

# LDS
roslaunch raspimouse_ros_examples mouse_with_lidar.launch lds:=true port:=/dev/ttyUSB0

```

Raspberry Pi Mouseを動かすため`teleop.launch`を起動します

```sh
# joystick control
roslaunch raspimouse_ros_examples teleop.launch mouse:=false joy:=true joyconfig:=dualshock3
```

次のコマンドでSLAMパッケージを起動します。（Remote computerでの実行推奨）

```sh
# URG
roslaunch raspimouse_ros_examples slam_gmapping.launch urg:=true

# RPLIDAR
roslaunch raspimouse_ros_examples slam_gmapping.launch rplidar:=true

# LDS
roslaunch raspimouse_ros_examples slam_gmapping.launch lds:=true
```

Raspberry Pi Mouseを動かして地図を作成します。

次のコマンドで作成した地図を保存します。

```sh
mkdir ~/maps
rosrun map_server map_saver -f ~/maps/mymap
```

#### Configure

[./launch/slam_gmapping.launch](./launch/slam_gmapping.launch)で[gmapping](http://wiki.ros.org/gmapping)パッケージのパラメータを調整します。

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

IMUセンサを使用した角度制御のコード例です。

#### Requirements

- [USB出力9軸IMUセンサモジュール](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3416&language=ja)
- [LiDAR Mount](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3867)
- RT-USB-9axisIMU ROS Package.
  - https://github.com/rt-net/rt_usb_9axisimu_driver

#### Installation

LiDAR MountにIMUセンサモジュールを取り付けます。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_2.JPG width=500 />

Raspberry Pi Mouse にLiDAR Mountを取り付けます。

<img src=https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_imu_1.JPG width=500 />

#### How to use

次のコマンドでノードを起動します。

```sh
roslaunch raspimouse_ros_examples direction_control.launch
```

SW0 ~ SW2を押して動作モードを切り替えます。

- SW0: ジャイロセンサのバイアスをキャリブレーションし、ラズパイマウスの方位角を`0 rad`にリセットします
- SW1: 方位角を`0 rad`に維持する角度制御を開始します
  - SW0 ~ SW2を押すか、ラズパイマウス本体を横に傾けると終了します
- SW2: 方位角を`-π ~ π rad`に変化させる角度制御を開始します
  - SW0 ~ SW2を押すか、ラズパイマウス本体を横に傾けると終了します

#### Configure

角度制御に使うPID制御器のゲインを変更するには[`./scripts/direction_control.py`](./scripts/direction_control.py)を編集します。

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

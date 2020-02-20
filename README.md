[English](README.en.md) | [日本語](README.md)

# raspimouse_ros_examples

[![Build Status](https://travis-ci.com/rt-net/raspimouse_ros_exapmles.svg?token=44UfTwwGaAupMGxC2ZWA&branch=master)](https://travis-ci.com/rt-net/raspimouse_ros_exapmles)

Raspberry Pi MouseのROSサンプルコード集です。

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

このリポジトリはApache 2.0ライセンスの元、公開されています。 
ライセンスについては[LICENSE](./LICENSE)を参照ください。

## How To Use Examples

- [keyboard_control](#keyboard_control)
- [joystick_control](#joystick_control)
- [object_tracking](#object_tracking)

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

![joystick_control_keyconfig](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/joystick_control_keyconfig.png)

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
[back to example list](#how-to-use-examples)

--- 

### object_tracking

![object_tracking](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/object_tracking.JPG)

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
roslaunch raspimouse_ros_examples pimouse_object_tracking.launch
```

\[Raspberry PiのローカルIPアドレス\]:8080でカメラから取得した画像や処理結果の確認が出来ます．

![web_video_server](https://github.com/rt-net/raspimouse_ros_exapmles/blob/images/web_video_server.png)

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
[back to example list](#how-to-use-examples)

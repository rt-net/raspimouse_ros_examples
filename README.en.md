[English](README.en.md) | [日本語](README.md)

# raspimouse_ros_exapmles

# License

This repository is published under the Apache 2.0 license. 

Please refer to [LICENSE](./LICENSE) for license.

### object_tracking.py

This is a example to track an orange ball.

Use OpenCV with USB connected web camera.

 install the OpenCV library of Python with the following command:
```sh
sudo apt-get install ros-melodic-cv-camera
sudo apt-get install python-opencv
```

Use a camera control package (v4l-utils) and a shell scriptto turn off the automatic adjustment function (auto exposure, auto white balance, etc.) of the camera.

```sh
sudo apt-get install v4l-utils
./scripts/camera.bash
```


Start the node using roslaunch.
```sh
roslaunch raspimouse_ros_examples pimouse_object_tracking.launch
```

You can check the image acquired from the camera with :
[Raspberry Pi local IP address]:8080

*For ball tracking*

Edit [`./scripts/object_tracking.py`](./scripts/object_tracking.py) as follows:

```python
    def detect_ball(self):
    # ...
    # Extract orange(use HSV color model)
        hsv = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)
        min_hsv_orange = np.array([15, 150, 40])
        max_hsv_orange = np.array([20, 255, 255])
        binary = cv2.inRange(hsv, min_hsv_orange, max_hsv_orange)
```

The values of min_hsv_orange and max_hsv_orange represent the parameters of HSV color space.

Please change the value of H (hue) according to the color of the ball.

  - This orange ball can be purchased from
[this page](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701&language=en)
in RT ROBOT SHOP.

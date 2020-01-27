# pimouse_object_tracking

### object_tracking.pyの実行

オレンジ色のボールの追跡を行うコード例です。
市販のWebカメラとOpenCVを使ってボール追跡をします。

次のコマンドでOpenCVのPythonライブラリをインストールしてください。
```sh
sudo apt-get install ros-melodic-cv-camera
sudo apt-get install python-opencv
```

次のコマンドでノードを起動します。
```sh
rosrun cv_camera cv_camera_node
rosrun pimouse_ros motors.py
rosrun raspimouse_ros_examples object_tracking.py
```

ボールの検出結果を確認したい場合，web_videeo_serverの追加が必要です．
```sh
sudo apt-get install ros-melodic-web-video-server 
rosrun web_video_server web_video_server
```
web_video_serverのデフォルトのポート番号は8080です．
[Raspberry PiのローカルIPアドレス]:8080で接続できます．

*ボール追跡をする場合*

[`./scripts/object_tracking.py`](./scripts/object_tracking.py)を編集します。

```python
    def detect_ball(self):<
    # ~~~ 省略 ~~~
    # HSV色空間を用いてオレンジ色を抽出<
        hsv = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)<
        min_hsv_orange = np.array([15, 150, 40])<
        max_hsv_orange = np.array([20, 255, 255])<
        binary = cv2.inRange(hsv, min_hsv_orange, max_hsv_orange)<
```

min_hsv_orange，及びmax_hsv_orangeの値はHSV色空間の[H（色相）, S（彩度）, V（明度）]を表します．

他の色のボールで追跡を行いたい場合はH（色相）の値を変更して下さい．

但しOpenCVでH（色相）は0~180の値を取る事に注意が必要です．

  - ボールは、アールティショップの
[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)
で購入できます。

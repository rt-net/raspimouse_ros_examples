[English](README.en.md) | [日本語](README.md)

# raspimouse_ros_exapmles

# License

このリポジトリはApache 2.0ライセンスの元、公開されています。 
ライセンスについては[LICENSE](./LICENSE)を参照ください。

### object_tracking.pyの実行

オレンジ色のボールの追跡を行うコード例です。

USB接続のWebカメラとOpenCVを使ってボール追跡をします。

WebカメラをRaspberry Piに接続して，Raspberry Pi Mouseの正面に向けて下さい．

次のコマンドでPython用のOpenCVライブラリをインストールして下さい。
```sh
sudo apt-get install ros-melodic-cv-camera
sudo apt-get install python-opencv
```

カメラの自動調節機能（自動露光，オートホワイトバランス等）を切るため，
カメラ制御用のパッケージ（v4l-utils）とシェルスクリプトを用います．

```sh
sudo apt-get install v4l-utils
./scripts/camera.bash
```


roslaunchを用いてノードを起動します。
```sh
roslaunch raspimouse_ros_examples pimouse_object_tracking.launch
```

[Raspberry PiのローカルIPアドレス]:8080でカメラから取得した画像や処理結果の確認が出来ます．

*ボール追跡をする場合*

[`./scripts/object_tracking.py`](./scripts/object_tracking.py)を編集します。

```python
    def detect_ball(self):<
    # ~~~ 省略 ~~~
    # Extract orange(use HSV color model)~
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

[English](README.en.md) | [日本語](README.md)

# raspimouse_ros_exapmles

# License

このリポジトリはApache 2.0ライセンスの元、公開されています。 
ライセンスについては[LICENSE](./LICENSE)を参照ください。

### object_tracking.pyの実行

色情報をもとに，オレンジ色のボールの追跡を行うコード例です。

USB接続のWebカメラとOpenCVを使ってボール追跡をします。
* Webカメラ
    * Logicool HD ウェブカメラ C310
* カメラマウント
    * [Raspberry Pi Mouse オプションキット No.4 \[Webカメラマウント\]](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3584)
* ターゲットとなるボール
    * [ソフトボール（オレンジ）](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)

ROS以外に以下のパッケージを使用します． 
* python
    * opencv
    * numpy
* v4l-utils

Raspberry Pi Mouseにカメラマウントを取り付け，WebカメラをRaspberry Piに接続します．

カメラの自動調節機能（自動露光，オートホワイトバランス等）を切るため，
カメラ制御用のパッケージ（v4l-utils）とシェルスクリプトを用います．

```sh
sudo apt install v4l-utils
./scripts/camera.bash
```


roslaunchを用いてノードを起動します。
```sh
roslaunch raspimouse_ros_examples pimouse_object_tracking.launch
```

[Raspberry PiのローカルIPアドレス]:8080でカメラから取得した画像や処理結果の確認が出来ます．

*追跡対象とする色の変更方法*

[`./scripts/object_tracking.py`](./scripts/object_tracking.py)を編集します。

```python
    def detect_ball(self):
        # ~~~ 省略 ~~~
        min_hsv, max_hsv = self.set_color_orange()
        # min_hsv, max_hsv = self.set_color_green()
        # min_hsv, max_hsv = self.set_color_blue()
```

反応が悪い時にはカメラの露光や，関数内のパラメータを調整して下さい．

（例）暗い環境で実行する場合
```python
    def set_color_orange(self):
        # [H(0~180), S(0~255), V(0~255)]
        # min_hsv_orange = np.array([15, 200, 80])
        min_hsv_orange = np.array([15, 150, 40])
        max_hsv_orange = np.array([20, 255, 255])
        return min_hsv_orange, max_hsv_orange
```

# Lecture 2: rosbag2, rviz2, rqt, CvBridge, tf2

## 1. 準備

### 1.1 演習資料と演習パッケージのダウンロード

下記のリンクにアクセスして資料と演習パッケージ，rosbagをダウンロードしてください．

- 資料のダウンロード：[handout_ros2_lecture_02_ja.pdf](https://drive.google.com/uc?export=download&id=1bOPD9BjyKbc96vSSg3HIoQrV1PLRSOLE)
- 演習パッケージのダウンロード：[lecture02_pkg.tar.gz](https://drive.google.com/uc?export=download&id=1hrZ1uphaHEGMasrSg_FFadzFi0N21IQc)
- rosbag(`sample`)のダウンロード：[`sample.tar.gz (43.5MB)`](https://drive.google.com/uc?export=download&id=1PvYPmePJsXU9G6npDntkVbqH7sx7S-sy)
- rosbag(`tf2_5min`)のダウンロード：[`tf2_5min.tar.gz (93KB)`](https://drive.google.com/uc?export=download&id=1OPULoeFOekQHslPs2xkYte81x8hx8bUC)

### 1.2 README.mdとPythonスクリプトの移動

下記にしたがって，ダウンロードした演習パッケージを移動してください．

- ダウンロードした演習パッケージ(`lecture02_pkg.tar.gz`)を解凍してください．
- 解凍した演習パッケージ(`lecture02_pkg`)を`~/ros2_lecture_ws/src/7_lectures`に移動してください．

### 1.3 パッケージのビルド

下記のコマンドを実行してください．

- 端末1
  - 仮想環境の起動等(<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - パッケージをビルド

    ```Bash
    colcon build --symlink-install
    ```

### 1.4 rosbag2データの移動

ダウンロードした`sample.tar.xz`と`tf2_5min.tar.xz`を`ros2_lecture_ws`に移動して解凍してください．

## 2. rosbag2

### 2.1 rosbag2データの再生

- 端末1

  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rosbag2データ（sample）の再生

    ```Bash
    ros2 bag play -l sample
    ```

    `-l`：繰り返し再生

- 端末2

  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rviz2でトピックを確認

    ```Bash
    rviz2
    ```

- 端末3

  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rqtでの確認

    ```Bash
    rqt
    ```

## 3. CvBridge

### 3.1 サンプル

サブスクライブした画像をOpenCVで処理するサンプルです．\
GUI画面の左上に青い○が表示されればOKです．

#### 3.1.1 実行方法

- 端末1
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rosbag2データ(sample)の再生

    ```Bash
    ros2 bag play -l sample
    ```

- 端末2
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - opencv_sampleノードの実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture02_pkg opencv_sample
    ```

### 3.2 演習課題（CvBridge）

プログラムを改良して,画像をpublishしよう

- opencv_exercise1.pyの作成
  - 使用するメッセージ型「senser_msgs/Image」をインポート
  - Publisherの設定
    - Topic Name: /image_topic_2
    - Message Type: Image
    - Queue Size: 10
- Callback()関数内でpublishする処理を追加しよう
  - 以下の条件でcv_imageを変換しましょう
    - Image : cv_image
    - encode: “bgr8”
- setup.pyにノード情報を定義
- rviz2で確認しよう

#### 3.2.1 実行方法

- 端末1

  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rosbag2データ(sample)の再生

    ```Bash
    ros2 bag play -l sample
    ```

- 端末2
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - opencv_exercise1ノードの実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture02_pkg opencv_exercise1
    ```

- 端末3
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rviz2で確認

    ```Bash
    rviz2
    ```

## 4. tf2

### 4.1 rosbag2データの再生

- 端末1
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rosbag2データ（sample）の再生

    ```Bash
    ros2 bag play -l tf2_5min
    ```

    `-l`：繰り返し再生

- 端末2
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rviz2でトピックを確認

    ```Bash
    rviz2
    ```

- 端末3
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - rqtでの確認

    ```Bash
    rqt
    ```

### 4.2 サンプル

動的にTFをブロードキャストして，TFを取得する

#### 4.2.1 実行方法

- 端末1
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - tf2_broadcasterノードの実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture02_pkg tf2_broadcaster
    ```

- 端末2
  - 仮想環境の起動等(すでに<font color="Yellow">実行済みの場合はスキップ</font>)

    ``` Bash
    cd ~/ros2_lecture_ws
    ```

    ``` Bash
    . 0_env.sh
    ```

    ```Bash
    . /entrypoint.sh
    ```

  - tf2_listenerノードの実行

    ```Bash
    source install/setup.bash
    ```

    ```Bash
    ros2 run lecture02_pkg tf2_listener
    ```

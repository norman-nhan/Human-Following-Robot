#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: opencv_sample.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(外部)
import cv2  # OpenCVのモジュール

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageConverter(Node):
    """ImageConverterクラス（Nodeクラスを継承）
    サブスクライブしたImageメッセージをOpenCVのデータ形式に変換し画像処理を行うクラス

    Attributes:
        bridge (CvBridge): CvBridgeのインスタンス
        image_sub (Subscription): ImageメッセージのトピックをサブスクライブするためのROS2サブスクライバ
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("opencv_sample")

        # CvBridgeクラスのインスタンスを生成
        self.bridge = CvBridge()

        # サブスクライバの宣言
        self.image_sub = self.create_subscription(
            msg_type=Image, topic="image_raw", callback=self.callback, qos_profile=10
        )

    def callback(self, msg: Image):
        """サブスクライブしたメッセージを受信したときに呼び出されるコールバックメソッド

        Args:
            msg (Image): 受信したImageメッセージ
        """
        try:
            # ImageメッセージをOpenCVのデータに変換（同時に，色の順序をOpenCVの標準的な順序に変更）
            cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            # エラーログを表示
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        # == OpenCV による画像処理 (何をやっているか考えてみてください) ==
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, center=(50, 50), radius=10, thickness=3, color=(255, 0, 0))
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # ImageConverterクラスのインスタンス生成
    image_converter = ImageConverter()

    try:
        # ノードが終了するまで指定したノードを実行
        rclpy.spin(image_converter)
    except KeyboardInterrupt:
        print("Shutting down")

    # 終了処理
    image_converter.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

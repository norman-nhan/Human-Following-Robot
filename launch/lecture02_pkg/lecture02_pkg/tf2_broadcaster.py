#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: tf2_broadcast.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(標準)
from math import cos, sin, pi

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped


class TF2Broadcaster(Node):
    """TF2Broadcasterクラス（Nodeクラスを継承）
    動的tfを配信するノードクラス

    Attributes:
        br (TransformBroadcaster): TransformBroadcasterのインスタンス
        count (int): 処理回数をカウントする変数
        timer (Timer): 指定された周期でコールバックメソッドを呼び出すためのタイマ
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("tf2_broadcaster")

        # カウント変数の初期化
        self.count = 0

        # TransformBroadcasterクラスのインスタンスを生成
        self.br = TransformBroadcaster(self)

        # 割り込み用タイマの定義（指定した周期[s]でコールバックメソッドを呼ぶ）
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

    def timer_callback(self):
        """割り込み用タイマのコールバックメソッド"""
        # 座標の宣言
        x = cos(pi / 10 * self.count)
        y = sin(pi / 10 * self.count)
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # オイラー角をクォータニオンに変換
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # TransformStampedクラスのインスタンス生成
        t = TransformStamped()

        # TransformStampedメッセージに値を設定
        t.header.stamp = self.get_clock().now().to_msg()  # 現在時刻を設定
        t.header.frame_id = "link1"  # ヘッダフレームIDを設定
        t.child_frame_id = "link2"  # 子フレームIDを設定
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # TFを配信
        self.br.sendTransform(t)

        # ログを表示
        self.get_logger().info("Transform Published")

        # カウントアップ
        self.count += 1


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # TF2Broadcasterクラスのインスタンス生成
    node = TF2Broadcaster()

    try:
        # ノードが終了するまで指定したノードを実行
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: tf2_listener.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# モジュールのインポート(ROS2関連)
import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration
import tf2_ros
from tf2_ros import TransformListener
from tf2_ros import TransformException  # TF2の例外処理
import tf_transformations


class TF2Listener(Node):
    """TF2Listenerクラス（Nodeクラスを継承）
    tfを取得するノードクラス

    Attributes:
        tf_buffer (tf2_ros.Buffer): tf2_rosのBufferクラスのインスタンス
        tf_listener (TransformListener): TransformListenerクラスのインスタンス
        timer (Timer): 指定された周期でコールバックメソッドを呼び出すためのタイマ
    """

    def __init__(self):
        """クラスの初期化メソッド"""
        # 継承したNodeクラスのコンストラクタをオーバーライド(引数は，'ノード名')
        super().__init__("tf2_listener")

        # tf2_ros.Bufferクラスのインスタンス生成
        self.tf_buffer = tf2_ros.Buffer()

        # TransformListenerクラスのインスタンス生成
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)

        # 割り込み用タイマの定義（指定した周期[s]でコールバックメソッドを呼ぶ）
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

    def timer_callback(self):
        """割り込み用タイマのコールバックメソッド"""
        try:
            # ソースフレームから見たターゲットフレームの相対座標を取得
            # フレームはターゲット→ソースの順に記述する
            # 引数timeはtfを取得する時間．rclpy.time.Time()を設定すると最新のtfを取得する.
            t = self.tf_buffer.lookup_transform(
                target_frame="link1", source_frame="link2", time=rclpy.time.Time()
            )
        except TransformException:
            return

        # 平行移動要素を取得
        translation = t.transform.translation

        # 回転要素を取得
        rotation = t.transform.rotation

        # quaternionをリスト形式に変換
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]

        # クォータニオンをオイラー角に変換
        euler = tf_transformations.euler_from_quaternion(quaternion)

        # ログの表示
        self.get_logger().info("-----------")
        self.get_logger().info(
            f"Translation: x={translation.x}, y={translation.y}, z={translation.z}"
        )  # x, y, zの表示
        self.get_logger().info(
            f"Rotation: roll={euler[0]}, pitch={euler[1]}, yaw={euler[2]}"
        )  # roll, pitch, yawの表示


def main(args=None):
    """Main関数"""
    # ROS2のPythonクライアントライブラリの初期化
    rclpy.init(args=args)

    # TF2Listenerクラスのインスタンス生成
    node = TF2Listener()

    # ノードが終了するまで指定したノードを実行
    rclpy.spin(node)

    # 終了処理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

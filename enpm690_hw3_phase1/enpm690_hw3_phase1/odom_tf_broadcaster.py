#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    """Broadcasts the odom -> base_footprint transform from /odom."""

    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("default_child_frame", "base_footprint")

        self._parent_frame = str(self.get_parameter("parent_frame").value)
        self._default_child_frame = str(self.get_parameter("default_child_frame").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._last_stamp_ns = -1

        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 20)
        self.get_logger().info("bridging %s into /tf" % odom_topic)

    def _odom_callback(self, msg: Odometry) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns <= self._last_stamp_ns:
            return
        self._last_stamp_ns = stamp_ns

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id or self._parent_frame
        transform.child_frame_id = msg.child_frame_id or self._default_child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

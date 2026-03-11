#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopCommandLogger(Node):
    """Prints readable teleop command semantics from /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("teleop_command_logger")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("linear_deadband", 0.01)
        self.declare_parameter("angular_deadband", 0.05)

        self._linear_deadband = float(self.get_parameter("linear_deadband").value)
        self._angular_deadband = float(self.get_parameter("angular_deadband").value)
        self._last_signature: tuple[str, float, float] | None = None

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.create_subscription(Twist, cmd_topic, self._cmd_callback, 10)
        self.get_logger().info("mode started: teleop command logger listening on %s" % cmd_topic)

    def _cmd_callback(self, msg: Twist) -> None:
        label = self._interpret_command(msg)
        linear = round(msg.linear.x, 2)
        angular = round(msg.angular.z, 2)
        signature = (label, linear, angular)

        if self._last_signature == signature:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"[{timestamp:7.2f}s] {label} | linear.x={linear:+.2f} m/s | angular.z={angular:+.2f} rad/s"
        )
        self._last_signature = signature

    def _interpret_command(self, msg: Twist) -> str:
        linear = msg.linear.x
        angular = msg.angular.z

        linear_state = "idle"
        if linear > self._linear_deadband:
            linear_state = "forward"
        elif linear < -self._linear_deadband:
            linear_state = "backward"

        angular_state = "straight"
        if angular > self._angular_deadband:
            angular_state = "turn left"
        elif angular < -self._angular_deadband:
            angular_state = "turn right"

        if linear_state == "idle" and angular_state == "straight":
            return "stop"
        if linear_state != "idle" and angular_state == "straight":
            return linear_state
        if linear_state == "idle" and angular_state != "straight":
            return angular_state
        return f"{linear_state} + {angular_state}"


def main() -> None:
    rclpy.init()
    node = TeleopCommandLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP_TEXT = """
ENPM690 Phase 1 keyboard teleop
-------------------------------
w: forward
x: backward
a: rotate left
d: rotate right
s or space: stop
q/z: increase/decrease linear speed
e/c: increase/decrease angular speed

Ctrl-C to exit
"""


class KeyboardTeleop(Node):
    """Simple keyboard teleop publisher for /cmd_vel."""

    def __init__(self, settings: list[int]) -> None:
        super().__init__("keyboard_teleop")
        self._settings = settings

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 1.00)
        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("angular_step", 0.10)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._cmd_topic = str(self.get_parameter("cmd_topic").value)
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._angular_speed = float(self.get_parameter("angular_speed").value)
        self._linear_step = float(self.get_parameter("linear_step").value)
        self._angular_step = float(self.get_parameter("angular_step").value)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._publisher = self.create_publisher(Twist, self._cmd_topic, 10)
        self._current_twist = Twist()

        self.create_timer(1.0 / publish_rate_hz, self._publish_current_command)
        self.create_timer(0.05, self._poll_keyboard)

        self.get_logger().info("publishing keyboard teleop on %s" % self._cmd_topic)
        self.get_logger().info(HELP_TEXT)
        self._log_speed_state()

    def _publish_current_command(self) -> None:
        self._publisher.publish(self._current_twist)

    def _poll_keyboard(self) -> None:
        if not self._key_available():
            return

        key = sys.stdin.read(1)
        if key == "\x03":
            raise KeyboardInterrupt

        if key == "w":
            self._set_motion(self._linear_speed, 0.0)
        elif key == "x":
            self._set_motion(-self._linear_speed, 0.0)
        elif key == "a":
            self._set_motion(0.0, self._angular_speed)
        elif key == "d":
            self._set_motion(0.0, -self._angular_speed)
        elif key in {"s", " "}:
            self._set_motion(0.0, 0.0)
        elif key == "q":
            self._linear_speed += self._linear_step
            self._log_speed_state()
        elif key == "z":
            self._linear_speed = max(self._linear_step, self._linear_speed - self._linear_step)
            self._log_speed_state()
        elif key == "e":
            self._angular_speed += self._angular_step
            self._log_speed_state()
        elif key == "c":
            self._angular_speed = max(self._angular_step, self._angular_speed - self._angular_step)
            self._log_speed_state()

    def _key_available(self) -> bool:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        return bool(readable)

    def _set_motion(self, linear_x: float, angular_z: float) -> None:
        self._current_twist.linear.x = linear_x
        self._current_twist.angular.z = angular_z
        self._publisher.publish(self._current_twist)

    def _log_speed_state(self) -> None:
        self.get_logger().info(
            f"speed settings | linear={self._linear_speed:.2f} m/s | angular={self._angular_speed:.2f} rad/s"
        )

    def stop(self) -> None:
        self._current_twist = Twist()
        self._publisher.publish(self._current_twist)


def main() -> None:
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init()
    node = KeyboardTeleop(settings)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard teleop exiting")
    finally:
        node.stop()
        node.destroy_node()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == "__main__":
    main()

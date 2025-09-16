#!/usr/bin/env python3
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class MotorFLKeyboard(Node):
    """
    Key controls:
      f -> forward
      b -> backward
      s -> stop
      q -> quit
    Publishes std_msgs/Int8 on /fl_cmd
    """

    def __init__(self):
        super().__init__('motor_fl_keyboard')
        self.pub = self.create_publisher(Int8, 'fl_cmd', 10)
        self.get_logger().info("Keys: [w]=forward  [s]=backward  [a]=stop  [q]=quit")

        # terminal raw mode
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

        # poll 10 Hz
        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == 'w':
                self.pub.publish(Int8(data=1))
                self.get_logger().info("Sent: FORWARD")
            elif ch == 's':
                self.pub.publish(Int8(data=-1))
                self.get_logger().info("Sent: BACKWARD")
            elif ch == 'a':
                self.pub.publish(Int8(data=0))
                self.get_logger().info("Sent: STOP")
            elif ch == 'q':
                self.get_logger().info("Quitting…")
                rclpy.shutdown()

    def destroy_node(self):
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    n = MotorFLKeyboard()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

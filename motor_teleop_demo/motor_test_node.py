#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import RPi.GPIO as GPIO  # provided by python3-rpi-lgpio

class MotorFLNode(Node):
    """
    Subscribes: /fl_cmd (std_msgs/Int8)
      1  -> forward (slow, fixed duty)
     -1  -> backward (slow, fixed duty)
      0  -> stop
    Uses RPi.GPIO software PWM for a simple bench test.
    """

    def __init__(self):
        super().__init__('motor_fl_node')

        # ----- Parameters -----
        self.declare_parameter('dir_pin', 16)       # BCM16
        self.declare_parameter('pwm_pin', 14)       # BCM14
        self.declare_parameter('pwm_hz', 200)       # test frequency
        self.declare_parameter('test_duty', 30.0)   # percent when ON
        self.declare_parameter('forward_high', True)  # True: HIGH = forward

        self.dir_pin     = int(self.get_parameter('dir_pin').value)
        self.pwm_pin     = int(self.get_parameter('pwm_pin').value)
        self.pwm_hz      = int(self.get_parameter('pwm_hz').value)
        self.test_duty   = float(self.get_parameter('test_duty').value)
        self.forward_high= bool(self.get_parameter('forward_high').value)

        # ----- GPIO init -----
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pwm_pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_hz)
        self.pwm.start(0.0)  # start stopped

        # ----- ROS -----
        self.sub = self.create_subscription(Int8, 'fl_cmd', self.on_cmd, 10)
        self.get_logger().info(
            f"Front-left motor ready on DIR=BCM{self.dir_pin}, PWM=BCM{self.pwm_pin} "
            f"({self.pwm_hz} Hz, {self.test_duty:.1f}% duty when commanded)"
        )

    def on_cmd(self, msg: Int8):
        v = int(msg.data)
        if v == 1:
            # Forward
            GPIO.output(self.dir_pin, GPIO.HIGH if self.forward_high else GPIO.LOW)
            self.pwm.ChangeDutyCycle(self.test_duty)
            self.get_logger().info("FL: FORWARD")
        elif v == -1:
            # Backward
            GPIO.output(self.dir_pin, GPIO.LOW if self.forward_high else GPIO.HIGH)
            self.pwm.ChangeDutyCycle(self.test_duty)
            self.get_logger().info("FL: BACKWARD")
        else:
            # Stop
            self.pwm.ChangeDutyCycle(0.0)
            self.get_logger().info("FL: STOP")

    def destroy_node(self):
        try:
            self.pwm.ChangeDutyCycle(0.0)
            self.pwm.stop()
            GPIO.output(self.dir_pin, GPIO.LOW)
            GPIO.cleanup([self.dir_pin, self.pwm_pin])
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    n = MotorFLNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

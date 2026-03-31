#!/usr/bin/env python3
import time

import pigpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')

        # --- Parameters ---
        self.declare_parameter('gpio_mode', 10)  # CH6 input - pin 19
        self.declare_parameter('mode_threshold_us', 1500)

        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('publish_hz', 20.0)

        self.declare_parameter('topic_rc', '/cmd_vel_rc')
        self.declare_parameter('topic_auto', '/cmd_vel_auto')
        self.declare_parameter('topic_out', '/cmd_vel')

        # Read params
        self.gpio_mode = int(self.get_parameter('gpio_mode').value)
        self.mode_threshold = int(self.get_parameter('mode_threshold_us').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        topic_rc = str(self.get_parameter('topic_rc').value)
        topic_auto = str(self.get_parameter('topic_auto').value)
        topic_out = str(self.get_parameter('topic_out').value)

        publish_hz = float(self.get_parameter('publish_hz').value)

        # --- pigpio setup ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio not running. Start: sudo systemctl start pigpiod")

        self.pi.set_mode(self.gpio_mode, pigpio.INPUT)

        # --- Mode PWM ---
        self._mode_us = None
        self._mode_rise_tick = None

        self.cb_mode = self.pi.callback(self.gpio_mode, pigpio.EITHER_EDGE, self._cb_mode)

        # --- Subscribers ---
        self.sub_rc = self.create_subscription(Twist, topic_rc, self._cb_rc, 10)
        self.sub_auto = self.create_subscription(Twist, topic_auto, self._cb_auto, 10)

        # --- Publisher ---
        self.pub = self.create_publisher(Twist, topic_out, 10)

        # Latest commands
        self._rc_cmd = Twist()
        self._auto_cmd = Twist()

        self._rc_time = time.time()
        self._auto_time = time.time()

        self._last_mode = None

        # Timer
        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info("cmd_vel_mux_node started")

    # --- Callbacks ---

    def _cb_mode(self, gpio, level, tick):
        if level == 1:
            self._mode_rise_tick = tick
        elif level == 0 and self._mode_rise_tick is not None:
            self._mode_us = pigpio.tickDiff(self._mode_rise_tick, tick)

    def _cb_rc(self, msg):
        self._rc_cmd = msg
        self._rc_time = time.time()

    def _cb_auto(self, msg):
        self._auto_cmd = msg
        self._auto_time = time.time()

    # --- Main loop ---

    def _tick(self):
        now = time.time()

        # If no mode signal yet → safe stop
        if self._mode_us is None:
            self.pub.publish(Twist())
            return

        # Determine mode
        rc_mode = self._mode_us > self.mode_threshold

        # Log only on change
        if rc_mode != self._last_mode:
            if rc_mode:
                self.get_logger().info("🟢 MODE: RC CONTROL")
            else:
                self.get_logger().info("🔵 MODE: AUTONOMOUS")
            self._last_mode = rc_mode

        # Select command source
        if rc_mode:
            # RC mode
            if (now - self._rc_time) > self.timeout_s:
                self.pub.publish(Twist())
            else:
                self.pub.publish(self._rc_cmd)
        else:
            # Autonomous mode
            if (now - self._auto_time) > self.timeout_s:
                self.pub.publish(Twist())
            else:
                self.pub.publish(self._auto_cmd)

    def destroy_node(self):
        try:
            self.cb_mode.cancel()
            self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
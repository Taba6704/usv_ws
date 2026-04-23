#!/usr/bin/env python3
import time

import pigpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


def map_pwm_to_unit(pw_us: int, neutral: int, lo: int, hi: int, deadband_us: int = 50) -> float:
    """
    Map RC PWM pulse width (microseconds) to [-1, 1] with neutral deadband.
    Piecewise linear scaling using calibration points:
      lo -> -1, neutral -> 0, hi -> +1
    """
    if pw_us is None:
        return 0.0

    if abs(pw_us - neutral) <= deadband_us:
        return 0.0

    if pw_us > neutral:
        denom = max(1, hi - neutral)
        x = (pw_us - neutral) / denom
        return clamp(x, 0.0, 1.0)

    denom = max(1, neutral - lo)
    x = (neutral - pw_us) / denom
    return -clamp(x, 0.0, 1.0)


class RcToCmdVel(Node):
    def __init__(self):
        super().__init__('rc_controller_node')

        # --- GPIO pins (BCM) ---
        self.declare_parameter('gpio_throttle', 17) # CH3 - pin 11
        self.declare_parameter('gpio_yaw', 27)      # CH4 - pin 13
        self.declare_parameter('e_stop_in', 18)     # CH5 - pin 12
        self.declare_parameter('relay_out', 23)     # relay control pin 16

        # --- Calibration ---
        self.declare_parameter('thr_lo', 1010)
        self.declare_parameter('thr_neutral', 1465)
        self.declare_parameter('thr_hi', 1980)

        self.declare_parameter('yaw_lo', 1000)
        self.declare_parameter('yaw_neutral', 1490)
        self.declare_parameter('yaw_hi', 1970)

        # --- Mapping / timing ---
        self.declare_parameter('deadband_us', 50)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('timeout_s', 0.35)
        self.declare_parameter('cmd_topic', '/cmd_vel_rc')

        # E-Stop controll
        self.declare_parameter('estop_threshold_us', 1500)
        self.declare_parameter('relay_active_high', True)  # set False if relay is active-low

        # Read Parameters
        self.gpio_thr = int(self.get_parameter('gpio_throttle').value)
        self.gpio_yaw = int(self.get_parameter('gpio_yaw').value)
        self.e_stop_in = int(self.get_parameter('e_stop_in').value)
        self.relay_out = int(self.get_parameter('relay_out').value)

        self.thr_lo = int(self.get_parameter('thr_lo').value)
        self.thr_neu = int(self.get_parameter('thr_neutral').value)
        self.thr_hi = int(self.get_parameter('thr_hi').value)

        self.yaw_lo = int(self.get_parameter('yaw_lo').value)
        self.yaw_neu = int(self.get_parameter('yaw_neutral').value)
        self.yaw_hi = int(self.get_parameter('yaw_hi').value)

        self.deadband = int(self.get_parameter('deadband_us').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        publish_hz = float(self.get_parameter('publish_hz').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.estop_threshold_us = int(self.get_parameter('estop_threshold_us').value)
        self.relay_active_high = bool(self.get_parameter('relay_active_high').value)

        # Publisher
        self.pub = self.create_publisher(Twist, cmd_topic, 10)

        # pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Start: sudo systemctl start pigpiod")

        self.pi.set_mode(self.gpio_thr, pigpio.INPUT)
        self.pi.set_mode(self.gpio_yaw, pigpio.INPUT)
        self.pi.set_mode(self.e_stop_in, pigpio.INPUT)
        self.pi.set_mode(self.relay_out, pigpio.OUTPUT)

        # Initialize E-Stop relay to safe
        self._estop_us = None
        self._estop_rise_tick = None

        # Start in e-stop/safe state until valid switch input is received
        self._set_relay(False)

        # Track previous state for logging
        self._last_estop_state = None

        # Latest pulse widths
        self._thr_us = None
        self._yaw_us = None

        # Edge timestamps
        self._thr_rise_tick = None
        self._yaw_rise_tick = None

        # For failsafe
        self._last_update_time = time.time()

        # Register callbacks
        self.cb_thr = self.pi.callback(self.gpio_thr, pigpio.EITHER_EDGE, self._cb_thr)
        self.cb_yaw = self.pi.callback(self.gpio_yaw, pigpio.EITHER_EDGE, self._cb_yaw)
        self.cb_estop = self.pi.callback(self.e_stop_in, pigpio.EITHER_EDGE, self._cb_estop)

        # Timer to publish
        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            f"RC->cmd_vel publishing to {cmd_topic} at {publish_hz} Hz "
            f"(thr GPIO {self.gpio_thr}, yaw GPIO {self.gpio_yaw})"
        )

    def _set_relay(self, enabled: bool):
        if self.relay_active_high:
            level = 1 if enabled else 0
        else:
            level = 0 if enabled else 1
        self.pi.write(self.relay_out, level)

    def _cb_estop(self, gpio, level, tick):
        if level == 1:
            self._estop_rise_tick = tick
        elif level == 0 and self._estop_rise_tick is not None:
            self._estop_us = pigpio.tickDiff(self._estop_rise_tick, tick)
            self._last_update_time = time.time()

    def _cb_thr(self, gpio, level, tick):
        if level == 1:
            self._thr_rise_tick = tick
        elif level == 0 and self._thr_rise_tick is not None:
            self._thr_us = pigpio.tickDiff(self._thr_rise_tick, tick)
            self._last_update_time = time.time()

    def _cb_yaw(self, gpio, level, tick):
        if level == 1:
            self._yaw_rise_tick = tick
        elif level == 0 and self._yaw_rise_tick is not None:
            self._yaw_us = pigpio.tickDiff(self._yaw_rise_tick, tick)
            self._last_update_time = time.time()

    def _tick(self):
        now = time.time()

        # Failsafe: if signal stops, command 0
        if (now - self._last_update_time) > self.timeout_s:
            self._set_relay(False)
            self._publish_stop()
            return

        # If we haven't received pulses yet
        if self._thr_us is None or self._yaw_us is None or self._estop_us is None:
            self._set_relay(False)
            self._publish_stop()
            return

        linear = map_pwm_to_unit(self._thr_us, self.thr_neu, self.thr_lo, self.thr_hi, self.deadband)
        angular = -map_pwm_to_unit(self._yaw_us, self.yaw_neu, self.yaw_lo, self.yaw_hi, self.deadband)
        estop_enabled = self._estop_us > self.estop_threshold_us

        # E-stop logic
        estop_enabled = self._estop_us > self.estop_threshold_us

        # Log ONLY when state changes
        if estop_enabled != self._last_estop_state:
            if estop_enabled:
               self.get_logger().info("🟢 E-STOP: DISENGAGED (SYSTEM ENABLED)")
            else:
                self.get_logger().warn("🔴 E-STOP: ENGAGED (SYSTEM STOPPED)")
            self._last_estop_state = estop_enabled

        # Apply behavior
        if not estop_enabled:
            self._set_relay(False)
            self._publish_stop()
            return
        else:
            self._set_relay(True)

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def _publish_stop(self):
        self.pub.publish(Twist())

    def destroy_node(self):
        try:
            self._set_relay(False)
            self.cb_thr.cancel()
            self.cb_yaw.cancel()
            self.cb_estop.cancel()
            self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RcToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import time # used for sleep() during ESC arming
import platform

import rclpy
from rclpy.node import Node # imports Node class to make ROS 2 Nodes
from geometry_msgs.msg import Twist # imports Twist message for /cmd_vel
from rclpy.duration import Duration

# --- Detect whether we're on a Raspberry Pi (ARM) ---
RUNNING_ON_PI = (
    platform.system() == "Linux"
    and platform.machine().startswith(("arm", "aarch64"))
)
"""
RUNNING_ON_PI is true if:
 -OS is Linux
 -CPU arch starts with arm or aarch64 (typical for Raspberry Pi)
""" 

# --- Try to import pigpio (works on Pi with deps installed) ---
try:
    if RUNNING_ON_PI:
        import pigpio
    else:
        raise ImportError("Not running on Pi, force mock")
except Exception:
    pigpio = None # signals "no real hardware"

# Ensures v is limited between lo and hi to keep PWM in safe bounds
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ThrusterDriver(Node):
    """
    Differential-drive thruster driver.
    Subscribes to /cmd_vel (geometry_msgs/Twist)
      - linear.x  => surge  (forward/back) in [-1, 1]
      - angular.z => yaw    (left/right)   in [-1, 1]
    Mixes to left/right thrusters, converts to PWM µs, applies:
      - watchdog
      - reverse lockout
      - ramp limiting
    Outputs via pigpio GPIO (or mock on non-Pi).
    """

    def __init__(self):
        super().__init__('thruster_driver') # calls parent node constructor and registers node w/ ros 2 under name "thruster_driver"

        # --- Parameters ---
        # makes a ros 2 parameter accessible via "ros2 param set", YAML, or Launch files
        self.declare_parameter('max_forward_us', 1900)       # PWM Upper Bound for full fwd
        self.declare_parameter('min_reverse_us', 1100)       # PMW Lower Bound for full back
        self.declare_parameter('neutral_us', 1500)           # stop pulse
        self.declare_parameter('deadband_us', 20)            # small neutral/stop tolerance
        self.declare_parameter('mix_gain', 1.0)              # overall throttle scale for mixed surge + yaw command
        self.declare_parameter('yaw_mix', 0.6)               # yaw contribution: how strongly yaw modifies left/right thrust
        self.declare_parameter('left_gpio', 12)              # GPIO 12 for left ESC
        self.declare_parameter('right_gpio', 13)             # GPIO 13 for right ESC
        self.declare_parameter('left_sign', 1.0)             # flip left thruster direction if needed
        self.declare_parameter('right_sign', 1.0)            # flip right thruster direction if needed
        self.declare_parameter('watchdog_ms', 500)           # Stop if no cmd within 0.5 s, publishing with cmd_vel topic needs to be fast enough (10-50 Hz) (-r 20)
        self.declare_parameter('arm_neutral_ms', 2000)       # Hold neutral/stop when direction flips
        self.declare_parameter('ramp_us_per_s', 400.0)       # Max PWM chnage rate
        self.declare_parameter('reverse_lockout_ms', 200)    # Neutral hold when direction flips

        # Reading parameters into member variables to use throughout class
        # "get_parameter(name).value" returns the current value
        self.max_forward_us     = int(self.get_parameter('max_forward_us').value)
        self.min_reverse_us     = int(self.get_parameter('min_reverse_us').value)
        self.neutral_us         = int(self.get_parameter('neutral_us').value)
        self.deadband_us        = int(self.get_parameter('deadband_us').value)
        self.mix_gain           = float(self.get_parameter('mix_gain').value)
        self.yaw_mix            = float(self.get_parameter('yaw_mix').value)
        self.left_gpio          = int(self.get_parameter('left_gpio').value)
        self.right_gpio         = int(self.get_parameter('right_gpio').value)
        self.left_sign          = float(self.get_parameter('left_sign').value)
        self.right_sign         = float(self.get_parameter('right_sign').value)
        self.watchdog_ms        = int(self.get_parameter('watchdog_ms').value)
        self.arm_neutral_ms     = int(self.get_parameter('arm_neutral_ms').value)
        self.ramp_us_per_s      = float(self.get_parameter('ramp_us_per_s').value)
        self.reverse_lockout_ms = int(self.get_parameter('reverse_lockout_ms').value)

        # --- GPIO / Mock setup ---
        if pigpio is not None:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpiod not running. Start it: sudo systemctl start pigpiod")

            self.hw_ok = True
            self.get_logger().info("✅ Using REAL pigpio on Raspberry Pi GPIO.")
            self.pi.set_mode(self.left_gpio, pigpio.OUTPUT)
            self.pi.set_mode(self.right_gpio, pigpio.OUTPUT)
        else:
            self.hw_ok = False
            self.get_logger().warn("⚠ Using MOCK GPIO output (not on Pi or pigpio not installed).")

            class MockPi:
                def set_mode(self, gpio, mode):
                    pass
                def set_servo_pulsewidth(self, gpio, us):
                    print(f"[MOCK] GPIO {gpio} pulsewidth -> {us} us")
                def stop(self):
                    pass

            self.pi = MockPi() # Assigns the mock board to self.pi, so the rest of the code can treat real and mock identically.

        # --- Safety / arming ---
        self.last_cmd_time = self.get_clock().now() # Gets current ROS time; used in watchdog
        self._write_us("left", self.neutral_us) # Send Neutral pulse to left ESC
        self._write_us("right", self.neutral_us) # Send Neutral pulse to right ESC
        self.get_logger().info("Arming ESCs at neutral...")
        time.sleep(self.arm_neutral_ms / 1000.0) # Pauses for arm_neutral_ms milliseconds
        self.get_logger().info("Arming complete.")

        # Desired vs Actual output PWM (µs)
        self.des_left_us = self.neutral_us
        self.des_right_us = self.neutral_us
        self.out_left_us = self.neutral_us
        self.out_right_us = self.neutral_us

        # Reverse lockout timers + direction states
        now = self.get_clock().now()
        self.left_lockout_until = now
        self.right_lockout_until = now
       
        # Track last "direction" for each thruster: -1 reverse, 0 neutral, +1 forward
        self.left_dir = 0
        self.right_dir = 0

        # Timer period (keep in sync with create_timer)
        self.control_dt = 0.05 # 20 Hz

        # ROS I/O: creates ROS subscription to cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.timer = self.create_timer(self.control_dt, self.on_timer) # 20 Hz watchdog, watchdog stops thrusters if commands stop coming


    # --- Helpers ---

    def _write_us(self, which: str, us: int):
        us = int(us)
        us = clamp(us, self.min_reverse_us, self.max_forward_us)

        gpio = self.left_gpio if which == "left" else self.right_gpio
        self.pi.set_servo_pulsewidth(gpio, us)

    def _scale_to_us(self, x: float) -> int:
        # x in [-1, 1] -> µs using neutral and bounds
        if abs(x) < 1e-3:
            return self.neutral_us
        if x > 0:
            return int(self.neutral_us + x * (self.max_forward_us - self.neutral_us))
        return int(self.neutral_us + x * (self.neutral_us - self.min_reverse_us))

    def _dir_from_us(self, us: int) -> int:
        if us > self.neutral_us + self.deadband_us:
            return 1
        if us < self.neutral_us - self.deadband_us:
            return -1
        return 0

    def _slew(self, current: int, target: int, max_step: float) -> int:
        if target > current:
            return int(min(current + max_step, target))
        if target < current:
            return int(max(current - max_step, target))
        return current

    # --- ROS callbacks ---

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        surge = clamp(msg.linear.x, -1.0, 1.0)
        yaw = clamp(msg.angular.z, -1.0, 1.0)

        # Differential mixing (tank steer)
        left_cmd  = self.mix_gain * clamp(surge + self.yaw_mix * yaw, -1.0, 1.0)
        right_cmd = self.mix_gain * clamp(surge - self.yaw_mix * yaw, -1.0, 1.0)

        # Apply per-thruster direction sign flips (mounting/wiring)
        left_cmd *= self.left_sign
        right_cmd *= self.right_sign

        left_us = self._scale_to_us(left_cmd)
        right_us = self._scale_to_us(right_cmd)

        # Deadband around neutral
        if abs(left_us - self.neutral_us) < self.deadband_us:
            left_us = self.neutral_us
        if abs(right_us - self.neutral_us) < self.deadband_us:
            right_us = self.neutral_us

        # Store desired outputs (timer will apply safety/smoothing then output)
        self.des_left_us = left_us
        self.des_right_us = right_us

    def on_timer(self):
        now = self.get_clock().now()

        # Watchdog: if no cmd recently, set desired to neutral
        dt_ms = (now - self.last_cmd_time).nanoseconds / 1e6
        if dt_ms > self.watchdog_ms:
            self.des_left_us = self.neutral_us
            self.des_right_us = self.neutral_us

        # Reverse lockout detection
        des_left_dir = self._dir_from_us(self.des_left_us)
        des_right_dir = self._dir_from_us(self.des_right_us)

        # If direction flips (FWD<->REV), start lockout window
        if (self.left_dir != 0) and (des_left_dir != 0) and (des_left_dir != self.left_dir):
            self.left_lockout_until  = now + Duration(nanoseconds=int(self.reverse_lockout_ms * 1e6))
        if (self.right_dir != 0) and (des_right_dir != 0) and (des_right_dir != self.right_dir):
            self.right_lockout_until = now + Duration(nanoseconds=int(self.reverse_lockout_ms * 1e6))

        # Apply lockout: force neutral while lockout active
        left_target = self.neutral_us if now < self.left_lockout_until else self.des_left_us
        right_target = self.neutral_us if now < self.right_lockout_until else self.des_right_us

        # Ramp limiting
        max_step = self.ramp_us_per_s * self.control_dt
        self.out_left_us = self._slew(self.out_left_us, left_target, max_step)
        self.out_right_us = self._slew(self.out_right_us, right_target, max_step)

        # Update direction states based on OUTPUT (not desired)
        self.left_dir = self._dir_from_us(self.out_left_us)
        self.right_dir = self._dir_from_us(self.out_right_us)

        # Write final outputs
        self._write_us("left", self.out_left_us)
        self._write_us("right", self.out_right_us)

    def destroy_node(self):
        # Ensure neutral on shutdown
        try:
            self._write_us("left", self.neutral_us)
            self._write_us("right", self.neutral_us)
            if self.hw_ok:
                self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

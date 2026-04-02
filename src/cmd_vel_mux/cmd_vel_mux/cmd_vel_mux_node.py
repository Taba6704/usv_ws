#!/usr/bin/env python3
import time

import pigpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

"""
Purpose: Command multiplexer node used to decide which commands
published from two different nodes will be published to and read
by the thruster_driver node based on the state of the SWA switch
on the RC Controller.

RC node ---------> /cmd_vel_rc ----\
                                    \
                                     > MUX NODE ---> /cmd_vel ---> thruster_driver
                                    /
Path Planner ---> /cmd_vel_auto ---/

CH6 switch ------------------------>/
"""

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')

        # Parameters
        self.declare_parameter('gpio_mode', 10)  # CH6 input - pin 19
        self.declare_parameter('mode_threshold_us', 1500) # PWM threshold to decide mode

        self.declare_parameter('timeout_s', 0.5) # How long the node will trust an old command before treating it as stale
        self.declare_parameter('publish_hz', 20.0) # 

        self.declare_parameter('topic_rc', '/cmd_vel_rc')
        self.declare_parameter('topic_auto', '/cmd_vel_auto')
        self.declare_parameter('topic_out', '/cmd_vel')

        # Read parameters
        self.gpio_mode = int(self.get_parameter('gpio_mode').value)
        self.mode_threshold = int(self.get_parameter('mode_threshold_us').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)

        topic_rc = str(self.get_parameter('topic_rc').value)
        topic_auto = str(self.get_parameter('topic_auto').value)
        topic_out = str(self.get_parameter('topic_out').value)

        publish_hz = float(self.get_parameter('publish_hz').value)

        # Pigpio setup
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio not running. Start: sudo systemctl start pigpiod")

        self.pi.set_mode(self.gpio_mode, pigpio.INPUT)

        # Mode PWM
        self._mode_us = None # Holds the measured pulse width
        self._mode_rise_tick = None # Stores the timestamp of the last rising edge

        self.cb_mode = self.pi.callback(self.gpio_mode, pigpio.EITHER_EDGE, self._cb_mode) # "Whenever GPIO 24 changes state, call the function _cb_mode."

        # Subscribers
        self.sub_rc = self.create_subscription(Twist, topic_rc, self._cb_rc, 10)
        self.sub_auto = self.create_subscription(Twist, topic_auto, self._cb_auto, 10)

        # --- Publisher ---
        self.pub = self.create_publisher(Twist, topic_out, 10)

        # Store latest commands
        self._rc_cmd = Twist()
        self._auto_cmd = Twist()

        # Store the Wall-clock time when latest RC and autonomous messages were recieved
        self._rc_time = time.time()
        self._auto_time = time.time()

        # Stores previous mode state for logger msgs
        self._last_mode = None

        # Timer
        self.timer = self.create_timer(1.0 / publish_hz, self._tick)

        # Start up message
        self.get_logger().info("cmd_vel_mux_node started")

    # Callbacks
    def _cb_mode(self, gpio, level, tick):
        """
        This function runs whenever the CH6 input chnages state.
        """
        if level == 1:
            self._mode_rise_tick = tick
        elif level == 0 and self._mode_rise_tick is not None:
            self._mode_us = pigpio.tickDiff(self._mode_rise_tick, tick)

    def _cb_rc(self, msg):
        """
        Whenever a new RC Twist message comes in:
            - Save it as a latest RC command
            - Record when it arrived
        """
        self._rc_cmd = msg
        self._rc_time = time.time()

    def _cb_auto(self, msg):
        """
        Whenever a new autonomous Twist message comes in:
            - Save it as a latest auto command
            - Record when it arrived
        """
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
        rc_mode = self._mode_us > self.mode_threshold # if CH6 PWM is greter than 1500 us the RC control

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
                self.pub.publish(Twist()) # if the last RC command is too old, publish zero
            else:
                self.pub.publish(self._rc_cmd) # publish RC command to /cmd_vel
        else:
            # Autonomous mode
            if (now - self._auto_time) > self.timeout_s:
                self.pub.publish(Twist()) # if the last auto command is too old, publish zero
            else:
                self.pub.publish(self._auto_cmd) # Publish auto command to /cmd_vel

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
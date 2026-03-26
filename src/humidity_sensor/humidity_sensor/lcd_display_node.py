#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import board
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd


class LCDDisplayNode(Node):
    def __init__(self):
        super().__init__('lcd_display_node')

        # LCD size
        self.lcd_columns = 16
        self.lcd_rows = 2

        # Parallel LCD pin setup
        self.lcd_rs = digitalio.DigitalInOut(board.D26) # GPIO26 = pin 37
        self.lcd_en = digitalio.DigitalInOut(board.D19) # GPIO19 = pin 35
        self.lcd_d4 = digitalio.DigitalInOut(board.D25) # GPIO25 = pin 22
        self.lcd_d5 = digitalio.DigitalInOut(board.D24) # GPIO24 = pin 18
        self.lcd_d6 = digitalio.DigitalInOut(board.D22) # GPIO22 = pin 15
        self.lcd_d7 = digitalio.DigitalInOut(board.D5)  # GPIO5  = pin 29

        # Initialize LCD
        self.lcd = characterlcd.Character_LCD_Mono(
            self.lcd_rs,
            self.lcd_en,
            self.lcd_d4,
            self.lcd_d5,
            self.lcd_d6,
            self.lcd_d7,
            self.lcd_columns,
            self.lcd_rows
        )

        self.lcd.clear()

        # Latest received values
        self.latest_temp = None
        self.latest_humidity = None

        # ROS subscriptions
        self.create_subscription(Float32, '/temperature_c', self.temp_callback, 10)
        self.create_subscription(Float32, '/humidity_percent', self.humidity_callback, 10)

        # Refresh LCD periodically
        self.timer = self.create_timer(0.5, self.update_display)

        self.get_logger().info('LCD display node started.')

    def temp_callback(self, msg: Float32):
        self.latest_temp = msg.data

    def humidity_callback(self, msg: Float32):
        self.latest_humidity = msg.data

    def update_display(self):
        if self.latest_temp is None or self.latest_humidity is None:
            line1 = 'TEMP: -- C'
            line2 = 'HUMIDITY: -- %'
        else:
            temp_str = f'{self.latest_temp:.1f}'
            hum_str = f'{self.latest_humidity:.1f}'

            # Keep each line <= 16 chars
            line1 = f'TEMP: {temp_str} C'
            line2 = f'HUMIDITY: {hum_str} %'

        # Pad or trim to 16 chars
        line1 = line1[:16].ljust(16)
        line2 = line2[:16].ljust(16)

        self.lcd.message = f'{line1}\n{line2}'

    def cleanup(self):
        try:
            self.lcd.clear()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LCDDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()